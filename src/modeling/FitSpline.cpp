// Through-points B-spline with G1/G2 constraints: C++ port of the
// interpolate() math from the Nikaishi ReverseTools workbench
// (rt_spline.py, "Studio Spline"). Chord-length parameters, collocation
// matrix built from hand-written B-spline basis derivatives
// (The NURBS Book A2.1 FindSpan / A2.3 DersBasisFuns), Gaussian
// elimination with partial pivoting. Note: rt_spline falls back to
// least squares on a singular system; here we just fail (no lstsq).

#include "modeling/FitSpline.h"

#include <TColStd_Array1OfInteger.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColgp_HArray1OfPnt.hxx>
#include <GeomAPI_Interpolate.hxx>

#include <algorithm>
#include <cmath>

namespace materializr {
namespace fitspline {
namespace {

// A2.1 FindSpan: index i with knots[i] <= u < knots[i+1]; at u == last
// knot returns the last non-zero span (the scipy convention).
int findSpan(int p, double u, const std::vector<double>& knots) {
    const int n = (int)knots.size() - p - 2;  // last non-zero span index
    if (u >= knots.back()) return n;
    if (u <= knots.front()) return p;
    int low = p, high = n + 1;
    int mid = (low + high) / 2;
    while (u < knots[mid] || u >= knots[mid + 1]) {
        if (u < knots[mid]) high = mid;
        else low = mid;
        mid = (low + high) / 2;
    }
    return mid;
}

// A2.3 DersBasisFuns: ders[k][j] = k-th derivative of N[span-p+j, p] at u,
// k = 0..nd. Mirror of the reference implementation; the j1 lower bound is
// 1 when rk >= -1 (verified against scipy.interpolate.BSpline).
void dersBasisFuns(int span, double u, int p, int nd, const std::vector<double>& knots,
                   std::vector<std::vector<double>>& ders) {
    ders.assign(nd + 1, std::vector<double>(p + 1, 0.0));
    std::vector<std::vector<double>> ndu(p + 1, std::vector<double>(p + 1, 0.0));
    std::vector<double> left(p + 1), right(p + 1);
    ndu[0][0] = 1.0;
    for (int j = 1; j <= p; ++j) {
        left[j] = u - knots[span + 1 - j];
        right[j] = knots[span + j] - u;
        double saved = 0.0;
        for (int r = 0; r < j; ++r) {
            ndu[j][r] = right[r + 1] + left[j - r];
            const double temp = ndu[r][j - 1] / ndu[j][r];
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }
    for (int j = 0; j <= p; ++j) ders[0][j] = ndu[j][p];

    std::vector<std::vector<double>> a(2, std::vector<double>(p + 2, 0.0));
    for (int r = 0; r <= p; ++r) {
        int s1 = 0, s2 = 1;
        a[0][0] = 1.0;
        for (int k = 1; k <= nd; ++k) {
            double d = 0.0;
            const int rk = r - k;
            const int pk = p - k;
            if (r >= k) {
                a[s2][0] = a[s1][0] / ndu[pk + 1][rk];
                d = a[s2][0] * ndu[rk][pk];
            }
            const int j1 = rk >= -1 ? 1 : -rk;
            const int j2 = r - 1 <= pk ? k - 1 : p - r;
            for (int j = j1; j <= j2; ++j) {
                a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][rk + j];
                d += a[s2][j] * ndu[rk + j][pk];
            }
            if (r <= pk) {
                a[s2][k] = -a[s1][k - 1] / ndu[pk + 1][r];
                d += a[s2][k] * ndu[r][pk];
            }
            ders[k][r] = d;
            std::swap(s1, s2);
        }
    }
    int r = p;
    for (int k = 1; k <= nd; ++k) {
        for (int j = 0; j <= p; ++j) ders[k][j] *= (double)r;
        r *= p - k;
    }
}

// Gaussian elimination with partial pivoting on the n x n matrix a
// (rows) against nrhs right-hand columns in b. False if singular.
bool gaussSolve(std::vector<std::vector<double>>& a, std::vector<std::vector<double>>& b) {
    const int n = (int)a.size();
    const int nrhs = (int)b[0].size();
    for (int col = 0; col < n; ++col) {
        int piv = col;
        for (int r = col + 1; r < n; ++r)
            if (std::fabs(a[r][col]) > std::fabs(a[piv][col])) piv = r;
        if (std::fabs(a[piv][col]) < 1e-14) return false;
        std::swap(a[col], a[piv]);
        std::swap(b[col], b[piv]);
        for (int r = col + 1; r < n; ++r) {
            const double f = a[r][col] / a[col][col];
            if (f == 0.0) continue;
            for (int c = col; c < n; ++c) a[r][c] -= f * a[col][c];
            for (int c = 0; c < nrhs; ++c) b[r][c] -= f * b[col][c];
        }
    }
    for (int i = n - 1; i >= 0; --i) {
        for (int j = 0; j < nrhs; ++j) {
            double s = b[i][j];
            for (int c = i + 1; c < n; ++c) s -= a[i][c] * b[c][j];
            b[i][j] = s / a[i][i];
        }
    }
    return true;
}

} // namespace

bool interpolate(const std::vector<glm::dvec3>& points, const std::vector<Constraint>& constraints,
                 int degree, Curve& out) {
    const int np = (int)points.size();
    if (np < 2) return false;
    for (int i = 0; i + 1 < np; ++i)
        if (glm::length(points[i + 1] - points[i]) < 1e-12) return false;

    // Chord-length parameterisation in [0,1].
    std::vector<double> t(np);
    double total = 0.0;
    for (int i = 1; i < np; ++i) total += glm::length(points[i] - points[i - 1]);
    t[0] = 0.0;
    for (int i = 1; i < np; ++i)
        t[i] = t[i - 1] + glm::length(points[i] - points[i - 1]) / total;

    // Collocation sites: position at every point, plus derivative rows.
    std::vector<double> sites;
    std::vector<int> kinds;
    std::vector<glm::dvec3> rhs;
    int multiplicity = 1;
    for (int k = 0; k < np; ++k) {
        sites.push_back(t[k]); kinds.push_back(0); rhs.push_back(points[k]);
        const Constraint* c = nullptr;
        for (const auto& cand : constraints)
            if (cand.index == k) c = &cand;
        if (c) {
            sites.push_back(t[k]); kinds.push_back(1); rhs.push_back(c->d1);
            multiplicity = std::max(multiplicity, 2);
            if (c->d2) {
                sites.push_back(t[k]); kinds.push_back(2); rhs.push_back(*c->d2);
                multiplicity = 3;
            }
        }
    }
    const int n = (int)sites.size();
    int p = std::min(std::max(degree, multiplicity), n - 1);
    if (np == 2) p = n - 1;  // a two-point blend stays a single span

    // Averaged inner knots, clamped ends (rt_spline.py).
    std::vector<double> knots(p + 1, 0.0);
    for (int j = 1; j < n - p; ++j) {
        double s = 0.0;
        for (int q = j; q < j + p; ++q) s += sites[q];
        knots.push_back(s / p);
    }
    knots.insert(knots.end(), p + 1, 1.0);

    std::vector<std::vector<double>> a(n, std::vector<double>(n, 0.0));
    std::vector<std::vector<double>> b(n, std::vector<double>(3, 0.0));
    std::vector<std::vector<double>> ders;
    for (int row = 0; row < n; ++row) {
        const int span = findSpan(p, sites[row], knots);
        dersBasisFuns(span, sites[row], p, kinds[row], knots, ders);
        for (int j = 0; j <= p; ++j) a[row][span - p + j] = ders[kinds[row]][j];
        b[row][0] = rhs[row].x; b[row][1] = rhs[row].y; b[row][2] = rhs[row].z;
    }
    if (!gaussSolve(a, b)) return false;

    out.degree = p;
    out.knots = knots;
    out.params = t;
    out.poles.resize(n);
    for (int i = 0; i < n; ++i) out.poles[i] = glm::dvec3(b[i][0], b[i][1], b[i][2]);
    return true;
}

Handle(Geom_BSplineCurve) interpolateClosed(const std::vector<glm::dvec3>& points) {
    if (points.size() < 3) return nullptr;
    try {
        Handle(TColgp_HArray1OfPnt) arr =
            new TColgp_HArray1OfPnt(1, (Standard_Integer)points.size());
        for (std::size_t i = 0; i < points.size(); ++i)
            arr->SetValue((Standard_Integer)i + 1,
                          gp_Pnt(points[i].x, points[i].y, points[i].z));
        GeomAPI_Interpolate ip(arr, Standard_True, 1e-7);
        ip.Perform();
        if (!ip.IsDone()) return nullptr;
        return ip.Curve();
    } catch (...) {
        return nullptr;
    }
}

Handle(Geom_BSplineCurve) toOcct(const Curve& c) {
    const int n = (int)c.poles.size();
    if (n == 0 || c.degree < 1) return nullptr;
    try {
        TColgp_Array1OfPnt poles(1, n);
        for (int i = 0; i < n; ++i) poles.SetValue(i + 1, gp_Pnt(c.poles[i].x, c.poles[i].y, c.poles[i].z));
        // Full knot vector -> distinct values + multiplicities.
        std::vector<double> uniq;
        std::vector<int> mults;
        for (double v : c.knots) {
            if (!uniq.empty() && v - uniq.back() < 1e-12) mults.back() += 1;
            else { uniq.push_back(v); mults.push_back(1); }
        }
        TColStd_Array1OfReal knots(1, (Standard_Integer)uniq.size());
        TColStd_Array1OfInteger multArr(1, (Standard_Integer)mults.size());
        for (std::size_t i = 0; i < uniq.size(); ++i) {
            knots.SetValue((Standard_Integer)i + 1, uniq[i]);
            multArr.SetValue((Standard_Integer)i + 1, mults[i]);
        }
        return new Geom_BSplineCurve(poles, knots, multArr, c.degree, Standard_False);
    } catch (...) {
        return nullptr;
    }
}

glm::dvec3 evaluate(const Curve& c, double t, int derivative) {
    const int p = c.degree;
    const double u = std::min(std::max(t, c.knots.front()), c.knots.back());
    const int span = findSpan(p, u, c.knots);
    std::vector<std::vector<double>> ders;
    dersBasisFuns(span, u, p, derivative, c.knots, ders);
    glm::dvec3 out(0.0);
    for (int j = 0; j <= p; ++j) out += ders[derivative][j] * c.poles[span - p + j];
    return out;
}

} // namespace fitspline
} // namespace materializr
