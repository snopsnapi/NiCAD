// Golden-data regression for the FitSpline port of rt_spline.interpolate()
// (Nikaishi ReverseTools "Studio Spline"). The reference numbers in
// tests/data/spline_golden.json are produced by tools/golden/spline_golden.py
// (scipy-based reimplementation of the Python original); the C++ side must
// match them to 1e-9, evaluate back through the points and the G1/G2
// constraints, and round-trip into an OCCT curve that agrees with the
// hand-written de Boor evaluator.

#include "modeling/FitSpline.h"

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <Extrema_ExtPC.hxx>
#include <GeomAdaptor_Curve.hxx>

#include <cmath>
#include <fstream>
#include <string>
#include <vector>

using namespace materializr;

namespace {

std::vector<nlohmann::json> loadGolden() {
    std::ifstream f(std::string(MZR_TEST_DATA_DIR) + "/spline_golden.json");
    EXPECT_TRUE(f.good()) << "missing tests/data/spline_golden.json "
                             "(run tools/golden/spline_golden.py)";
    if (!f.good()) return {};
    nlohmann::json j;
    f >> j;
    return j.get<std::vector<nlohmann::json>>();
}

std::vector<glm::dvec3> toPoints(const nlohmann::json& arr) {
    std::vector<glm::dvec3> out;
    for (const auto& p : arr) out.emplace_back(p[0].get<double>(), p[1].get<double>(), p[2].get<double>());
    return out;
}

fitspline::Curve runCase(const nlohmann::json& tc) {
    std::vector<fitspline::Constraint> cons;
    for (const auto& c : tc["constraints"]) {
        fitspline::Constraint fc;
        fc.index = c["index"].get<int>();
        const auto& d1 = c["d1"];
        fc.d1 = glm::dvec3(d1[0].get<double>(), d1[1].get<double>(), d1[2].get<double>());
        if (!c["d2"].is_null()) {
            const auto& d2 = c["d2"];
            fc.d2 = glm::dvec3(d2[0].get<double>(), d2[1].get<double>(), d2[2].get<double>());
        }
        cons.push_back(fc);
    }
    fitspline::Curve curve;
    EXPECT_TRUE(fitspline::interpolate(toPoints(tc["points"]), cons, tc["degree_in"].get<int>(), curve));
    return curve;
}

void expectNear(double got, double ref, double tol, const std::string& what) {
    EXPECT_LE(std::fabs(got - ref), tol * (1.0 + std::fabs(ref)))
        << what << ": got " << got << " ref " << ref;
}

TEST(FitSpline, MatchesGoldenData) {
    for (const auto& tc : loadGolden()) {
        const std::string name = tc["name"].get<std::string>();
        fitspline::Curve curve = runCase(tc);
        if (curve.knots.empty()) continue;  // interpolate() failed; runCase already reported
        const auto& ref = tc["out"];
        EXPECT_EQ(curve.degree, ref["degree"].get<int>()) << name;
        ASSERT_EQ(curve.knots.size(), ref["knots"].size()) << name;
        for (std::size_t i = 0; i < curve.knots.size(); ++i)
            expectNear(curve.knots[i], ref["knots"][i].get<double>(), 1e-9, name + " knot");
        ASSERT_EQ(curve.poles.size(), ref["poles"].size()) << name;
        for (std::size_t i = 0; i < curve.poles.size(); ++i)
            for (int c = 0; c < 3; ++c)
                expectNear(curve.poles[i][c], ref["poles"][i][c].get<double>(), 1e-9, name + " pole");
        ASSERT_EQ(curve.params.size(), ref["params"].size()) << name;
        for (std::size_t i = 0; i < curve.params.size(); ++i)
            expectNear(curve.params[i], ref["params"][i].get<double>(), 1e-9, name + " param");
    }
}

TEST(FitSpline, PassesThroughPointsAndDerivatives) {
    for (const auto& tc : loadGolden()) {
        const std::string name = tc["name"].get<std::string>();
        fitspline::Curve curve = runCase(tc);
        if (curve.knots.empty()) continue;
        const std::vector<glm::dvec3> pts = toPoints(tc["points"]);
        for (std::size_t i = 0; i < pts.size(); ++i) {
            const glm::dvec3 got = fitspline::evaluate(curve, curve.params[(std::size_t)i]);
            EXPECT_NEAR(glm::length(got - pts[i]), 0.0, 1e-9) << name << " point " << i;
        }
        for (const auto& c : tc["constraints"]) {
            const int idx = c["index"].get<int>();
            const glm::dvec3 d1(c["d1"][0].get<double>(), c["d1"][1].get<double>(), c["d1"][2].get<double>());
            const glm::dvec3 got1 = fitspline::evaluate(curve, curve.params[(std::size_t)idx], 1);
            EXPECT_NEAR(glm::length(got1 - d1), 0.0, 1e-7) << name << " d1";
            if (!c["d2"].is_null()) {
                const glm::dvec3 d2(c["d2"][0].get<double>(), c["d2"][1].get<double>(), c["d2"][2].get<double>());
                const glm::dvec3 got2 = fitspline::evaluate(curve, curve.params[(std::size_t)idx], 2);
                EXPECT_NEAR(glm::length(got2 - d2), 0.0, 1e-6) << name << " d2";
            }
        }
    }
}

TEST(FitSpline, OcctCurveAgreesWithOwnEvaluator) {
    for (const auto& tc : loadGolden()) {
        const std::string name = tc["name"].get<std::string>();
        fitspline::Curve curve = runCase(tc);
        if (curve.knots.empty()) continue;
        Handle(Geom_BSplineCurve) occt = fitspline::toOcct(curve);
        ASSERT_FALSE(occt.IsNull()) << name;
        for (int i = 0; i <= 20; ++i) {
            const double t = (double)i / 20.0;
            const gp_Pnt q = occt->Value(t);
            const glm::dvec3 got = fitspline::evaluate(curve, t);
            EXPECT_NEAR(q.Distance(gp_Pnt(got.x, got.y, got.z)), 0.0, 1e-9) << name << " t=" << t;
        }
    }
}

TEST(FitSpline, RejectsBadInput) {
    fitspline::Curve curve;
    EXPECT_FALSE(fitspline::interpolate({}, {}, 3, curve));
    EXPECT_FALSE(fitspline::interpolate({glm::dvec3(1.0)}, {}, 3, curve));
    EXPECT_FALSE(fitspline::interpolate({glm::dvec3(1.0), glm::dvec3(1.0), glm::dvec3(2.0)}, {}, 3, curve));
}

TEST(FitSpline, ClosedInterpolatesPeriodically) {
    // 6 points on an axis-aligned ellipse in the z=7 plane.
    std::vector<glm::dvec3> pts;
    for (int i = 0; i < 6; ++i) {
        const double a = 2.0 * M_PI * (double)i / 6.0;
        pts.emplace_back(12.3 * std::cos(a), 7.7 * std::sin(a), 7.0);
    }
    Handle(Geom_BSplineCurve) curve = fitspline::interpolateClosed(pts);
    ASSERT_FALSE(curve.IsNull());
    EXPECT_TRUE(curve->IsPeriodic());
    GeomAdaptor_Curve adaptor(curve);
    for (const glm::dvec3& p : pts) {
        const gp_Pnt q(p.x, p.y, p.z);
        Extrema_ExtPC ext(q, adaptor);
        ASSERT_TRUE(ext.IsDone());
        double dmin = 1e300;
        for (int i = 1; i <= ext.NbExt(); ++i)
            dmin = std::min(dmin, std::sqrt(ext.SquareDistance(i)));
        EXPECT_LT(dmin, 1e-7);
    }
}

} // namespace
