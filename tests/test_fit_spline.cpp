// Golden-data regression for the FitSpline port of rt_spline.interpolate()
// (Nikaishi ReverseTools "Studio Spline"). The reference numbers in
// tests/data/spline_golden.json are produced by tools/golden/spline_golden.py
// (scipy-based reimplementation of the Python original); the C++ side must
// match them to 1e-9, evaluate back through the points and the G1/G2
// constraints, and round-trip into an OCCT curve that agrees with the
// hand-written de Boor evaluator.

#include "modeling/FitSpline.h"
#include "modeling/Sketch.h"

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

TEST(FitSpline, SketchInterpolate2D) {
    // The sketch sampler must now produce the exact C2 interpolant, not the
    // old G1 Catmull-Rom: the polyline passes through every control point
    // (at spanStartIdx) and the underlying curve is C2 in the interior.
    std::vector<glm::vec2> ctrl;
    ctrl.emplace_back(0.0f, 0.0f);
    ctrl.emplace_back(10.0f, 14.0f);
    ctrl.emplace_back(24.0f, 5.0f);
    ctrl.emplace_back(33.0f, 21.0f);
    ctrl.emplace_back(48.0f, 9.0f);
    ctrl.emplace_back(60.0f, 16.0f);

    std::vector<int> spanStart;
    const std::vector<glm::vec2> samp =
        Sketch::interpolate2D(ctrl, 8, false, &spanStart);

    // Open: n-1 span starts plus the trailing end marker = one entry per
    // control point (the marker indexes the last sample = the last point).
    ASSERT_EQ(spanStart.size(), ctrl.size());
    for (size_t i = 0; i < ctrl.size(); ++i) {
        ASSERT_LT(static_cast<size_t>(spanStart[i]), samp.size());
        EXPECT_NEAR(glm::length(samp[static_cast<size_t>(spanStart[i])] - ctrl[i]),
                    0.0, 1e-4)
            << "open sample at span " << i;
    }

    // C2 across every interior control point: second derivatives agree at
    // params[i] +/- 1e-6 (1e-3 relative). interpolate2D builds the curve
    // with the same chord parameterisation from the same points.
    std::vector<glm::dvec3> pts3;
    for (const glm::vec2& p : ctrl) pts3.emplace_back(p.x, p.y, 0.0);
    fitspline::Curve c;
    ASSERT_TRUE(fitspline::interpolate(pts3, {}, 3, c));
    ASSERT_EQ(c.params.size(), ctrl.size());
    for (size_t i = 1; i + 1 < ctrl.size(); ++i) {
        const double t = c.params[i];
        const glm::dvec3 d2l = fitspline::evaluate(c, t - 1e-6, 2);
        const glm::dvec3 d2r = fitspline::evaluate(c, t + 1e-6, 2);
        const double scale = 1.0 + std::max(glm::length(d2l), glm::length(d2r));
        EXPECT_LE(glm::length(d2l - d2r), 1e-3 * scale) << "C2 at point " << i;
    }

    // Closed: the polyline wraps back to the first point and every span
    // starts on its control point, the last span closing on the first.
    std::vector<glm::vec2> cctrl;
    cctrl.emplace_back(0.0f, 0.0f);
    cctrl.emplace_back(20.0f, 4.0f);
    cctrl.emplace_back(34.0f, 18.0f);
    cctrl.emplace_back(18.0f, 30.0f);
    cctrl.emplace_back(-4.0f, 16.0f);
    std::vector<int> cspan;
    const std::vector<glm::vec2> csamp =
        Sketch::interpolate2D(cctrl, 8, true, &cspan);
    ASSERT_FALSE(csamp.empty());
    EXPECT_NEAR(glm::length(csamp.front() - csamp.back()), 0.0, 1e-4);
    ASSERT_EQ(cspan.size(), cctrl.size() + 1);
    for (size_t i = 0; i < cctrl.size(); ++i) {
        ASSERT_LT(static_cast<size_t>(cspan[i]), csamp.size());
        EXPECT_NEAR(glm::length(csamp[static_cast<size_t>(cspan[i])] - cctrl[i]),
                    0.0, 1e-4)
            << "closed sample at span " << i;
    }
}

} // namespace
