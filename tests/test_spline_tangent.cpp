// TZ-22: spline Tangent/G2 contacts. A Tangent (G1) or G2 constraint with
// entityA = spline pins the spline end NEAREST the reference curve onto that
// curve (line/circle/arc/spline). Sketch::enforceSplineContacts (post-solver
// pass) projects the bound end onto the curve; Sketch::splineEndConditions
// turns the contact into fitspline end-derivative rows so the interpolant
// leaves the end along the curve's tangent (and, for G2, with its curvature).

#include "modeling/Sketch.h"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using namespace materializr;

namespace {

int addContact(Sketch& sk, ConstraintType type, int splineId, int curveId) {
    Constraint c{}; // value fields unused for Tangent/G2
    c.type = type;
    c.entityA = splineId;
    c.entityB = curveId;
    return sk.addConstraint(c);
}

const Constraint* findConstraint(const Sketch& sk, ConstraintType type) {
    for (const auto& c : sk.getConstraints())
        if (c.type == type) return &c;
    return nullptr;
}

// Menger curvature of the polyline's last three samples (the same estimate
// the spec uses for the G2 assertion).
double endCurvature(const std::vector<glm::vec2>& samp) {
    const size_t n = samp.size();
    const glm::dvec2 a(samp[n - 3].x, samp[n - 3].y);
    const glm::dvec2 b(samp[n - 2].x, samp[n - 2].y);
    const glm::dvec2 c(samp[n - 1].x, samp[n - 1].y);
    const glm::dvec2 ab = b - a, bc = c - b, ac = c - a;
    const double cross = ab.x * bc.y - ab.y * bc.x;
    return 2.0 * std::abs(cross) /
           (glm::length(ab) * glm::length(bc) * glm::length(ac));
}

} // namespace

// Circle R=10 at the origin; an open 4-point spline whose last point sits
// near the rim. Tangent -> the end lands exactly on the rim and the drawn
// curve leaves it perpendicular to the radius.
TEST(SplineTangent, EndSnapsOntoCircleAndLeavesAlongTangent) {
    Sketch sk;
    const int center = sk.addPoint(glm::vec2(0.0f, 0.0f));
    const int circle = sk.addCircle(center, 10.0);
    std::vector<int> pts;
    pts.push_back(sk.addPoint(glm::vec2(0.0f, 8.0f)));
    pts.push_back(sk.addPoint(glm::vec2(3.0f, 9.0f)));
    pts.push_back(sk.addPoint(glm::vec2(7.0f, 8.0f)));
    pts.push_back(sk.addPoint(glm::vec2(10.5f, 3.0f)));
    const int spline = sk.addSpline(pts);
    addContact(sk, ConstraintType::Tangent, spline, circle);

    sk.enforceSplineContacts();

    const SketchPoint* end = sk.getPoint(pts.back());
    ASSERT_TRUE(end);
    EXPECT_NEAR(glm::length(glm::dvec2(end->pos.x, end->pos.y)), 10.0, 1e-5);
    const Constraint* c = findConstraint(sk, ConstraintType::Tangent);
    ASSERT_TRUE(c);
    EXPECT_TRUE(c->isSatisfied);

    const std::vector<glm::vec2> samp =
        sk.sampleSpline2D(sk.getSplines().front(), 200);
    ASSERT_GE(samp.size(), 2);
    const glm::dvec2 tangent =
        glm::normalize(glm::dvec2(samp.back().x - samp[samp.size() - 2].x,
                                  samp.back().y - samp[samp.size() - 2].y));
    const glm::dvec2 radius =
        glm::normalize(glm::dvec2(end->pos.x, end->pos.y));
    EXPECT_LT(std::abs(glm::dot(tangent, radius)), 1e-2);
}

// Same arrangement with G2: the end curvature of the drawn curve matches the
// circle's 1/R within 10%.
TEST(SplineTangent, G2MatchesCircleCurvature) {
    Sketch sk;
    const int center = sk.addPoint(glm::vec2(0.0f, 0.0f));
    const int circle = sk.addCircle(center, 10.0);
    std::vector<int> pts;
    pts.push_back(sk.addPoint(glm::vec2(0.0f, 8.0f)));
    pts.push_back(sk.addPoint(glm::vec2(3.0f, 9.0f)));
    pts.push_back(sk.addPoint(glm::vec2(7.0f, 8.0f)));
    pts.push_back(sk.addPoint(glm::vec2(10.5f, 3.0f)));
    const int spline = sk.addSpline(pts);
    addContact(sk, ConstraintType::G2, spline, circle);

    sk.enforceSplineContacts();

    const SketchPoint* end = sk.getPoint(pts.back());
    ASSERT_TRUE(end);
    EXPECT_NEAR(glm::length(glm::dvec2(end->pos.x, end->pos.y)), 10.0, 1e-5);
    const Constraint* c = findConstraint(sk, ConstraintType::G2);
    ASSERT_TRUE(c);
    EXPECT_TRUE(c->isSatisfied);

    const std::vector<glm::vec2> samp =
        sk.sampleSpline2D(sk.getSplines().front(), 200);
    ASSERT_GE(samp.size(), 3);
    EXPECT_NEAR(endCurvature(samp), 0.1, 0.01);
}

// Horizontal line; the spline end near it snaps onto the line and the drawn
// curve leaves the end horizontally.
TEST(SplineTangent, EndSnapsOntoLineLeavingHorizontally) {
    Sketch sk;
    const int a = sk.addPoint(glm::vec2(0.0f, 0.0f));
    const int b = sk.addPoint(glm::vec2(20.0f, 0.0f));
    const int line = sk.addLine(a, b);
    std::vector<int> pts;
    pts.push_back(sk.addPoint(glm::vec2(0.0f, 3.0f)));
    pts.push_back(sk.addPoint(glm::vec2(2.0f, 2.0f)));
    pts.push_back(sk.addPoint(glm::vec2(5.0f, 0.5f)));
    const int spline = sk.addSpline(pts);
    addContact(sk, ConstraintType::Tangent, spline, line);

    sk.enforceSplineContacts();

    const SketchPoint* end = sk.getPoint(pts.back());
    ASSERT_TRUE(end);
    EXPECT_NEAR(end->pos.y, 0.0, 1e-5);
    EXPECT_NEAR(end->pos.x, 5.0, 1e-5);
    const Constraint* c = findConstraint(sk, ConstraintType::Tangent);
    ASSERT_TRUE(c);
    EXPECT_TRUE(c->isSatisfied);

    const std::vector<glm::vec2> samp =
        sk.sampleSpline2D(sk.getSplines().front(), 200);
    ASSERT_GE(samp.size(), 2);
    const glm::dvec2 tangent =
        glm::normalize(glm::dvec2(samp.back().x - samp[samp.size() - 2].x,
                                  samp.back().y - samp[samp.size() - 2].y));
    EXPECT_LT(std::abs(tangent.y), 1e-2);
}

// A closed spline has no free end to bind: the contact cannot apply and the
// constraint reports unsatisfied instead of moving anything.
TEST(SplineTangent, ClosedSplineBindsNothing) {
    Sketch sk;
    const int center = sk.addPoint(glm::vec2(0.0f, 0.0f));
    const int circle = sk.addCircle(center, 10.0);
    const int p1 = sk.addPoint(glm::vec2(0.0f, 5.0f));
    const int p2 = sk.addPoint(glm::vec2(5.0f, 5.0f));
    const int p3 = sk.addPoint(glm::vec2(5.0f, 0.0f));
    const int spline = sk.addSpline({p1, p2, p3, p1});
    addContact(sk, ConstraintType::Tangent, spline, circle);

    sk.enforceSplineContacts();

    const Constraint* c = findConstraint(sk, ConstraintType::Tangent);
    ASSERT_TRUE(c);
    EXPECT_FALSE(c->isSatisfied);
    EXPECT_EQ(sk.getPoint(p1)->pos, glm::vec2(0.0f, 5.0f));
    EXPECT_EQ(sk.getPoint(p2)->pos, glm::vec2(5.0f, 5.0f));
    EXPECT_EQ(sk.getPoint(p3)->pos, glm::vec2(5.0f, 0.0f));
}

// Curvature comb (fork): curvature from the exact interpolant's d1/d2.
// A closed spline through 32 points on a circle R=10 (CCW) is very nearly
// that circle: k ~ 1/R everywhere (a cubic ripples a few % between the
// points), positive, and normal*k points at the centre.
TEST(CurvatureComb, CircleLikeSplineHasCurvatureOneOverR) {
    const double R = 10.0;
    std::vector<glm::vec2> ctrl;
    for (int i = 0; i < 32; ++i) {
        const double a = 2.0 * 3.14159265358979323846 * i / 32.0;
        ctrl.emplace_back(static_cast<float>(R * std::cos(a)),
                          static_cast<float>(R * std::sin(a)));
    }
    const std::vector<CurvatureCombSample> comb =
        Sketch::curvatureComb2D(ctrl, 8, true);
    ASSERT_EQ(comb.size(), 32u * 8u + 1u);
    double mean = 0.0;
    for (const CurvatureCombSample& s : comb) {
        mean += s.curvature / comb.size();
        EXPECT_NEAR(s.curvature, 1.0 / R, 0.05 / R);
        EXPECT_NEAR(glm::length(s.normal), 1.0, 1e-9);
        // The tooth direction points at the centre (origin).
        const glm::dvec2 toCentre = -glm::normalize(s.point);
        EXPECT_GT(glm::dot(s.normal * s.curvature, toCentre), 0.0);
    }
    EXPECT_NEAR(mean, 1.0 / R, 0.01 / R);
    // Clockwise traversal: same |k|, sign flips, tooth still toward centre.
    std::vector<glm::vec2> rev(ctrl.rbegin(), ctrl.rend());
    const std::vector<CurvatureCombSample> combCw =
        Sketch::curvatureComb2D(rev, 8, true);
    ASSERT_FALSE(combCw.empty());
    for (const CurvatureCombSample& s : combCw) {
        EXPECT_NEAR(s.curvature, -1.0 / R, 0.05 / R);
        EXPECT_GT(glm::dot(s.normal * s.curvature, -glm::normalize(s.point)), 0.0);
    }
}

// The comb follows the spline's G2 end condition: at the end bound to the
// circle R=10 it reads the circle's curvature exactly (analytic d2), and its
// end point is the drawn curve's end point.
TEST(CurvatureComb, FollowsG2EndCondition) {
    Sketch sk;
    const int center = sk.addPoint(glm::vec2(0.0f, 0.0f));
    const int circle = sk.addCircle(center, 10.0);
    std::vector<int> pts;
    pts.push_back(sk.addPoint(glm::vec2(0.0f, 8.0f)));
    pts.push_back(sk.addPoint(glm::vec2(3.0f, 9.0f)));
    pts.push_back(sk.addPoint(glm::vec2(7.0f, 8.0f)));
    pts.push_back(sk.addPoint(glm::vec2(10.5f, 3.0f)));
    const int spline = sk.addSpline(pts);
    addContact(sk, ConstraintType::G2, spline, circle);
    sk.enforceSplineContacts();

    const SketchSpline& sp = sk.getSplines().front();
    const std::vector<CurvatureCombSample> comb = sk.curvatureComb(sp, 8);
    ASSERT_EQ(comb.size(), 3u * 8u + 1u);
    EXPECT_NEAR(std::abs(comb.back().curvature), 0.1, 1e-3);
    // Tooth at the bound end points toward the circle's centre.
    EXPECT_GT(glm::dot(comb.back().normal * comb.back().curvature,
                       -glm::normalize(comb.back().point)), 0.0);
    const std::vector<glm::vec2> samp = sk.sampleSpline2D(sp, 8);
    EXPECT_NEAR(comb.back().point.x, samp.back().x, 1e-5);
    EXPECT_NEAR(comb.back().point.y, samp.back().y, 1e-5);

    // Without the contact the free spline's end curvature differs.
    Sketch free;
    std::vector<int> fp;
    for (int id : pts) fp.push_back(free.addPoint(sk.getPoint(id)->pos));
    free.addSpline(fp);
    const std::vector<CurvatureCombSample> freeComb =
        free.curvatureComb(free.getSplines().front(), 8);
    ASSERT_FALSE(freeComb.empty());
    EXPECT_GT(std::abs(std::abs(freeComb.back().curvature) - 0.1), 1e-2);
}
