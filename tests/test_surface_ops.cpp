// Surface workspace geometry (fork, src/modeling/SurfaceOps): delete faces
// without healing, extrude curves into surfaces, sweep, trim.

#include "modeling/SurfaceOps.h"
#include "modeling/Sketch.h"
#include "core/Document.h"
#include "modeling/DefeatureOp.h"
#include "modeling/SurfaceExtrudeOp.h"
#include "modeling/SweepOp.h"
#include "modeling/TrimSurfaceOp.h"
#include "modeling/SurfaceEditOps.h"
#include "modeling/StudioSplineOp.h"
#include <algorithm>
#include <memory>

#include <gtest/gtest.h>

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <BRep_Tool.hxx>
#include <gp_Pln.hxx>
#include <gp_Circ.hxx>
#include <gp_Ax2.hxx>
#include <Geom_Curve.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <cmath>

#include <vector>

using namespace materializr;

namespace {

int count(const TopoDS_Shape& s, TopAbs_ShapeEnum t) {
    int n = 0;
    for (TopExp_Explorer ex(s, t); ex.More(); ex.Next()) ++n;
    return n;
}

double area(const TopoDS_Shape& s) {
    GProp_GProps g;
    BRepGProp::SurfaceProperties(s, g);
    return g.Mass();
}

TopoDS_Wire lineWire(const gp_Pnt& a, const gp_Pnt& b) {
    return BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(a, b).Edge()).Wire();
}

} // namespace

// Box 10x20x30, top face removed without healing: five faces left as one
// open shell, no solid.
TEST(SurfaceOps, RemoveFaceWithoutHealLeavesOpenShell) {
    const TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape();
    TopoDS_Face top;
    for (TopExp_Explorer ex(box, TopAbs_FACE); ex.More(); ex.Next()) {
        const gp_Pnt p = surf::facePoint(TopoDS::Face(ex.Current()));
        if (std::abs(p.Z() - 30.0) < 1e-6) top = TopoDS::Face(ex.Current());
    }
    ASSERT_FALSE(top.IsNull());
    const TopoDS_Shape out = surf::removeFacesOpen(box, {top});
    ASSERT_FALSE(out.IsNull());
    EXPECT_EQ(count(out, TopAbs_FACE), 5);
    EXPECT_EQ(count(out, TopAbs_SOLID), 0);
    EXPECT_EQ(count(out, TopAbs_SHELL), 1);
    EXPECT_NEAR(area(out), 2.0 * (10 * 20 + 10 * 30 + 20 * 30) - 10 * 20, 1e-6);
}

TEST(SurfaceOps, RemoveFaceNotInBodyIsNull) {
    const TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();
    const TopoDS_Shape other = BRepPrimAPI_MakeBox(5.0, 5.0, 5.0).Shape();
    TopExp_Explorer ex(other, TopAbs_FACE);
    EXPECT_TRUE(surf::removeFacesOpen(box, {TopoDS::Face(ex.Current())}).IsNull());
}

// An open L-shaped polyline extruded 7 mm: two faces, area = length * 7.
TEST(SurfaceOps, ExtrudeOpenCurveIsSurface) {
    BRepBuilderAPI_MakePolygon poly;
    poly.Add(gp_Pnt(0, 0, 0));
    poly.Add(gp_Pnt(10, 0, 0));
    poly.Add(gp_Pnt(10, 5, 0));
    const TopoDS_Shape out = surf::extrudeCurves({poly.Wire()}, gp_Vec(0, 0, 7));
    ASSERT_FALSE(out.IsNull());
    EXPECT_EQ(count(out, TopAbs_FACE), 2);
    EXPECT_EQ(count(out, TopAbs_SOLID), 0);
    EXPECT_NEAR(area(out), 15.0 * 7.0, 1e-6);
}

TEST(SurfaceOps, ExtrudeZeroVectorIsNull) {
    EXPECT_TRUE(surf::extrudeCurves({lineWire(gp_Pnt(0, 0, 0), gp_Pnt(1, 0, 0))},
                                    gp_Vec(0, 0, 0)).IsNull());
}

// A 4 mm line profile swept along a 12 mm straight path: one 4x12 face.
TEST(SurfaceOps, SweepCurveIsSurface) {
    const TopoDS_Wire profile = lineWire(gp_Pnt(0, 0, 0), gp_Pnt(4, 0, 0));
    const TopoDS_Wire path = lineWire(gp_Pnt(0, 0, 0), gp_Pnt(0, 0, 12));
    const TopoDS_Shape out = surf::sweep(profile, path);
    ASSERT_FALSE(out.IsNull());
    EXPECT_EQ(count(out, TopAbs_SOLID), 0);
    EXPECT_NEAR(area(out), 48.0, 1e-6);
}

// A closed square face swept along a path gives a solid.
TEST(SurfaceOps, SweepFaceIsSolid) {
    BRepBuilderAPI_MakePolygon sq;
    sq.Add(gp_Pnt(0, 0, 0)); sq.Add(gp_Pnt(2, 0, 0));
    sq.Add(gp_Pnt(2, 2, 0)); sq.Add(gp_Pnt(0, 2, 0));
    sq.Close();
    const TopoDS_Face face = BRepBuilderAPI_MakeFace(sq.Wire()).Face();
    const TopoDS_Shape out =
        surf::sweep(face, lineWire(gp_Pnt(1, 1, 0), gp_Pnt(1, 1, 5)));
    ASSERT_FALSE(out.IsNull());
    EXPECT_EQ(count(out, TopAbs_SOLID), 1);
}

// A 10x10 square sheet trimmed by a cutter along x = 5 (a line in a sketch
// plane, pushed through the sheet): removing the piece around (2, 5) leaves
// the other half, area 50.
TEST(SurfaceOps, TrimSheetByCurve) {
    const TopoDS_Face sheet =
        BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)),
                                0.0, 10.0, 0.0, 10.0).Face();
    // The cutting line lives in the plane y-z (normal +x would make it a
    // plane parallel to the sheet), drawn along y at x = 5, z = 0, and is
    // pushed through along z.
    const TopoDS_Shape cutter = surf::curveCutter(
        {lineWire(gp_Pnt(5, -1, 0), gp_Pnt(5, 11, 0))}, gp_Vec(0, 0, 1), 5.0);
    ASSERT_FALSE(cutter.IsNull());
    const TopoDS_Shape split = surf::splitByTool(sheet, cutter);
    ASSERT_FALSE(split.IsNull());
    EXPECT_EQ(count(split, TopAbs_FACE), 2);

    const TopoDS_Shape out = surf::trim(sheet, cutter, {gp_Pnt(2, 5, 0)});
    ASSERT_FALSE(out.IsNull());
    EXPECT_EQ(count(out, TopAbs_FACE), 1);
    EXPECT_NEAR(area(out), 50.0, 1e-6);
    EXPECT_GT(surf::facePoint(TopoDS::Face(TopExp_Explorer(out, TopAbs_FACE).Current())).X(), 5.0);
}

// Sketch curves as exact wires: an open L of two lines and a separate full
// circle come back as two wires - the L open with two edges, the circle
// closed - and the L extrudes into a 2-face surface.
TEST(SurfaceOps, SketchCurveWires) {
    Sketch sk;   // XY plane at the origin by default
    const int a = sk.addPoint(glm::vec2(0.0f, 0.0f));
    const int b = sk.addPoint(glm::vec2(10.0f, 0.0f));
    const int c = sk.addPoint(glm::vec2(10.0f, 5.0f));
    sk.addLine(a, b);
    sk.addLine(b, c);
    const int ctr = sk.addPoint(glm::vec2(30.0f, 0.0f));
    sk.addCircle(ctr, 4.0);
    const std::vector<TopoDS_Wire> wires = sk.buildCurveWires();
    ASSERT_EQ(wires.size(), 2u);
    int open = 0, closed = 0;
    for (const TopoDS_Wire& w : wires) {
        if (BRep_Tool::IsClosed(w)) {
            ++closed;
            EXPECT_EQ(count(w, TopAbs_EDGE), 1);
        } else {
            ++open;
            EXPECT_EQ(count(w, TopAbs_EDGE), 2);
        }
    }
    EXPECT_EQ(open, 1);
    EXPECT_EQ(closed, 1);
    for (const TopoDS_Wire& w : wires) {
        if (BRep_Tool::IsClosed(w)) continue;
        const TopoDS_Shape s = surf::extrudeCurves({w}, gp_Vec(0, 0, 3));
        EXPECT_NEAR(area(s), 15.0 * 3.0, 1e-6);
    }
}

// The operations on a real Document: Delete Face without Heal opens the box
// (undo closes it again), and the Heal flag survives a save round trip.
TEST(SurfaceOpsDoc, DeleteFaceWithoutHeal) {
    Document doc;
    const int id = doc.addBody(BRepPrimAPI_MakeBox(10, 10, 10).Shape(), "box");
    DefeatureOp op;
    op.setBody(id);
    op.setHeal(false);
    TopExp_Explorer ex(doc.getBody(id), TopAbs_FACE);
    op.addFace(TopoDS::Face(ex.Current()));
    ASSERT_TRUE(op.execute(doc));
    EXPECT_NEAR(area(doc.getBody(id)), 500.0, 1e-6);
    EXPECT_EQ(count(doc.getBody(id), TopAbs_SOLID), 0);
    ASSERT_TRUE(op.undo(doc));
    EXPECT_NEAR(area(doc.getBody(id)), 600.0, 1e-6);
    DefeatureOp re;
    ASSERT_TRUE(re.deserializeParams(op.serializeParams()));
    EXPECT_FALSE(re.heal());
}

// Surface Extrude of an open sketch line (10 mm, 6 mm up) and a Trim of that
// sheet by a second sketch's line at x = 4: the tapped piece (x < 4) goes,
// 6 x 6 remains; undo restores the sheet.
TEST(SurfaceOpsDoc, SurfaceExtrudeThenTrim) {
    Document doc;
    auto sk = std::make_shared<Sketch>();
    sk->addLine(sk->addPoint(glm::vec2(0, 0)), sk->addPoint(glm::vec2(10, 0)));
    const int sid = doc.addSketch(sk);
    SurfaceExtrudeOp ex;
    ex.setSketchSource(sid);
    ex.setDistance(6.0);
    ASSERT_TRUE(ex.execute(doc));
    const int body = ex.createdBodyId();
    ASSERT_GE(body, 0);
    EXPECT_NEAR(area(doc.getBody(body)), 60.0, 1e-6);
    SurfaceExtrudeOp exRe;
    ASSERT_TRUE(exRe.deserializeParams(ex.serializeParams()));
    EXPECT_EQ(exRe.getSketchId(), sid);
    EXPECT_NEAR(exRe.getDistance(), 6.0, 1e-9);

    auto cutSk = std::make_shared<Sketch>();
    cutSk->addLine(cutSk->addPoint(glm::vec2(4, -5)), cutSk->addPoint(glm::vec2(4, 5)));
    const int cutId = doc.addSketch(cutSk);
    TrimSurfaceOp trim;
    trim.setTarget(body);
    trim.setToolSketch(cutId);
    trim.setRemovePoints({gp_Pnt(1, 0, 3)});
    ASSERT_TRUE(trim.execute(doc));
    EXPECT_NEAR(area(doc.getBody(body)), 36.0, 1e-6);
    ASSERT_TRUE(trim.undo(doc));
    EXPECT_NEAR(area(doc.getBody(body)), 60.0, 1e-6);
    TrimSurfaceOp trimRe;
    ASSERT_TRUE(trimRe.deserializeParams(trim.serializeParams()));
    EXPECT_EQ(trimRe.targetId(), body);
}

// SweepOp accepts a curve profile and makes a surface body.
TEST(SurfaceOpsDoc, SweepCurveProfile) {
    Document doc;
    SweepOp op;
    op.setProfile(lineWire(gp_Pnt(0, 0, 0), gp_Pnt(4, 0, 0)));
    op.setPath(lineWire(gp_Pnt(0, 0, 0), gp_Pnt(0, 0, 12)));
    ASSERT_TRUE(op.execute(doc));
    int n = 0;
    for (int id : doc.getAllBodyIds()) {
        ++n;
        EXPECT_NEAR(area(doc.getBody(id)), 48.0, 1e-6);
    }
    EXPECT_EQ(n, 1);
}

namespace {

TopoDS_Face xyPatch(double x0, double x1, double y0, double y1) {
    return BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)),
                                   x0, x1, y0, y1).Face();
}

// The edge of `face` whose midpoint is nearest `p`.
TopoDS_Edge edgeNear(const TopoDS_Shape& s, const gp_Pnt& p) {
    TopoDS_Edge best;
    double bestD = 1e300;
    for (TopExp_Explorer ex(s, TopAbs_EDGE); ex.More(); ex.Next()) {
        const TopoDS_Edge e = TopoDS::Edge(ex.Current());
        double f, l;
        Handle(Geom_Curve) c = BRep_Tool::Curve(e, f, l);
        if (c.IsNull()) continue;
        const double d = c->Value(0.5 * (f + l)).Distance(p);
        if (d < bestD) { bestD = d; best = e; }
    }
    return best;
}

} // namespace

// A 10x10 sheet extended 5 mm past its x = 10 edge: 15 x 10.
TEST(SurfaceOps, ExtendPlanarEdge) {
    const TopoDS_Face sheet = xyPatch(0, 10, 0, 10);
    const TopoDS_Shape out =
        surf::extendEdges(sheet, {edgeNear(sheet, gp_Pnt(10, 5, 0))}, 5.0);
    ASSERT_FALSE(out.IsNull());
    EXPECT_EQ(count(out, TopAbs_FACE), 1);
    EXPECT_NEAR(area(out), 150.0, 1e-4);
}

// Two edges of the same face extended at once: 15 x 15.
TEST(SurfaceOps, ExtendTwoSides) {
    const TopoDS_Face sheet = xyPatch(0, 10, 0, 10);
    const TopoDS_Shape out = surf::extendEdges(
        sheet, {edgeNear(sheet, gp_Pnt(10, 5, 0)), edgeNear(sheet, gp_Pnt(5, 0, 0))}, 5.0);
    ASSERT_FALSE(out.IsNull());
    EXPECT_NEAR(area(out), 225.0, 1e-4);
}

// An L of two sheets sharing an edge: extending a free edge of one keeps the
// two joined in one shell. The shared edge is not free - it is refused.
TEST(SurfaceOps, ExtendKeepsShellJoined) {
    BRepBuilderAPI_MakePolygon poly;
    poly.Add(gp_Pnt(0, 0, 10));
    poly.Add(gp_Pnt(0, 0, 0));
    poly.Add(gp_Pnt(10, 0, 0));
    const TopoDS_Shape l = surf::extrudeCurves({poly.Wire()}, gp_Vec(0, 10, 0));
    ASSERT_EQ(count(l, TopAbs_FACE), 2);
    const TopoDS_Shape out =
        surf::extendEdges(l, {edgeNear(l, gp_Pnt(10, 5, 0))}, 4.0);
    ASSERT_FALSE(out.IsNull());
    EXPECT_EQ(count(out, TopAbs_SHELL), 1);
    EXPECT_NEAR(area(out), 100.0 + 140.0, 1e-4);
    EXPECT_TRUE(surf::extendEdges(l, {edgeNear(l, gp_Pnt(0, 5, 0))}, 4.0).IsNull());
}

// A sheet offset 3 mm along its normal: same area, moved to z = 3.
TEST(SurfaceOps, OffsetPlanarSheet) {
    const TopoDS_Shape out = surf::offsetFaces({xyPatch(0, 10, 0, 10)}, 3.0);
    ASSERT_FALSE(out.IsNull());
    EXPECT_NEAR(area(out), 100.0, 1e-4);
    Bnd_Box bb;
    BRepBndLib::Add(out, bb);
    double x0, y0, z0, x1, y1, z1;
    bb.Get(x0, y0, z0, x1, y1, z1);
    EXPECT_NEAR(0.5 * (z0 + z1), 3.0, 1e-3);
}

// A quarter cylinder R = 10, height 5, offset outward by 2: area scales R'/R.
TEST(SurfaceOps, OffsetCylindricalSheet) {
    BRepBuilderAPI_MakeEdge arc(gp_Circ(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 10.0),
                                0.0, M_PI / 2.0);
    const TopoDS_Shape sheet = surf::extrudeCurves(
        {BRepBuilderAPI_MakeWire(arc.Edge()).Wire()}, gp_Vec(0, 0, 5));
    ASSERT_EQ(count(sheet, TopAbs_FACE), 1);
    const TopoDS_Face face = TopoDS::Face(TopExp_Explorer(sheet, TopAbs_FACE).Current());
    const double a0 = area(face);
    const TopoDS_Shape out = surf::offsetFaces({face}, 2.0);
    ASSERT_FALSE(out.IsNull());
    const double a1 = area(out);
    // Outward or inward depends on the face's normal: accept either radius.
    const bool outward = std::abs(a1 - a0 * 12.0 / 10.0) < 1e-3 * a0;
    const bool inward = std::abs(a1 - a0 * 8.0 / 10.0) < 1e-3 * a0;
    EXPECT_TRUE(outward || inward) << "a0=" << a0 << " a1=" << a1;
}

// Extend and Offset as history steps on a Document: Extend grows the sheet
// in place (undo restores it) and round-trips its params; Offset adds a new
// body and leaves the source alone.
TEST(SurfaceOpsDoc, ExtendAndOffsetSteps) {
    Document doc;
    const int id = doc.addBody(xyPatch(0, 10, 0, 10), "sheet");
    ExtendSurfaceOp ext;
    ext.setBody(id);
    ext.setEdges({edgeNear(doc.getBody(id), gp_Pnt(10, 5, 0))});
    ext.setLength(5.0);
    ASSERT_TRUE(ext.execute(doc));
    EXPECT_NEAR(area(doc.getBody(id)), 150.0, 1e-4);
    ExtendSurfaceOp extRe;
    ASSERT_TRUE(extRe.deserializeParams(ext.serializeParams()));
    EXPECT_NEAR(extRe.length(), 5.0, 1e-9);
    ASSERT_TRUE(ext.undo(doc));
    EXPECT_NEAR(area(doc.getBody(id)), 100.0, 1e-4);

    OffsetSurfaceOp off;
    off.setSource(id);
    off.setFaces({TopoDS::Face(TopExp_Explorer(doc.getBody(id), TopAbs_FACE).Current())});
    off.setDistance(3.0);
    ASSERT_TRUE(off.execute(doc));
    ASSERT_GE(off.createdBodyId(), 0);
    EXPECT_NE(off.createdBodyId(), id);
    EXPECT_NEAR(area(doc.getBody(off.createdBodyId())), 100.0, 1e-4);
    EXPECT_NEAR(area(doc.getBody(id)), 100.0, 1e-4);
    OffsetSurfaceOp offRe;
    ASSERT_TRUE(offRe.deserializeParams(off.serializeParams()));
    EXPECT_NEAR(offRe.distance(), 3.0, 1e-9);
}

// Studio Spline: the curve passes through every point (also closed), honours
// an end tangent, survives a params round trip, and is a curve-only body.
TEST(StudioSpline, ThroughPointsAndTangent) {
    const std::vector<gp_Pnt> pts = {gp_Pnt(0, 0, 0), gp_Pnt(10, 5, 3),
                                     gp_Pnt(20, 0, 8), gp_Pnt(30, -4, 2)};
    const TopoDS_Shape w = StudioSplineOp::buildCurve(pts, false, gp_Vec(0, 0, 1));
    ASSERT_FALSE(w.IsNull());
    EXPECT_EQ(count(w, TopAbs_FACE), 0);
    const TopoDS_Edge e = TopoDS::Edge(TopExp_Explorer(w, TopAbs_EDGE).Current());
    double f, l;
    Handle(Geom_Curve) c = BRep_Tool::Curve(e, f, l);
    ASSERT_FALSE(c.IsNull());
    for (const gp_Pnt& p : pts) {
        double best = 1e9;
        for (int i = 0; i <= 2000; ++i)
            best = std::min(best, c->Value(f + (l - f) * i / 2000.0).Distance(p));
        EXPECT_LT(best, 0.05);
    }
    gp_Pnt p0;
    gp_Vec d0;
    c->D1(f, p0, d0);
    EXPECT_GT(d0.Normalized().Z(), 0.999);

    const TopoDS_Shape loop = StudioSplineOp::buildCurve(pts, true);
    ASSERT_FALSE(loop.IsNull());
    EXPECT_TRUE(BRep_Tool::IsClosed(TopoDS::Wire(loop)));

    Document doc;
    StudioSplineOp op;
    op.setPoints(pts);
    ASSERT_TRUE(op.execute(doc));
    ASSERT_GE(op.createdBodyId(), 0);
    op.setPoint(1, gp_Pnt(10, 9, 3));
    ASSERT_TRUE(op.execute(doc));   // re-execute reuses the body id
    EXPECT_EQ(static_cast<int>(doc.getAllBodyIds().size()), 1);
    StudioSplineOp re;
    ASSERT_TRUE(re.deserializeParams(op.serializeParams()));
    ASSERT_EQ(re.points().size(), 4u);
    EXPECT_NEAR(re.points()[1].Y(), 9.0, 1e-9);
}

// Point conditions: G1 at an interior point fixes the tangent there; G2 at
// the end also sets the curvature vector; both survive a params round trip.
TEST(StudioSpline, PointConditions) {
    const std::vector<gp_Pnt> pts = {gp_Pnt(0, 0, 0), gp_Pnt(10, 5, 0),
                                     gp_Pnt(20, 0, 0), gp_Pnt(30, 5, 0)};
    std::vector<StudioSplineOp::PointCond> conds(4);
    conds[1].type = StudioSplineOp::Cond::G1;
    conds[1].dir = gp_Vec(0, 0, 1) + gp_Vec(1, 0, 0);          // up-and-along
    conds[3].type = StudioSplineOp::Cond::G2;
    conds[3].dir = gp_Vec(1, 0, 0);
    conds[3].curv = gp_Vec(0, 0.1, 0);                          // R = 10, centre +Y
    const TopoDS_Shape w = StudioSplineOp::buildCurve(pts, false, gp_Vec(), gp_Vec(), conds);
    ASSERT_FALSE(w.IsNull());
    double f, l;
    Handle(Geom_Curve) c = BRep_Tool::Curve(
        TopoDS::Edge(TopExp_Explorer(w, TopAbs_EDGE).Current()), f, l);
    ASSERT_FALSE(c.IsNull());
    // Parameter of the point nearest pts[1].
    double tBest = f, dBest = 1e9;
    for (int i = 0; i <= 4000; ++i) {
        const double t = f + (l - f) * i / 4000.0;
        const double d = c->Value(t).Distance(pts[1]);
        if (d < dBest) { dBest = d; tBest = t; }
    }
    gp_Pnt p;
    gp_Vec d1, d2;
    c->D1(tBest, p, d1);
    const gp_Vec want = gp_Vec(1, 0, 1).Normalized();
    EXPECT_GT(d1.Normalized().Dot(want), 0.995);
    // End curvature: |d1 x d2| / |d1|^3, centre side +Y.
    c->D2(l, p, d1, d2);
    const double kappa = d1.Crossed(d2).Magnitude() / std::pow(d1.Magnitude(), 3);
    EXPECT_NEAR(kappa, 0.1, 1e-3);
    const gp_Vec normalComp = d2 - d1.Normalized() * d2.Dot(d1.Normalized());
    EXPECT_GT(normalComp.Y(), 0.0);

    StudioSplineOp op;
    op.setPoints(pts);
    op.setCondition(3, conds[3]);
    StudioSplineOp re;
    ASSERT_TRUE(re.deserializeParams(op.serializeParams()));
    EXPECT_EQ(re.condition(3).type, StudioSplineOp::Cond::G2);
    EXPECT_NEAR(re.condition(3).curv.Y(), 0.1, 1e-9);
    EXPECT_EQ(re.condition(1).type, StudioSplineOp::Cond::Free);
}
