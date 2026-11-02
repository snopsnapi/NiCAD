// Surface workspace geometry (fork, src/modeling/SurfaceOps): delete faces
// without healing, extrude curves into surfaces, sweep, trim.

#include "modeling/SurfaceOps.h"
#include "modeling/Sketch.h"

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
