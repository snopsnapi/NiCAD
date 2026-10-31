// Offset Face core: selected faces move along their outward normals while
// the neighbours extend to meet them (Fusion-style Offset Face, not a prism
// on top like Push/Pull). On an outward move the blends touching the moved
// face (fillets, chamfers) are detected and removed first, so the result
// lands on sharp restored geometry.

#include "modeling/OffsetFace.h"

#include <gtest/gtest.h>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <algorithm>
#include <cmath>
#include <vector>

using namespace materializr;

namespace {

double volume(const TopoDS_Shape& s) {
    GProp_GProps g;
    BRepGProp::VolumeProperties(s, g);
    return g.Mass();
}

void expectVolume(const TopoDS_Shape& s, double expected) {
    EXPECT_NEAR(volume(s), expected, std::abs(expected) * 1e-3);
}

int faceCount(const TopoDS_Shape& s) {
    int n = 0;
    for (TopExp_Explorer ex(s, TopAbs_FACE); ex.More(); ex.Next()) ++n;
    return n;
}

double maxZ(const TopoDS_Shape& s) {
    double z = -1e300;
    for (TopExp_Explorer ex(s, TopAbs_VERTEX); ex.More(); ex.Next())
        z = std::max(z, BRep_Tool::Pnt(TopoDS::Vertex(ex.Current())).Z());
    return z;
}

// Highest planar face whose outward normal is +Z.
TopoDS_Face topFace(const TopoDS_Shape& body) {
    TopoDS_Face best;
    double bestZ = -1e300;
    for (TopExp_Explorer ex(body, TopAbs_FACE); ex.More(); ex.Next()) {
        TopoDS_Face f = TopoDS::Face(ex.Current());
        BRepAdaptor_Surface s(f);
        if (s.GetType() != GeomAbs_Plane) continue;
        gp_Dir n = s.Plane().Axis().Direction();
        if (f.Orientation() == TopAbs_REVERSED) n.Reverse();
        if (n.Z() < 0.999) continue;
        Standard_Real u0, u1, v0, v1;
        BRepTools::UVBounds(f, u0, u1, v0, v1);
        const double z = s.Value(0.5 * (u0 + u1), 0.5 * (v0 + v1)).Z();
        if (z > bestZ) { bestZ = z; best = f; }
    }
    EXPECT_FALSE(best.IsNull());
    return best;
}

TopoDS_Face cylinderSideFace(const TopoDS_Shape& body) {
    for (TopExp_Explorer ex(body, TopAbs_FACE); ex.More(); ex.Next()) {
        TopoDS_Face f = TopoDS::Face(ex.Current());
        if (BRepAdaptor_Surface(f).GetType() == GeomAbs_Cylinder) return f;
    }
    EXPECT_TRUE(false) << "no cylindrical face found";
    return TopoDS_Face();
}

// All straight edges lying in the horizontal plane z = zPlane.
std::vector<TopoDS_Edge> edgesAtZ(const TopoDS_Shape& body, double zPlane) {
    std::vector<TopoDS_Edge> out;
    for (TopExp_Explorer ex(body, TopAbs_EDGE); ex.More(); ex.Next()) {
        TopoDS_Edge e = TopoDS::Edge(ex.Current());
        BRepAdaptor_Curve c(e);
        if (c.GetType() != GeomAbs_Line) continue;
        if (std::abs(c.Value(c.FirstParameter()).Z() - zPlane) < 1e-6 &&
            std::abs(c.Value(c.LastParameter()).Z() - zPlane) < 1e-6)
            out.push_back(e);
    }
    return out;
}

TopoDS_Shape chamferedBox() {
    TopoDS_Shape box = BRepPrimAPI_MakeBox(20.0, 20.0, 20.0).Shape();
    BRepFilletAPI_MakeChamfer ch(box);
    for (const TopoDS_Edge& e : edgesAtZ(box, 20.0)) ch.Add(2.0, e);
    ch.Build();
    EXPECT_TRUE(ch.IsDone());
    return ch.Shape();
}

TopoDS_Shape filletedBox() {
    TopoDS_Shape box = BRepPrimAPI_MakeBox(20.0, 20.0, 20.0).Shape();
    BRepFilletAPI_MakeFillet mk(box);
    for (const TopoDS_Edge& e : edgesAtZ(box, 20.0)) mk.Add(2.0, e);
    mk.Build();
    EXPECT_TRUE(mk.IsDone());
    return mk.Shape();
}

} // namespace

TEST(OffsetFace, Box_TopUp) {
    TopoDS_Shape box = BRepPrimAPI_MakeBox(20.0, 20.0, 20.0).Shape();
    offsetface::Result r;
    ASSERT_TRUE(offsetface::offsetFaces(box, {topFace(box)}, 5.0, {}, r))
        << r.error;
    expectVolume(r.shape, 10000.0);
    EXPECT_EQ(faceCount(r.shape), 6);
    EXPECT_TRUE(BRepCheck_Analyzer(r.shape).IsValid());
    EXPECT_NEAR(maxZ(r.shape), 25.0, 1e-6);
}

TEST(OffsetFace, Box_TopDown) {
    TopoDS_Shape box = BRepPrimAPI_MakeBox(20.0, 20.0, 20.0).Shape();
    offsetface::Result r;
    ASSERT_TRUE(offsetface::offsetFaces(box, {topFace(box)}, -5.0, {}, r))
        << r.error;
    expectVolume(r.shape, 6000.0);
    EXPECT_TRUE(BRepCheck_Analyzer(r.shape).IsValid());
}

TEST(OffsetFace, ChamferedBox_TopUp_RemovesChamfers) {
    TopoDS_Shape body = chamferedBox();
    const TopoDS_Face top = topFace(body);
    const auto blends = offsetface::findAdjacentBlends(body, top, 10.0);
    EXPECT_EQ(blends.size(), 4);

    offsetface::Result r;
    ASSERT_TRUE(offsetface::offsetFaces(body, {top}, 3.0, {}, r)) << r.error;
    EXPECT_EQ(r.removedBlends.size(), 4);
    expectVolume(r.shape, 9200.0);
    EXPECT_EQ(faceCount(r.shape), 6);
    EXPECT_TRUE(BRepCheck_Analyzer(r.shape).IsValid());
}

TEST(OffsetFace, FilletedBox_TopUp_RemovesFillets) {
    TopoDS_Shape body = filletedBox();
    const TopoDS_Face top = topFace(body);
    const auto blends = offsetface::findAdjacentBlends(body, top, 10.0);
    EXPECT_EQ(blends.size(), 4);

    offsetface::Result r;
    ASSERT_TRUE(offsetface::offsetFaces(body, {top}, 3.0, {}, r)) << r.error;
    EXPECT_EQ(r.removedBlends.size(), 4);
    expectVolume(r.shape, 9200.0);
    EXPECT_EQ(faceCount(r.shape), 6);
    EXPECT_TRUE(BRepCheck_Analyzer(r.shape).IsValid());
}

TEST(OffsetFace, ChamferKeptWhenDisabled) {
    TopoDS_Shape body = chamferedBox();
    const TopoDS_Face top = topFace(body);
    const double v0 = volume(body);

    offsetface::Options opt;
    opt.removeAdjacentBlends = false;
    offsetface::Result r;
    const bool ok = offsetface::offsetFaces(body, {top}, 3.0, opt, r);
    if (ok) {
        EXPECT_TRUE(r.removedBlends.empty());
        EXPECT_GT(volume(r.shape), v0);
    } else {
        EXPECT_FALSE(r.error.empty());
    }
}

TEST(OffsetFace, Cylinder_SideOut) {
    TopoDS_Shape cyl = BRepPrimAPI_MakeCylinder(10.0, 20.0).Shape();
    offsetface::Result r;
    ASSERT_TRUE(offsetface::offsetFaces(cyl, {cylinderSideFace(cyl)}, 2.0, {}, r))
        << r.error;
    expectVolume(r.shape, 3.14159265358979323846 * 12.0 * 12.0 * 20.0);
    EXPECT_TRUE(BRepCheck_Analyzer(r.shape).IsValid());
}

TEST(OffsetFace, PlainBox_NoBlends) {
    TopoDS_Shape box = BRepPrimAPI_MakeBox(20.0, 20.0, 20.0).Shape();
    EXPECT_TRUE(offsetface::findAdjacentBlends(box, topFace(box), 10.0).empty());
}

TEST(OffsetFace, EmptyInput_Fails) {
    TopoDS_Shape box = BRepPrimAPI_MakeBox(20.0, 20.0, 20.0).Shape();
    offsetface::Result r;
    EXPECT_FALSE(offsetface::offsetFaces(box, {}, 5.0, {}, r));
    EXPECT_FALSE(r.error.empty());
}
