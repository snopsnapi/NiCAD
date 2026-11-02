// Form workspace core (fork, src/modeling/Form): Catmull-Clark cage with
// creases, cage edits, and conversion to NURBS.

#include "modeling/Form.h"

#include <gtest/gtest.h>

#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

#include <cmath>

using namespace materializr;

namespace {

double volume(const TopoDS_Shape& s) {
    GProp_GProps g;
    BRepGProp::VolumeProperties(s, g);
    return g.Mass();
}
double area(const TopoDS_Shape& s) {
    GProp_GProps g;
    BRepGProp::SurfaceProperties(s, g);
    return g.Mass();
}
int count(const TopoDS_Shape& s, TopAbs_ShapeEnum t) {
    int n = 0;
    for (TopExp_Explorer ex(s, t); ex.More(); ex.Next()) ++n;
    return n;
}

} // namespace

// A 2 mm cube cage: the smooth limit stays inside the cube, rounds every
// corner, and converts to one valid closed solid (24 patches).
TEST(Form, CubeSmoothsAndConvertsToSolid) {
    const form::Cage cube = form::box(2, 2, 2, 1);
    ASSERT_TRUE(cube.valid());
    EXPECT_TRUE(cube.closed());
    const form::Cage lim = form::subdivideLevels(cube, 4, true);
    double maxR = 0;
    for (const gp_Pnt& p : lim.verts) {
        EXPECT_LE(std::abs(p.X()), 1.0 + 1e-9);
        EXPECT_LE(std::abs(p.Y()), 1.0 + 1e-9);
        maxR = std::max(maxR, gp_Vec(p.XYZ()).Magnitude());
    }
    EXPECT_LT(maxR, std::sqrt(3.0) - 0.3);   // corners rounded off
    const TopoDS_Shape s = form::toNurbs(cube, 3);
    ASSERT_FALSE(s.IsNull());
    EXPECT_EQ(count(s, TopAbs_SOLID), 1);
    EXPECT_EQ(count(s, TopAbs_FACE), 24);
    EXPECT_TRUE(BRepCheck_Analyzer(s).IsValid());
    const double v = volume(s);
    EXPECT_GT(v, 2.5);
    EXPECT_LT(v, 8.0);
}

// Every edge creased: the limit is the cube itself.
TEST(Form, FullyCreasedCubeIsTheCube) {
    form::Cage cube = form::box(2, 2, 2, 1);
    cube.setCrease(cube.edges(), true);
    const TopoDS_Shape s = form::toNurbs(cube, 2);
    ASSERT_FALSE(s.IsNull());
    EXPECT_NEAR(volume(s), 8.0, 1e-3);
}

// An open plane cage converts to a flat sheet over the same square.
TEST(Form, PlaneConvertsFlat) {
    const form::Cage pl = form::plane(10, 10, 2, 2);
    EXPECT_FALSE(pl.closed());
    const TopoDS_Shape s = form::toNurbs(pl, 2);
    ASSERT_FALSE(s.IsNull());
    EXPECT_EQ(count(s, TopAbs_SOLID), 0);
    EXPECT_NEAR(area(s), 100.0, 1e-3);
}

// Subdividing one face: the neighbours become 5-gons (T-junctions); the cage
// still converts to a closed solid.
TEST(Form, SubdivideFaceMakesTJunctions) {
    form::Cage cube = form::box(2, 2, 2, 1);
    cube.subdivideFace(0);
    EXPECT_EQ(cube.faces.size(), 9u);
    int pentagons = 0;
    for (const auto& f : cube.faces) pentagons += (f.size() == 5);
    EXPECT_EQ(pentagons, 4);
    EXPECT_TRUE(cube.closed());
    const TopoDS_Shape s = form::toNurbs(cube, 2);
    ASSERT_FALSE(s.IsNull());
    EXPECT_EQ(count(s, TopAbs_SOLID), 1);
}

// An edge loop around a cube splits its four ring faces.
TEST(Form, InsertEdgeLoop) {
    form::Cage cube = form::box(2, 2, 2, 1);
    cube.insertEdgeLoop(cube.edges().front());
    EXPECT_EQ(cube.faces.size(), 10u);
    for (const auto& f : cube.faces) EXPECT_EQ(f.size(), 4u);
    EXPECT_TRUE(cube.closed());
}

// Extruding the top face: 4 side quads, the face moved up, still closed.
TEST(Form, ExtrudeFace) {
    form::Cage cube = form::box(2, 2, 2, 1);
    int top = -1;
    for (size_t f = 0; f < cube.faces.size(); ++f)
        if (cube.faceNormal(static_cast<int>(f)).Y() > 0.9) top = static_cast<int>(f);
    ASSERT_GE(top, 0);
    const auto moved = cube.extrudeFaces({top}, 1.0);
    ASSERT_EQ(moved.size(), 1u);
    EXPECT_EQ(cube.faces.size(), 10u);
    EXPECT_EQ(cube.verts.size(), 12u);
    EXPECT_NEAR(cube.faceCentre(top).Y(), 2.0, 1e-9);
    EXPECT_TRUE(cube.closed());
}

TEST(Form, SerializeRoundTrip) {
    form::Cage c = form::cylinder(5, 10, 8, 2);
    c.setCrease({c.edges()[3]}, true);
    form::Cage r;
    ASSERT_TRUE(r.deserialize(c.serialize()));
    EXPECT_EQ(r.verts.size(), c.verts.size());
    EXPECT_EQ(r.faces, c.faces);
    EXPECT_EQ(r.creases, c.creases);
    const TopoDS_Shape s = form::toNurbs(r, 2);
    ASSERT_FALSE(s.IsNull());
    EXPECT_EQ(count(s, TopAbs_SOLID), 1);
}

TEST(Form, DisplayShapeIsTriangles) {
    const TopoDS_Shape d = form::displayShape(form::sphere(5, 1), 2);
    ASSERT_FALSE(d.IsNull());
    EXPECT_EQ(count(d, TopAbs_FACE), 6 * 4 * 4 * 2);
}

TEST(Form, DisplayTrianglesAndRayHit) {
    const auto tris = form::displayTriangles(form::box(2, 2, 2, 1), 2);
    EXPECT_EQ(tris.size(), static_cast<size_t>(6 * 4 * 4 * 2 * 3 * 6));
    double t = 0;
    ASSERT_TRUE(form::rayHit(tris, gp_Pnt(0, 0, 10), gp_Vec(0, 0, -1), t));
    EXPECT_GT(t, 9.0);
    EXPECT_LT(t, 10.0);
    EXPECT_FALSE(form::rayHit(tris, gp_Pnt(5, 5, 10), gp_Vec(0, 0, -1), t));
}

// The Form and Convert Form steps on a Document: the cage lands in the form
// table, conversion adds a solid body and hides the form, undo reverses both,
// and the params round-trip.
#include "core/Document.h"
#include "modeling/FormOp.h"
TEST(Form, FormAndConvertSteps) {
    Document doc;
    FormOp f;
    f.setCage(form::box(10, 10, 10, 1).serialize());
    ASSERT_TRUE(f.execute(doc));
    ASSERT_GE(f.formId(), 0);
    ASSERT_NE(doc.getForm(f.formId()), nullptr);
    EXPECT_TRUE(doc.isFormVisible(f.formId()));
    ConvertFormOp cv;
    cv.setForm(f.formId());
    ASSERT_TRUE(cv.execute(doc));
    ASSERT_GE(cv.createdBodyId(), 0);
    EXPECT_GT(volume(doc.getBody(cv.createdBodyId())), 300.0);
    EXPECT_FALSE(doc.isFormVisible(f.formId()));
    ASSERT_TRUE(cv.undo(doc));
    EXPECT_TRUE(doc.isFormVisible(f.formId()));
    FormOp re;
    ASSERT_TRUE(re.deserializeParams(f.serializeParams()));
    EXPECT_EQ(re.formId(), f.formId());
    EXPECT_EQ(re.cage(), f.cage());
}
