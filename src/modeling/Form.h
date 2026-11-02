#pragma once
// Form modelling (fork, Form workspace): a T-spline-style control cage with a
// Catmull-Clark smooth surface, creases, and conversion to NURBS.
//
// The cage is a polygon mesh (any n >= 3 per face). Catmull-Clark accepts
// n-gons, so splitting one face into four (subdivideFace) leaves its
// neighbours with an extra vertex - the T-junction a T-spline would have -
// without any special case. Creased and boundary edges stay sharp (crease
// rules: edge point = midpoint, crease vertex = (6P + a + b) / 8, three or more
// sharp edges = corner).
//
// NURBS conversion: every quad of the FIRST subdivision level becomes one
// bicubic B-spline patch interpolating a grid of limit-surface points (taken
// from deeper levels). Neighbouring patches share their boundary data, and a
// clamped tensor-product interpolation's boundary curve depends on its
// boundary row alone, so patch edges meet exactly; the patches are sewn into
// a shell, and a closed cage becomes a solid.
//
// Pure geometry: no Document, no UI. tests/test_form.cpp checks it.

#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace materializr {
namespace form {

using Edge = std::pair<int, int>;   // undirected, first < second

inline Edge edgeKey(int a, int b) { return a < b ? Edge(a, b) : Edge(b, a); }

struct Cage {
    std::vector<gp_Pnt> verts;
    std::vector<std::vector<int>> faces;   // CCW seen from outside
    std::set<Edge> creases;
    // Symmetry (Fusion's Edit Form symmetry): mirror plane normal to world
    // axis symAxis (0 X, 1 Y, 2 Z; -1 off) at coordinate symPos. Saved with
    // the cage.
    int symAxis = -1;
    double symPos = 0.0;

    gp_Pnt mirror(const gp_Pnt& p) const;
    // Each vertex's mirror partner found by position (itself for a vertex on
    // the plane, -1 for none). Empty when symmetry is off.
    std::vector<int> mirrorPartners() const;
    // Move `ids` by `t` from their positions in `start`, keeping symmetry:
    // each moved vertex's partner (unless also moved) goes to its mirror
    // image, and a vertex on the plane stays on it.
    void transformSymmetric(const Cage& start, const std::vector<int>& ids, const gp_Trsf& t);
    // The mirror image of a face / an edge, or -1 / {-1,-1}.
    int mirrorFace(int f, const std::vector<int>& partner) const;
    Edge mirrorEdge(const Edge& e, const std::vector<int>& partner) const;

    bool valid() const;
    // Every undirected edge of the faces, once.
    std::vector<Edge> edges() const;
    // Faces using the edge (1 = boundary, 2 = interior).
    std::vector<int> edgeFaces(const Edge& e) const;
    bool closed() const;   // no boundary edges

    // ── Edits (all keep the cage valid; indices of untouched vertices and
    //    faces are preserved where the edit allows) ─────────────────────────
    void transformVertices(const std::vector<int>& ids, const gp_Trsf& t);
    void setCrease(const std::vector<Edge>& es, bool on);
    // Face -> n quads around a new centre vertex; edge midpoints are inserted
    // into the neighbouring faces too (they gain a vertex: T-junctions).
    void subdivideFace(int face);
    // Insert an edge loop across the quad ring through `e`: every quad of
    // the ring is split through the midpoints of its two ring edges. The ring
    // stops at a non-quad or a boundary.
    void insertEdgeLoop(const Edge& e);
    // Pull the faces out along their average normal by `dist`, with side
    // quads along the region's border. Returns the new faces' indices (the
    // moved copies of `faces`).
    std::vector<int> extrudeFaces(const std::vector<int>& faces, double dist);
    void deleteFaces(const std::vector<int>& faces);

    // Average of the adjacent faces' normals (unit; zero if undefined).
    gp_Vec vertexNormal(int v) const;
    gp_Vec faceNormal(int f) const;
    gp_Pnt faceCentre(int f) const;

    std::string serialize() const;
    bool deserialize(const std::string& s);
};

// ── Primitives (centred on the origin, world Y up) ──────────────────────────
Cage box(double sx, double sy, double sz, int segs);
Cage plane(double sx, double sz, int segX, int segZ);
Cage cylinder(double radius, double height, int around, int segsH);
Cage sphere(double radius, int segs);   // a subdivided cube projected out

// ── Subdivision ─────────────────────────────────────────────────────────────
struct Subdivided {
    Cage mesh;   // all quads
    // For each face of the INPUT, its child quads in corner order (child j
    // starts at the input face's vertex j).
    std::vector<std::vector<int>> children;
};
Subdivided subdivide(const Cage& c);
// `levels` subdivisions; with `limit`, vertices pushed to the limit surface.
Cage subdivideLevels(const Cage& c, int levels, bool limit);
// Limit-surface positions of the mesh's vertices.
std::vector<gp_Pnt> limitPositions(const Cage& c);

// Display shape: the subdivided surface as planar triangle faces (a mesh
// body). Null on an empty cage.
TopoDS_Shape displayShape(const Cage& c, int levels);
// Fast display (no OCCT shapes): the subdivided limit surface as triangles,
// 6 floats per vertex (position, smooth normal), 3 vertices per triangle.
std::vector<float> displayTriangles(const Cage& c, int levels);
// Nearest hit of the ray on displayTriangles' output; false on a miss.
bool rayHit(const std::vector<float>& tris, const gp_Pnt& origin, const gp_Vec& dir,
            double& t);

// NURBS: one bicubic patch per level-1 quad, sewn (closed cage -> solid).
// `gridLevels` = how many extra levels feed each patch's point grid
// ((2^gridLevels + 1)^2 points). Null on failure.
TopoDS_Shape toNurbs(const Cage& c, int gridLevels = 3);

} // namespace form
} // namespace materializr
