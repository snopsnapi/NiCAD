#pragma once
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <string>
#include <vector>

namespace materializr {
namespace offsetface {

struct Options {
    // Remove fillets/chamfers that touch an offset face before offsetting.
    // Applied only when distance > 0 (moving the face outwards).
    bool removeAdjacentBlends = true;
    // A neighbour counts as a blend only if it is at most this wide (mm).
    double maxBlendWidth = 10.0;
};

struct Result {
    TopoDS_Shape shape;                       // valid solid on success
    std::vector<TopoDS_Face> removedBlends;   // blend faces of the INPUT body that were removed
    std::string error;                        // human-readable reason on failure
};

// Faces of `body` adjacent to `face` (sharing an edge) that look like a fillet or chamfer:
//  * fillet  - non-planar face (cylinder / torus / sphere / B-spline / revolution) tangent (normals within 1 deg)
//              to `face` along the shared edge AND tangent to the face on its opposite side;
//  * chamfer - planar face, NOT tangent to `face` (angle between normals 5..85 deg) and narrow.
// "Narrow": the largest distance from the shared edge to any vertex of the candidate <= maxWidth.
// Never returns `face` itself. Order: TopExp map order (deterministic).
std::vector<TopoDS_Face> findAdjacentBlends(const TopoDS_Shape& body, const TopoDS_Face& face,
                                            double maxWidth);

// Offset `faces` of `body` by `distance` along their outward normals; all other faces offset 0.
// Returns false and fills r.error on failure; on success r.shape is a BRepCheck-valid solid.
bool offsetFaces(const TopoDS_Shape& body, const std::vector<TopoDS_Face>& faces,
                 double distance, const Options& opt, Result& r);

} // namespace offsetface
} // namespace materializr
