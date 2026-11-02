#pragma once
// Surface modelling geometry (fork): the pure OCCT side of the Surface
// workspace - delete faces without healing, extrude open/closed curves into
// surfaces, sweep a curve or face along a path, and trim a surface with a
// tool. No Document, no UI: the operations (DefeatureOp, SurfaceExtrudeOp,
// SweepOp, TrimSurfaceOp) call these, and tests/test_surface_ops.cpp checks
// them directly.

#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <vector>

namespace materializr {
namespace surf {

// Every face of `body` except `remove`, as an open shell (a compound of
// shells when what is left falls apart). No healing - the gaps stay open.
// Null when nothing is left or no listed face belongs to the body.
TopoDS_Shape removeFacesOpen(const TopoDS_Shape& body,
                             const std::vector<TopoDS_Face>& remove);

// Each wire swept straight along `v` into a surface (prism of a wire = a
// shell, never a solid). A compound for several wires. Null on failure or a
// zero vector.
TopoDS_Shape extrudeCurves(const std::vector<TopoDS_Wire>& wires, const gp_Vec& v);

// `profile` (a wire, edge or face) swept along `path`. A wire/edge profile
// gives a surface; a face gives a solid. Null on failure.
TopoDS_Shape sweep(const TopoDS_Shape& profile, const TopoDS_Wire& path);

// Planar curves as a cutting tool: each wire extruded `extent` to both sides
// of its plane along `normal`, so the tool runs right through a target.
TopoDS_Shape curveCutter(const std::vector<TopoDS_Wire>& wires,
                         const gp_Vec& normal, double extent);

// `target` split by `tool` (BRepAlgoAPI_Splitter). The target's faces come
// back cut along the tool; the tool itself is not part of the result. Null
// on failure.
TopoDS_Shape splitByTool(const TopoDS_Shape& target, const TopoDS_Shape& tool);

// Trim: split `target` by `tool`, then drop every piece (face) nearest one
// of `removeAt` (world points, one per piece to discard). What is left comes
// back as removeFacesOpen does. Null when the split fails or nothing is left.
TopoDS_Shape trim(const TopoDS_Shape& target, const TopoDS_Shape& tool,
                  const std::vector<gp_Pnt>& removeAt);

// The face of `shape` nearest `p` (null when the shape has no faces).
TopoDS_Face nearestFace(const TopoDS_Shape& shape, const gp_Pnt& p);

// A point on `face` well inside it (UV-centre projected back when the centre
// falls in a hole) - where a piece is "tapped" when picking it.
gp_Pnt facePoint(const TopoDS_Face& face);

} // namespace surf
} // namespace materializr
