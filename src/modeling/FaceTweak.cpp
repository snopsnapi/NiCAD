#include "FaceTweak.h"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BOPAlgo_ArgumentAnalyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <GeomAPI_IntCS.hxx>
#include <GeomAPI_IntSS.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Plane.hxx>
#include <Geom_Surface.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Precision.hxx>
#include <ShapeFix_Face.hxx>
#include <ShapeFix_Shape.hxx>
#include <ShapeFix_Wire.hxx>
#include <Standard_ErrorHandler.hxx> // OCC_CATCH_SIGNALS
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Pln.hxx>
#include <BOPAlgo_MakerVolume.hxx>
#include <BOPTools_AlgoTools3D.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBndLib.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepLib.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomLProp_SLProps.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <IntTools_Context.hxx>
#include <gp_Cone.hxx>
#include <gp_Pnt2d.hxx>
#include <Bnd_Box.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopTools_MapOfShape.hxx>
#include "UnifyTolerance.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <set>
#include <vector>

namespace materializr::tweak {

namespace {

// Shape-keyed lookup. TopoDS handles have no operator<, and the identity we
// want is IsSame: the same vertex reached through two different faces has to
// land on one entry.
struct SameKey {
    bool operator()(const TopoDS_Shape& a, const TopoDS_Shape& b) const {
        if (a.TShape().get() != b.TShape().get())
            return a.TShape().get() < b.TShape().get();
        return a.Location().HashCode() < b.Location().HashCode();
    }
};
template <class V> using ShapeMap = std::map<TopoDS_Shape, V, SameKey>;
using ShapeSet = std::set<TopoDS_Shape, SameKey>;

bool planeOf(const TopoDS_Face& f, gp_Pln& out) {
    BRepAdaptor_Surface s(f, Standard_False);
    if (s.GetType() != GeomAbs_Plane) return false;
    out = s.Plane();
    return true;
}

double volumeOf(const TopoDS_Shape& s) {
    GProp_GProps g;
    BRepGProp::VolumeProperties(s, g);
    return g.Mass();
}

double areaOf(const TopoDS_Shape& s) {
    GProp_GProps g;
    BRepGProp::SurfaceProperties(s, g);
    return g.Mass();
}

gp_Pnt midOf(const TopoDS_Edge& e) {
    double f, l;
    Handle(Geom_Curve) c = BRep_Tool::Curve(e, f, l);
    if (c.IsNull()) return gp_Pnt();
    return c->Value(0.5 * (f + l));
}

// Where a curve crosses a plane, nearest `near`. This is the corner solve: the
// edge running off the moved face keeps its geometry, so the new corner is
// simply where that edge now meets the face's new plane. Works for a line, an
// arc, a spline - which is the whole reason the neighbours are free to curve.
// `nearTo`, not `near`: windows.h defines near and far as empty macros, and
// OCCT 7.9.3 leaks windows.h into its own headers, so a parameter called `near`
// vanishes on MSVC and takes the comma with it. Costs nothing to avoid; costs a
// red Windows build to discover.
bool curveMeetsPlane(const Handle(Geom_Curve)& c, double f, double l,
                     const gp_Pln& pl, const gp_Pnt& nearTo, gp_Pnt& out) {
    if (c.IsNull()) return false;
    Handle(Geom_Plane) gp = new Geom_Plane(pl);
    GeomAPI_IntCS ics(c, gp);
    if (!ics.IsDone() || ics.NbPoints() == 0) return false;

    // A circle can cross a plane twice and a spline more often than that. The
    // corner we want is the one the edge already had, so pick by proximity to
    // where the vertex sits now - not, say, the first solution the kernel
    // happens to report.
    // Deliberately NOT clamped to the edge's own parameter range. Pushing a
    // face outward puts the new corner PAST the end of the edge it slides
    // along - a box growing taller needs its vertical edges to lengthen, and
    // they lengthen along the same carrier line. Clamping looked like a
    // sensible guard and silently broke every outward move while leaving tilts
    // working, because a tilt keeps the corners between the old ends.
    (void)f;
    (void)l;
    bool found = false;
    double best = 1e300;
    for (int i = 1; i <= ics.NbPoints(); ++i) {
        const gp_Pnt p = ics.Point(i);
        const double d = p.Distance(nearTo);
        if (d < best) { best = d; out = p; found = true; }
    }
    return found;
}

// Parameter of `p` on `c`, and whether it actually lies there.
bool paramOn(const Handle(Geom_Curve)& c, const gp_Pnt& p, double tol, double& out) {
    GeomAPI_ProjectPointOnCurve proj(p, c);
    if (proj.NbPoints() == 0) return false;
    if (proj.LowerDistance() > tol) return false;
    out = proj.LowerDistanceParameter();
    return true;
}

} // namespace

const char* refusalText(Refusal r) {
    switch (r) {
        case Refusal::None:              return "";
        case Refusal::FaceNotFound:      return "that face isn't part of this body";
        case Refusal::NotPlanar:         return "only flat faces can be tweaked yet";
        case Refusal::NonManifoldCorner: return "a corner of this face doesn't have a single edge running into the body";
        case Refusal::NoChange:          return "sliding a flat face inside its own plane doesn't move anything - tilt it, or push it along its normal";
        case Refusal::Degenerate:        return "that move takes the face past an edge it has to stay attached to";
        case Refusal::NoIntersection:    return "a neighbouring face no longer meets this one after that move";
        case Refusal::OffCurve:          return "an edge running off this face can't reach its new corner";
        case Refusal::BuildFailed:       return "the rebuilt body wouldn't close up";
        case Refusal::Invalid:           return "the rebuilt body failed validation";
        case Refusal::SelfIntersects:    return "that move pushes a face through another one (for example a hole would break out through a wall) - this isn't supported yet";
    }
    return "";
}

static Result moveFaceCore(const TopoDS_Shape& body, const TopoDS_Face& face,
                           const gp_Trsf& xf) {
    Result res;
    if (body.IsNull() || face.IsNull()) {
        res.refusal = Refusal::FaceNotFound;
        return res;
    }

    try {
        OCC_CATCH_SIGNALS

        bool onBody = false;
        for (TopExp_Explorer ex(body, TopAbs_FACE); ex.More(); ex.Next())
            if (ex.Current().IsSame(face)) { onBody = true; break; }
        if (!onBody) { res.refusal = Refusal::FaceNotFound; return res; }

        gp_Pln planeF;
        if (!planeOf(face, planeF)) { res.refusal = Refusal::NotPlanar; return res; }
        const gp_Pln movedF = planeF.Transformed(xf);

        // A move that leaves the plane where it was would rebuild every corner
        // onto its own position and report success having changed nothing. Say
        // so instead - the user made a gesture and deserves to know why the
        // body didn't budge.
        if (planeF.Axis().Direction().IsParallel(movedF.Axis().Direction(), 1e-9) &&
            std::abs(movedF.Distance(planeF.Location())) < Precision::Confusion()) {
            res.refusal = Refusal::NoChange;
            return res;
        }

        TopTools_IndexedDataMapOfShapeListOfShape vertEdges, edgeFaces;
        TopExp::MapShapesAndAncestors(body, TopAbs_VERTEX, TopAbs_EDGE, vertEdges);
        TopExp::MapShapesAndAncestors(body, TopAbs_EDGE, TopAbs_FACE, edgeFaces);

        // The edges and vertices of the face being moved.
        ShapeSet edgesOnF, vertsOnF;
        for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next())
            if (!BRep_Tool::Degenerated(TopoDS::Edge(ex.Current())))
                edgesOnF.insert(ex.Current());
        for (TopExp_Explorer ex(face, TopAbs_VERTEX); ex.More(); ex.Next())
            vertsOnF.insert(ex.Current());
        if (edgesOnF.empty() || vertsOnF.empty()) {
            res.refusal = Refusal::BuildFailed;
            return res;
        }

        // ── Corners: where each leaving edge now crosses the moved plane ─────
        ShapeMap<gp_Pnt> movedVertices;
        for (const TopoDS_Shape& vs : vertsOnF) {
            const TopoDS_Vertex v = TopoDS::Vertex(vs);
            if (!vertEdges.Contains(v)) { res.refusal = Refusal::NonManifoldCorner; return res; }

            // The one edge at this corner that runs INTO the body. Its geometry
            // is untouched by the move - neither of its faces is the one being
            // moved - so it is the fixed thing the new corner slides along.
            TopoDS_Edge leaving;
            int nLeaving = 0;
            for (TopTools_ListIteratorOfListOfShape it(vertEdges.FindFromKey(v));
                 it.More(); it.Next()) {
                const TopoDS_Edge e = TopoDS::Edge(it.Value());
                if (BRep_Tool::Degenerated(e)) continue;
                if (edgesOnF.count(e)) continue;
                bool dup = false;
                if (!leaving.IsNull() && leaving.IsSame(e)) dup = true;
                if (dup) continue;
                leaving = e;
                ++nLeaving;
            }
            // Exactly one is the ordinary case: three faces at a corner, two of
            // their shared edges on the moved face and one heading away. Zero or
            // several means the corner isn't one this can solve - a vertex
            // buried inside the face's own wire, or four faces meeting at a
            // point.
            if (nLeaving != 1) { res.refusal = Refusal::NonManifoldCorner; return res; }

            double lf, ll;
            Handle(Geom_Curve) lc = BRep_Tool::Curve(leaving, lf, ll);
            gp_Pnt np;
            if (!curveMeetsPlane(lc, lf, ll, movedF, BRep_Tool::Pnt(v), np)) {
                res.refusal = Refusal::Degenerate;
                return res;
            }
            movedVertices[v] = np;
        }

        // Fresh vertices, minted once so every face sharing a corner shares the
        // same TopoDS_Vertex and the shell sews without a tolerance hunt.
        ShapeMap<TopoDS_Vertex> newVertex;
        for (const auto& [oldV, pnt] : movedVertices)
            newVertex[oldV] = BRepBuilderAPI_MakeVertex(pnt);

        auto vertexFor = [&](const TopoDS_Vertex& v) {
            auto it = newVertex.find(v);
            return it == newVertex.end() ? v : it->second;
        };
        auto pointFor = [&](const TopoDS_Vertex& v) {
            auto it = movedVertices.find(v);
            return it == movedVertices.end() ? BRep_Tool::Pnt(v) : it->second;
        };
        auto isMoved = [&](const TopoDS_Vertex& v) { return movedVertices.count(v) > 0; };

        // ── Edges on the moved face: the new plane against each neighbour ────
        ShapeMap<TopoDS_Edge> newEdge;
        for (const TopoDS_Shape& es : edgesOnF) {
            const TopoDS_Edge e = TopoDS::Edge(es);
            TopoDS_Vertex v1, v2;
            TopExp::Vertices(e, v1, v2);
            if (!isMoved(v1) || !isMoved(v2)) { res.refusal = Refusal::NonManifoldCorner; return res; }

            // The neighbour across this edge.
            TopoDS_Face nb;
            if (edgeFaces.Contains(e)) {
                for (TopTools_ListIteratorOfListOfShape it(edgeFaces.FindFromKey(e));
                     it.More(); it.Next()) {
                    if (it.Value().IsSame(face)) continue;
                    nb = TopoDS::Face(it.Value());
                    break;
                }
            }
            if (nb.IsNull()) { res.refusal = Refusal::BuildFailed; return res; }

            gp_Pln nbPlane;
            const bool nbPlanar = planeOf(nb, nbPlane);
            const bool closed = v1.IsSame(v2);

            if (nbPlanar && !closed) {
                // Two planes meet in a line, and both of its ends are already
                // solved exactly. Building the segment straight between them is
                // the same answer as intersecting the surfaces, without asking
                // the kernel for an approximation of a line.
                BRepBuilderAPI_MakeEdge mk(vertexFor(v1), vertexFor(v2));
                if (!mk.IsDone()) { res.refusal = Refusal::BuildFailed; return res; }
                newEdge[e] = mk.Edge();
                continue;
            }

            // Curved neighbour (or a closed edge, which no straight segment can
            // be): intersect the surfaces for real. A plane through a cylinder
            // gives a circle or an ellipse, through a cone a conic - the kernel
            // returns these analytically for quadrics, so the rebuilt edge is
            // exact rather than a sampled approximation.
            Handle(Geom_Surface) nbSurf = BRep_Tool::Surface(nb);
            Handle(Geom_Plane) movedSurf = new Geom_Plane(movedF);
            GeomAPI_IntSS iss(movedSurf, nbSurf, Precision::Confusion());
            if (!iss.IsDone() || iss.NbLines() == 0) {
                res.refusal = Refusal::NoIntersection;
                return res;
            }
            // A plane can cut a surface in several branches (both nappes of a
            // cone, two lines through a cylinder). The branch we want is the one
            // this edge is already on, judged from its midpoint.
            const gp_Pnt oldMid = midOf(e);
            Handle(Geom_Curve) best;
            double bestD = 1e300;
            for (int i = 1; i <= iss.NbLines(); ++i) {
                Handle(Geom_Curve) c = iss.Line(i);
                if (c.IsNull()) continue;
                GeomAPI_ProjectPointOnCurve proj(oldMid, c);
                if (proj.NbPoints() == 0) continue;
                if (proj.LowerDistance() < bestD) { bestD = proj.LowerDistance(); best = c; }
            }
            if (best.IsNull()) { res.refusal = Refusal::NoIntersection; return res; }
            // GeomAPI_IntSS hands back its analytic results wrapped in a
            // Geom_TrimmedCurve. The wrapper reports IsPeriodic() == false even
            // around a circle, so a closed rim looked un-closable; unwrap to the
            // basis curve, which knows it is periodic and can be walked a full
            // turn from any parameter.
            while (!best.IsNull() &&
                   best->IsKind(STANDARD_TYPE(Geom_TrimmedCurve)))
                best = Handle(Geom_TrimmedCurve)::DownCast(best)->BasisCurve();
            if (best.IsNull()) { res.refusal = Refusal::NoIntersection; return res; }

            const double tol = std::max(BRep_Tool::Tolerance(e), 1e-5);
            if (closed) {
                // A full loop - the lid of a cylinder, the rim of a bore. It has
                // ONE vertex, sitting where the neighbour's seam runs into it,
                // and that vertex is shared with the neighbour's seam edges. So
                // the loop has to START there: built from the bare curve instead,
                // BRepBuilderAPI_MakeEdge mints its own vertex at the curve's
                // parameter zero, which lands somewhere else entirely and leaves
                // the shell with two coincident-ish vertices it cannot sew.
                double pv = 0.0;
                if (!best->IsPeriodic() ||
                    !paramOn(best, pointFor(v1), tol * 100.0, pv)) {
                    res.refusal = Refusal::OffCurve;
                    return res;
                }
                BRepBuilderAPI_MakeEdge mk(best, vertexFor(v1), vertexFor(v1),
                                           pv, pv + best->Period());
                if (!mk.IsDone()) { res.refusal = Refusal::BuildFailed; return res; }
                newEdge[e] = mk.Edge();
                continue;
            }

            double p1 = 0.0, p2 = 0.0;
            if (!paramOn(best, pointFor(v1), tol * 100.0, p1) ||
                !paramOn(best, pointFor(v2), tol * 100.0, p2)) {
                res.refusal = Refusal::OffCurve;
                return res;
            }
            // On a periodic curve two arcs join the same pair of points, and
            // taking the wrong one turns the face inside out. Pick the arc whose
            // own midpoint sits nearest where this edge already runs.
            if (best->IsPeriodic()) {
                const double period = best->Period();
                const gp_Pnt aMid = best->Value(0.5 * (p1 + p2));
                const double p2Alt = (p2 > p1) ? p2 - period : p2 + period;
                const gp_Pnt bMid = best->Value(0.5 * (p1 + p2Alt));
                if (bMid.Distance(oldMid) < aMid.Distance(oldMid)) p2 = p2Alt;
            }
            if (std::abs(p2 - p1) < Precision::PConfusion()) {
                res.refusal = Refusal::Degenerate;
                return res;
            }
            BRepBuilderAPI_MakeEdge mk(best, vertexFor(v1), vertexFor(v2), p1, p2);
            if (!mk.IsDone()) { res.refusal = Refusal::BuildFailed; return res; }
            newEdge[e] = mk.Edge();
        }

        // ── Edges leaving the face: same curve, new end ──────────────────────
        for (int i = 1; i <= edgeFaces.Extent(); ++i) {
            const TopoDS_Edge e = TopoDS::Edge(edgeFaces.FindKey(i));
            if (BRep_Tool::Degenerated(e) || edgesOnF.count(e)) continue;
            TopoDS_Vertex v1, v2;
            TopExp::Vertices(e, v1, v2);
            const bool m1 = isMoved(v1), m2 = isMoved(v2);
            if (!m1 && !m2) continue;

            double f0, l0;
            Handle(Geom_Curve) c = BRep_Tool::Curve(e, f0, l0);
            if (c.IsNull()) { res.refusal = Refusal::BuildFailed; return res; }

            // Neither of this edge's faces moved, so its curve is untouched -
            // only the end that sat on the moved face slides along it. The new
            // corner came OFF this curve, so it lies on it by construction; the
            // check is here for the case where it came off a different edge at a
            // shared corner.
            double p1 = f0, p2 = l0;
            const double tol = std::max(BRep_Tool::Tolerance(e), 1e-6) * 100.0;
            if (m1 && !paramOn(c, pointFor(v1), tol, p1)) { res.refusal = Refusal::OffCurve; return res; }
            if (m2 && !paramOn(c, pointFor(v2), tol, p2)) { res.refusal = Refusal::OffCurve; return res; }
            if (std::abs(p2 - p1) < Precision::PConfusion()) {
                res.refusal = Refusal::Degenerate;
                return res;
            }
            BRepBuilderAPI_MakeEdge mk(c, vertexFor(v1), vertexFor(v2),
                                       std::min(p1, p2), std::max(p1, p2));
            if (!mk.IsDone()) { res.refusal = Refusal::BuildFailed; return res; }
            newEdge[e] = mk.Edge();
        }

        // ── Faces: rebuild any that carry a rebuilt edge ─────────────────────
        BRepBuilderAPI_Sewing sew(Precision::Confusion() * 10.0);
        int rebuilt = 0;
        for (TopExp_Explorer fx(body, TopAbs_FACE); fx.More(); fx.Next()) {
            const TopoDS_Face f = TopoDS::Face(fx.Current());
            bool touched = false;
            for (TopExp_Explorer ex(f, TopAbs_EDGE); ex.More(); ex.Next())
                if (newEdge.count(ex.Current())) { touched = true; break; }
            if (!touched) { sew.Add(f); continue; }

            // The moved face gets the new plane; a neighbour keeps its own
            // surface and only its boundary changes.
            Handle(Geom_Surface) surf = f.IsSame(face)
                ? Handle(Geom_Surface)(new Geom_Plane(movedF))
                : BRep_Tool::Surface(f);
            if (surf.IsNull()) { res.refusal = Refusal::BuildFailed; return res; }

            // Wire order comes from BRepTools_WireExplorer, which walks
            // connected - a plain TopExp_Explorer returns edges in map order and
            // MakeWire then refuses the ones that don't arrive adjacent.
            std::vector<TopoDS_Wire> wires;
            bool wireOk = true;
            for (TopExp_Explorer wx(f, TopAbs_WIRE); wx.More() && wireOk; wx.Next()) {
                BRepBuilderAPI_MakeWire mw;
                int edges = 0;
                for (BRepTools_WireExplorer we(TopoDS::Wire(wx.Current()), f);
                     we.More(); we.Next()) {
                    // A degenerate edge is a parametric artefact - a cone apex,
                    // a sphere pole. It has no 3d curve to rebuild and carrying
                    // it into a wire of new edges only makes MakeWire refuse.
                    if (BRep_Tool::Degenerated(we.Current())) continue;
                    auto it = newEdge.find(we.Current());
                    if (it == newEdge.end()) {
                        mw.Add(we.Current());
                    } else {
                        // Carry the orientation the wire uses. WireExplorer
                        // hands back the edge as this wire reads it; the
                        // replacement was built FORWARD, and adding it that way
                        // reverses half the boundary. A planar face survives
                        // that because ShapeFix re-winds it, but a cylinder's
                        // SEAM appears in the wire twice with opposite
                        // orientations, so dropping them makes the same edge
                        // twice over and the face fails validation.
                        mw.Add(TopoDS::Edge(it->second.Oriented(
                            we.Current().Orientation())));
                    }
                    ++edges;
                }
                if (edges == 0) continue;
                if (!mw.IsDone()) { wireOk = false; break; }
                wires.push_back(mw.Wire());
            }
            if (!wireOk || wires.empty()) { res.refusal = Refusal::BuildFailed; return res; }

            BRepBuilderAPI_MakeFace mf(surf, wires.front(), Precision::Confusion());
            if (!mf.IsDone()) { res.refusal = Refusal::BuildFailed; return res; }
            for (size_t i = 1; i < wires.size(); ++i) {
                mf.Add(wires[i]);
                if (!mf.IsDone()) { res.refusal = Refusal::BuildFailed; return res; }
            }

            // Two jobs for ShapeFix here, and the second is what curved
            // neighbours need: the rebuilt edges carry 3d curves only, and a
            // face on a cylinder is meaningless to the kernel until each of its
            // edges has a pcurve in that cylinder's (u,v). The first is the
            // long-standing one - wire winding decides which side of a face is
            // material and is never coordinated by hand here.
            ShapeFix_Face fix(mf.Face());
            fix.FixOrientationMode() = 1;
            fix.FixWireMode() = 1;
            // A cylinder or cone rebuilt from its wire has to be told where its
            // seam is, or the face is left open in (u,v) and fails validation.
            fix.FixMissingSeamMode() = 1;
            // Adding pcurves is the WIRE fixer's job, reached through the face
            // fixer's tool - it is on by default, but stating it keeps the
            // dependency visible: without it a rebuilt cylindrical face has
            // edges with 3d curves and no (u,v) representation, and the sew
            // downstream quietly drops it.
            if (!fix.FixWireTool().IsNull())
                fix.FixWireTool()->FixAddPCurveMode() = 1;
            fix.Perform();
            const TopoDS_Face nf = fix.Face();
            if (areaOf(nf) <= 1e-12) { res.refusal = Refusal::BuildFailed; return res; }
            sew.Add(nf);
            ++rebuilt;
        }
        if (rebuilt == 0) { res.refusal = Refusal::BuildFailed; return res; }

        // ── Close it back up ────────────────────────────────────────────────
        sew.Perform();
        TopoDS_Shape sewn = sew.SewedShape();
        if (sewn.IsNull() || sew.NbFreeEdges() > 0) {
            res.refusal = Refusal::BuildFailed;
            return res;
        }

        // A healing pass over the assembled shell. The per-face ShapeFix above
        // can only see one face at a time, and the things that survive it are
        // exactly the ones that are only wrong in context: a seam edge rebuilt
        // from its 3d curve has lost BOTH of its pcurves on the cylinder it
        // divides, and no single-face fix restores the pair. Cheap on a shell
        // this size, and the result is checked below either way.
        {
            ShapeFix_Shape heal(sewn);
            heal.SetPrecision(Precision::Confusion());
            heal.SetMaxTolerance(1e-3);
            heal.Perform();
            if (!heal.Shape().IsNull()) sewn = heal.Shape();
        }
        for (TopExp_Explorer ex(sewn, TopAbs_SHELL); ex.More(); ex.Next()) {
            TopoDS_Shell shell = TopoDS::Shell(ex.Current());
            BRepBuilderAPI_MakeSolid ms(shell);
            if (!ms.IsDone()) break;
            TopoDS_Shape solid = ms.Solid();
            // Sewing settles orientation among the faces but not which side is
            // inside; a negative volume is an inside-out solid, and reversing
            // the shell is the whole fix.
            if (volumeOf(solid) < 0.0) {
                shell.Reverse();
                BRepBuilderAPI_MakeSolid ms2(shell);
                if (!ms2.IsDone()) break;
                solid = ms2.Solid();
            }
            if (!BRepCheck_Analyzer(solid).IsValid()) {
                res.refusal = Refusal::Invalid;
                return res;
            }
            if (volumeOf(solid) <= 1e-9) { res.refusal = Refusal::Degenerate; return res; }
            // Fork: every rebuilt face is valid on its own, so BRepCheck passes
            // a body whose stretched neighbour now runs THROUGH another face
            // (a bore tilted out through a side wall). That needs a topology
            // change this engine doesn't make - refuse instead of handing back
            // a self-intersecting solid.
            {
                BOPAlgo_ArgumentAnalyzer an;
                an.SetShape1(solid);
                an.SelfInterMode() = Standard_True;
                an.Perform();
                if (an.HasFaulty()) { res.refusal = Refusal::SelfIntersects; return res; }
            }
            res.shape = solid;
            return res;
        }
        res.refusal = Refusal::BuildFailed;
        return res;
    } catch (...) {
        res.refusal = Refusal::BuildFailed;
        return res;
    }
}

} // namespace materializr::tweak

namespace materializr::tweak {

// ─── Extend-and-rebuild: the general engine ─────────────────────────────────
//
// moveFaceCore() above keeps the body's topology as it is - every corner
// slides along its own edge - so anything that needs the topology to CHANGE
// is out of its reach: a bore that breaks out through a wall, a neighbour that
// shrinks to nothing, a notch on the face's rim, a corner where four faces
// meet. Each of those used to get a fallback of its own, and every new case
// fell between them.
//
// This engine lets the boolean kernel decide the topology, the same way OCCT's
// own feature removal (BOPAlgo_RemoveFeatures) fills a removed feature:
//
//   1. Drop the moved face. Every face that shared an edge with it (outer
//      walls, a bore's wall, a boss's wall, a notch) is EXTENDED along its own
//      surface - a plane to a big rectangle, a cylinder to a long full tube.
//   2. Add the face's new plane as one big face.
//   3. BOPAlgo_MakerVolume splits all of that against itself and returns every
//      closed cell. Breakouts, vanishing faces and new edges are just how the
//      cells come out.
//   4. Keep a cell if it is material: it carries a piece of an ORIGINAL face of
//      the body (a non-moved face, or the original region of an extended one)
//      and lies on that face's inner side. The region over the old face up to
//      the new plane joins the body's own cell (the old face is gone), a bore's
//      column has the bore's floor on its outside, the space a sunk face leaves
//      is open to the outside and never closes into a cell at all.
//   5. Fuse the kept cells, merge the split faces back (UnifySameDomain).
//
// Nothing here depends on the neighbours being planes: any surface that can be
// extended works, which is also what a curved moved face will need later.

namespace {

// The face's outward normal (orientation-aware).
bool outwardPlane(const TopoDS_Face& f, gp_Pln& pl, gp_Dir& n) {
    BRepAdaptor_Surface s(f, Standard_False);
    if (s.GetType() != GeomAbs_Plane) return false;
    pl = s.Plane();
    n = pl.Axis().Direction();
    if (f.Orientation() == TopAbs_REVERSED) n.Reverse();
    return true;
}

constexpr double kTwoPi = 6.283185307179586;

Handle(Geom_Surface) basisOf(const TopoDS_Face& f) {
    Handle(Geom_Surface) s = BRep_Tool::Surface(f);   // location applied
    while (!s.IsNull() && s->IsKind(STANDARD_TYPE(Geom_RectangularTrimmedSurface)))
        s = Handle(Geom_RectangularTrimmedSurface)::DownCast(s)->BasisSurface();
    return s;
}

// What stands in for neighbour `g` of the moved face in the rebuild.
//
// A neighbour on the face's OUTER loop (a wall) is extended its whole surface
// over, both ways: the face's region is what its walls enclose, and a wall
// may have to run on past its far edge too - an L-shaped step whose top sinks
// below the other top needs the step wall carried on down.
//
// A neighbour reached only through an INNER loop (a bore's wall, a boss's
// wall) is carried on only ACROSS the edges it shares with the face
// (`shared`): a bore runs on up to the new face, a boss's wall on down into
// the body - never up past the boss's own top, which would wall off the space
// a buried boss leaves.
//   plane              -> a big rectangle (full), or g plus a wide strip past
//                         each shared line (one-sided)
//   cylinder, cone     -> the full tube, lengthened at both / the shared end
//   anything else      -> the whole surface extended (BRepLib::ExtendFace)
std::vector<TopoDS_Face> extendAcross(const TopoDS_Face& g,
                                      const std::vector<TopoDS_Edge>& shared,
                                      bool full, double L) {
    std::vector<TopoDS_Face> out;
    const Handle(Geom_Surface) s = basisOf(g);
    if (s.IsNull()) return out;
    const double tol = Precision::Confusion();
    BRepAdaptor_Surface a(g, Standard_False);
    double u1, u2, v1, v2;
    BRepTools::UVBounds(g, u1, u2, v1, v2);

    if (a.GetType() == GeomAbs_Plane && full) {
        out.push_back(BRepBuilderAPI_MakeFace(s, u1 - L, u2 + L, v1 - L, v2 + L, tol).Face());
        return out;
    }
    if (a.GetType() == GeomAbs_Plane) {
        out.push_back(g);
        const gp_Dir ng = a.Plane().Axis().Direction();
        std::vector<gp_Lin> done;
        for (const TopoDS_Edge& e : shared) {
            BRepAdaptor_Curve c(e);
            if (c.GetType() != GeomAbs_Line) continue;   // plane meets plane in a line
            const gp_Lin ln = c.Line();
            bool dup = false;
            for (const gp_Lin& d : done)
                if (d.Direction().IsParallel(ln.Direction(), 1e-9) &&
                    d.Distance(ln.Location()) < 1e-7) { dup = true; break; }
            if (dup) continue;
            done.push_back(ln);
            // In-plane, perpendicular to the line, pointing out of g.
            gp_Dir away = ng.Crossed(ln.Direction());
            const gp_Pnt mid = c.Value(0.5 * (c.FirstParameter() + c.LastParameter()));
            const double probe = std::max(1e-4, 1e-3 * (c.LastParameter() - c.FirstParameter()));
            BRepClass_FaceClassifier fc(g, mid.Translated(gp_Vec(away) * probe), tol * 10.0);
            if (fc.State() == TopAbs_IN) away.Reverse();
            const gp_Pnt o = ln.Location();   // the line's own origin
            const gp_Vec along(ln.Direction());
            // Lateral reach measured from the edge's middle, not the line's origin.
            const double tMid = gp_Vec(o, mid).Dot(along);
            const gp_Pnt base = o.Translated(along * tMid);
            const gp_Pnt p0 = base.Translated(along * -L);
            const gp_Pnt p1 = base.Translated(along * L);
            const gp_Pnt p2 = p1.Translated(gp_Vec(away) * L);
            const gp_Pnt p3 = p0.Translated(gp_Vec(away) * L);
            BRepBuilderAPI_MakePolygon poly(p0, p1, p2, p3, Standard_True);
            if (!poly.IsDone()) continue;
            BRepBuilderAPI_MakeFace mf(gp_Pln(base, ng), poly.Wire(), Standard_True);
            if (mf.IsDone()) out.push_back(mf.Face());
        }
        return out;
    }

    if (a.GetType() == GeomAbs_Cylinder || a.GetType() == GeomAbs_Cone) {
        // Which end of the tube the shared edges sit at.
        double ev1 = 1e300, ev2 = -1e300;
        for (const TopoDS_Edge& e : shared) {
            double eu1, eu2, w1, w2;
            BRepTools::UVBounds(g, e, eu1, eu2, w1, w2);
            ev1 = std::min(ev1, w1);
            ev2 = std::max(ev2, w2);
        }
        const double vt = 1e-6 * std::max(1.0, v2 - v1);
        double lo = v1, hi = v2;
        if (ev1 <= v1 + vt) lo = v1 - L;
        if (ev2 >= v2 - vt) hi = v2 + L;
        if (full || (lo == v1 && hi == v2)) { lo = v1 - L; hi = v2 + L; }
        if (a.GetType() == GeomAbs_Cone) {
            // Not past the apex: the second nappe is a different face.
            const gp_Cone cn = a.Cone();
            const double sa = std::sin(cn.SemiAngle());
            if (std::abs(sa) > 1e-12) {
                const double apex = -cn.RefRadius() / sa;
                const double gap = 1e-3 * std::max(1.0, std::abs(apex));
                if (sa > 0.0) lo = std::max(lo, apex + gap);
                else          hi = std::min(hi, apex - gap);
            }
        }
        out.push_back(BRepBuilderAPI_MakeFace(s, 0.0, kTwoPi, lo, hi, tol).Face());
        return out;
    }

    TopoDS_Face ext;
    if (a.GetType() == GeomAbs_Sphere || a.GetType() == GeomAbs_Torus)
        ext = BRepBuilderAPI_MakeFace(s, tol).Face();   // the whole surface
    else
        BRepLib::ExtendFace(g, L, Standard_True, Standard_True, Standard_True, Standard_True, ext);
    if (!ext.IsNull()) out.push_back(ext);
    return out;
}

// Outward normal of `f` (as oriented in its solid) at the point nearest `p`.
bool normalAt(const TopoDS_Face& f, const gp_Pnt& p, gp_Dir& n, double& dist) {
    const Handle(Geom_Surface) s = BRep_Tool::Surface(f);
    if (s.IsNull()) return false;
    GeomAPI_ProjectPointOnSurf pr(p, s);
    if (!pr.IsDone() || pr.NbPoints() == 0) return false;
    double u, v;
    pr.LowerDistanceParameters(u, v);
    dist = pr.LowerDistance();
    GeomLProp_SLProps props(s, u, v, 1, Precision::Confusion());
    if (!props.IsNormalDefined()) return false;
    n = props.Normal();
    if (f.Orientation() == TopAbs_REVERSED) n.Reverse();
    return true;
}

// Does the point `p` (on or very near `f`'s surface) fall inside `f`?
bool insideFace(const TopoDS_Face& f, const gp_Pnt& p, double tol) {
    BRepClass_FaceClassifier fc(f, p, tol);
    return fc.State() == TopAbs_IN || fc.State() == TopAbs_ON;
}

// Merge the faces the rebuild split back into one (UnifySameDomain), safely.
// A full merge sometimes can't be had: a bore wall that the new plane cuts into
// two pieces touching at a single point would merge into an invalid face, and
// then UnifySameDomain gives up on the WHOLE body - the flat faces stay split
// too, and the next move grabs half a face. So a full merge first; if that is
// not adoptable, a merge of flat faces only (every edge of a curved face kept).
// Each attempt works on its own copy: UnifySameDomain edits its input in place.
TopoDS_Shape mergeSplits(const TopoDS_Shape& shape) {
    const double v0 = volumeOf(shape);
    auto adoptable = [&](const TopoDS_Shape& m) {
        if (m.IsNull()) return false;
        if (std::abs(volumeOf(m) - v0) > 1e-4 * std::max(1.0, std::abs(v0))) return false;
        return BRepCheck_Analyzer(m).IsValid() == Standard_True;
    };
    for (int attempt = 0; attempt < 2; ++attempt) {
        try {
            OCC_CATCH_SIGNALS
            const TopoDS_Shape work = BRepBuilderAPI_Copy(shape).Shape();
            ShapeUpgrade_UnifySameDomain u(work, Standard_True, Standard_True, Standard_True);
            u.SetAngularTolerance(materializr::kUnifyAngularTol);
            if (attempt == 1) {
                TopTools_MapOfShape keep;
                for (TopExp_Explorer fx(work, TopAbs_FACE); fx.More(); fx.Next()) {
                    gp_Pln pl;
                    if (planeOf(TopoDS::Face(fx.Current()), pl)) continue;
                    for (TopExp_Explorer ex(fx.Current(), TopAbs_EDGE); ex.More(); ex.Next())
                        keep.Add(ex.Current());
                }
                u.KeepShapes(keep);
            }
            u.Build();
            if (adoptable(u.Shape())) return u.Shape();
        } catch (...) {}
    }
    return shape;
}

struct Original {
    TopoDS_Face face;   // as oriented in the body
    Bnd_Box box;
};

} // namespace

static Result rebuildMove(const TopoDS_Shape& body, const TopoDS_Face& face,
                          const gp_Trsf& xf) {
    static const bool dbg = std::getenv("MZ_TWEAK_DEBUG") != nullptr;
    auto say = [&](const char* m) { if (dbg) std::fprintf(stderr, "[tweak] %s\n", m); };
    Result res;
    try {
        OCC_CATCH_SIGNALS
        gp_Pln planeF;
        gp_Dir nF;
        if (!outwardPlane(face, planeF, nF)) { res.refusal = Refusal::NotPlanar; return res; }
        const gp_Pln movedF = planeF.Transformed(xf);
        gp_Dir nNew = nF;
        nNew.Transform(xf);

        Bnd_Box bb;
        BRepBndLib::Add(body, bb);
        const double size = bb.IsVoid() ? 10.0 : std::sqrt(bb.SquareExtent());
        GProp_GProps fp;
        BRepGProp::SurfaceProperties(face, fp);
        const gp_Pnt c0 = fp.CentreOfMass();
        const gp_Pnt c1 = c0.Transformed(xf);
        // How far the move can reach: the most any corner of the face travels
        // or sits off the new plane. Neighbours are extended a few times that
        // (a corner sliding down a steep wall travels further than the face
        // does), never across the whole body: an extension that runs on
        // through unrelated material only splits faces that then have to be
        // merged back, and the merge is the fragile part.
        double reach = c0.Distance(c1);
        Bnd_Box fb;
        BRepBndLib::Add(face, fb);
        const double sizeF = fb.IsVoid() ? size : std::sqrt(fb.SquareExtent());
        for (TopExp_Explorer vx(face, TopAbs_VERTEX); vx.More(); vx.Next()) {
            const gp_Pnt v = BRep_Tool::Pnt(TopoDS::Vertex(vx.Current()));
            reach = std::max(reach, v.Distance(v.Transformed(xf)));
            reach = std::max(reach, movedF.Distance(v));
        }
        const double L = std::min(4.0 * size + 2.0 * reach, 3.0 * reach + 0.25 * sizeF + 0.01 * size);

        // 1. The moved face's neighbours, extended; the rest as they are.
        TopTools_IndexedDataMapOfShapeListOfShape edgeFaces;
        TopExp::MapShapesAndAncestors(body, TopAbs_EDGE, TopAbs_FACE, edgeFaces);
        ShapeMap<std::vector<TopoDS_Edge>> adjacent;   // neighbour -> edges it shares with the face
        ShapeSet outerEdges, onOuter;                   // the face's outer loop; neighbours on it
        {
            const TopoDS_Wire ow = BRepTools::OuterWire(face);
            for (TopExp_Explorer ex(ow, TopAbs_EDGE); ex.More(); ex.Next()) outerEdges.insert(ex.Current());
        }
        for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next()) {
            const TopoDS_Edge e = TopoDS::Edge(ex.Current());
            if (BRep_Tool::Degenerated(e) || !edgeFaces.Contains(e)) continue;
            for (TopTools_ListIteratorOfListOfShape it(edgeFaces.FindFromKey(e)); it.More(); it.Next()) {
                if (it.Value().IsSame(face)) continue;
                if (outerEdges.count(e)) onOuter.insert(it.Value());
                auto& v = adjacent[it.Value()];
                bool have = false;
                for (const TopoDS_Edge& x : v) if (x.IsSame(e)) { have = true; break; }
                if (!have) v.push_back(e);
            }
        }
        if (adjacent.empty()) { res.refusal = Refusal::BuildFailed; return res; }

        TopTools_ListOfShape args;
        std::vector<Original> originals;
        for (TopExp_Explorer fx(body, TopAbs_FACE); fx.More(); fx.Next()) {
            const TopoDS_Face g = TopoDS::Face(fx.Current());
            if (g.IsSame(face)) continue;
            Original o;
            o.face = g;
            BRepBndLib::Add(g, o.box);
            o.box.Enlarge(1e-4 * size);
            originals.push_back(o);
            auto adj = adjacent.find(g);
            if (adj != adjacent.end()) {
                const std::vector<TopoDS_Face> ext = extendAcross(g, adj->second, onOuter.count(g) > 0, L);
                if (ext.empty()) { say("a neighbour wouldn't extend"); res.refusal = Refusal::BuildFailed; return res; }
                for (const TopoDS_Face& e : ext) args.Append(e);
            } else {
                args.Append(g);
            }
        }

        // 2. The new plane, centred where the face's centre went - and the old
        //    face itself, so the cells next to it can be told apart (step 4).
        gp_Pln bigPl(c1, nNew);
        const double B = sizeF + L;   // the new face can't spread further than that
        args.Append(BRepBuilderAPI_MakeFace(bigPl, -B, B, -B, B).Face());
        args.Append(face);

        // 3. Every closed cell.
        BOPAlgo_MakerVolume mv;
        mv.SetArguments(args);
        mv.SetIntersect(Standard_True);
        mv.SetRunParallel(Standard_False);
        mv.SetAvoidInternalShapes(Standard_True);
        mv.Perform();
        if (mv.HasErrors() || mv.Shape().IsNull()) { say("maker volume failed"); res.refusal = Refusal::BuildFailed; return res; }
        say("cells built");

        // 4. Which cells are material.
        //    A cell next to the old face (either side) is the region the move
        //    sweeps: material exactly when it lies under the new plane - that
        //    is the move itself, growing or cutting. Any other cell keeps what
        //    the body says: it is material if it lies on the inner side of a
        //    piece of an original face.
        Handle(IntTools_Context) ctx = new IntTools_Context;
        const double tolOn = std::max(1e-6, 1e-7 * size);
        Bnd_Box faceBox;
        BRepBndLib::Add(face, faceBox);
        faceBox.Enlarge(1e-4 * size);
        std::vector<TopoDS_Shape> kept;
        int nCells = 0;
        for (TopExp_Explorer sx(mv.Shape(), TopAbs_SOLID); sx.More(); sx.Next()) {
            ++nCells;
            const TopoDS_Shape cell = sx.Current();
            int in = 0, out = 0;
            bool touchesFace = false;
            for (TopExp_Explorer cf(cell, TopAbs_FACE); cf.More() && !touchesFace; cf.Next()) {
                const TopoDS_Face piece = TopoDS::Face(cf.Current());
                gp_Pnt q;
                gp_Pnt2d q2;
                if (BOPTools_AlgoTools3D::PointInFace(piece, q, q2, ctx) != 0) continue;
                gp_Dir nPiece;
                double d0 = 0.0;
                if (!normalAt(piece, q, nPiece, d0)) continue;
                if (!faceBox.IsOut(q) && planeF.Distance(q) < tolOn && insideFace(face, q, tolOn) &&
                    std::abs(nPiece.Dot(nF)) > 0.5) {
                    touchesFace = true;
                    if (dbg) std::fprintf(stderr, "[tweak]   face piece at (%.3f %.3f %.3f) n.nF %.3f\n",
                                          q.X(), q.Y(), q.Z(), nPiece.Dot(nF));
                    break;
                }
                for (const Original& o : originals) {
                    if (o.box.IsOut(q)) continue;
                    gp_Dir nOrig;
                    double d = 0.0;
                    if (!normalAt(o.face, q, nOrig, d) || d > tolOn) continue;
                    if (!insideFace(o.face, q, tolOn)) continue;
                    const double dot = nPiece.Dot(nOrig);
                    if (dot > 0.5) ++in;          // the cell is on the body's side of it
                    else if (dot < -0.5) ++out;   // the cell is outside the body there
                    break;
                }
            }
            bool keep;
            if (touchesFace) {
                // Wholly on one side of the new plane (it split everything),
                // so the centre of mass says which.
                GProp_GProps vp;
                BRepGProp::VolumeProperties(cell, vp);
                keep = gp_Vec(c1, vp.CentreOfMass()).Dot(gp_Vec(nNew)) < 0.0;
            } else {
                keep = in > 0 && in >= out;
            }
            if (dbg) {
                GProp_GProps vp;
                BRepGProp::VolumeProperties(cell, vp);
                const gp_Pnt cm = vp.CentreOfMass();
                std::fprintf(stderr, "[tweak]   cell vol %.4f at (%.3f %.3f %.3f): %s in %d out %d -> %s\n",
                             vp.Mass(), cm.X(), cm.Y(), cm.Z(), touchesFace ? "by the face," : "",
                             in, out, keep ? "keep" : "drop");
            }
            if (keep) kept.push_back(cell);
        }
        if (dbg) std::fprintf(stderr, "[tweak] cells %d, kept %zu\n", nCells, kept.size());
        if (kept.empty()) { res.refusal = Refusal::Degenerate; return res; }

        // 5. One solid again.
        TopoDS_Shape out = kept.front();
        if (kept.size() > 1) {
            TopTools_ListOfShape a, t;
            a.Append(kept.front());
            for (size_t i = 1; i < kept.size(); ++i) t.Append(kept[i]);
            BRepAlgoAPI_Fuse fu;
            fu.SetArguments(a);
            fu.SetTools(t);
            fu.SetRunParallel(Standard_False);
            fu.Build();
            if (!fu.IsDone() || fu.Shape().IsNull()) { res.refusal = Refusal::BuildFailed; return res; }
            out = fu.Shape();
        }
        out = mergeSplits(out);

        TopoDS_Shape solid;
        int nSolids = 0;
        for (TopExp_Explorer sx(out, TopAbs_SOLID); sx.More(); sx.Next()) { solid = sx.Current(); ++nSolids; }
        if (nSolids != 1) { say("not one solid"); res.refusal = Refusal::Degenerate; return res; }
        int shellsBefore = 0, shellsAfter = 0;
        for (TopExp_Explorer x(body, TopAbs_SHELL); x.More(); x.Next()) ++shellsBefore;
        for (TopExp_Explorer x(solid, TopAbs_SHELL); x.More(); x.Next()) ++shellsAfter;
        if (shellsAfter > shellsBefore) { say("a stray void"); res.refusal = Refusal::Degenerate; return res; }   // a stray void
        if (!BRepCheck_Analyzer(solid).IsValid()) { say("invalid"); res.refusal = Refusal::Invalid; return res; }
        if (volumeOf(solid) <= 1e-9) { res.refusal = Refusal::Degenerate; return res; }

        // The new plane has to show up: a move that buries or loses the face
        // entirely (pushed past the far side) is not a face move.
        bool hasNewFace = false;
        for (TopExp_Explorer fx(solid, TopAbs_FACE); fx.More() && !hasNewFace; fx.Next()) {
            gp_Pln q;
            gp_Dir n;
            if (!outwardPlane(TopoDS::Face(fx.Current()), q, n)) continue;
            if (n.Dot(nNew) > 1.0 - 1e-9 && movedF.Distance(q.Location()) < tolOn * 10.0) hasNewFace = true;
        }
        if (!hasNewFace) { say("no new face"); res.refusal = Refusal::Degenerate; return res; }

        res.shape = solid;
        return res;
    } catch (...) {
        res.refusal = Refusal::BuildFailed;
        return res;
    }
}

// The public entry. The vertex-sliding engine first: it is exact, cheap enough
// for every frame of a drag, and leaves the untouched faces as they were. When
// it can't (the move changes the body's topology), the extend-and-rebuild
// engine does it with the boolean kernel.
Result moveFace(const TopoDS_Shape& body, const TopoDS_Face& face, const gp_Trsf& xf) {
    const Result plain = moveFaceCore(body, face, xf);
    if (plain.ok() || plain.refusal == Refusal::NoChange ||
        plain.refusal == Refusal::NotPlanar || plain.refusal == Refusal::FaceNotFound)
        return plain;
    const Result r = rebuildMove(body, face, xf);
    return r.ok() ? r : plain;
}

} // namespace materializr::tweak
