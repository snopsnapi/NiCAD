#include "OffsetFace.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Defeaturing.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp_Face.hxx>
#include <BRepOffset_MakeOffset.hxx>
#include <BRepOffset_Mode.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomAbs_JoinType.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

namespace materializr {
namespace offsetface {
namespace {

constexpr double kPi = 3.14159265358979323846;

// Outward normal of `f` at the surface point nearest to `p` (`p` is expected
// to lie on the face). BRepGProp_Face::Normal accounts for the face
// orientation, so the result is the outward normal.
bool normalAtPoint(const TopoDS_Face& f, const gp_Pnt& p, gp_Dir& n) {
    try {
        Standard_Real u0, u1, v0, v1;
        BRepTools::UVBounds(f, u0, u1, v0, v1);
        GeomAPI_ProjectPointOnSurf proj(p, BRep_Tool::Surface(f), u0, u1, v0, v1);
        if (!proj.IsDone() || proj.NbPoints() < 1) return false;
        Standard_Real u, v;
        proj.LowerDistanceParameters(u, v);
        gp_Pnt p2;
        gp_Vec nv;
        BRepGProp_Face(f).Normal(u, v, p2, nv);
        if (nv.Magnitude() < 1e-9) return false;
        n = gp_Dir(nv);
        return true;
    } catch (...) {
        return false;
    }
}

// Largest vertex-to-shared-edge distance of `cand`; blend faces are narrow,
// so every vertex must sit within maxWidth of some shared edge.
bool narrowEnough(const TopoDS_Face& cand, const std::vector<TopoDS_Edge>& shared,
                  double maxWidth) {
    TopTools_IndexedMapOfShape vmap;
    TopExp::MapShapes(cand, TopAbs_VERTEX, vmap);
    for (Standard_Integer i = 1; i <= vmap.Extent(); ++i) {
        double best = Precision::Infinite();
        for (const TopoDS_Edge& e : shared) {
            try {
                BRepExtrema_DistShapeShape dss(vmap.FindKey(i), e);
                if (dss.IsDone()) best = std::min(best, dss.Value());
            } catch (...) {}
        }
        if (best > maxWidth) return false;
    }
    return true;
}

// Is `cand` also tangent (1 deg) to the body face on its opposite side? A
// fillet rolls from `ref` into some other face; a face that only touches
// `ref` smoothly (e.g. a tangent split) does not.
bool hasTangentOpposite(const TopoDS_Face& cand, const TopoDS_Face& ref,
                        const TopTools_IndexedDataMapOfShapeListOfShape& edgeFaceMap) {
    const double cos1 = std::cos(1.0 * kPi / 180.0);
    for (Standard_Integer i = 1; i <= edgeFaceMap.Extent(); ++i) {
        const TopTools_ListOfShape& fl = edgeFaceMap.FindFromIndex(i);
        bool hasC = false, hasRef = false;
        TopoDS_Face other;
        int others = 0;
        for (TopTools_ListIteratorOfListOfShape it(fl); it.More(); it.Next()) {
            if (it.Value().IsSame(cand)) hasC = true;
            else if (it.Value().IsSame(ref)) hasRef = true;
            else { other = TopoDS::Face(it.Value()); ++others; }
        }
        if (!hasC || hasRef || others != 1) continue;
        const TopoDS_Edge e = TopoDS::Edge(edgeFaceMap.FindKey(i));
        BRepAdaptor_Curve c(e);
        const Standard_Real t0 = c.FirstParameter(), t1 = c.LastParameter();
        for (int k = 0; k <= 5; ++k) {
            const Standard_Real t = t0 + (t1 - t0) * k / 5.0;
            const gp_Pnt p = c.Value(t);
            gp_Dir nC, nO;
            if (!normalAtPoint(cand, p, nC) || !normalAtPoint(other, p, nO)) break;
            if (nC.Dot(nO) < cos1) break;
            if (k == 5) return true;
        }
    }
    return false;
}

bool isBlend(const TopoDS_Face& cand, const TopoDS_Face& ref,
             const std::vector<TopoDS_Edge>& shared,
             const TopTools_IndexedDataMapOfShapeListOfShape& edgeFaceMap,
             double maxWidth) {
    if (shared.empty()) return false;
    if (!narrowEnough(cand, shared, maxWidth)) return false;

    // Dihedral sampled along the shared edges.
    double minDot = 1.0, maxDot = -1.0;
    bool sampled = false;
    for (const TopoDS_Edge& e : shared) {
        BRepAdaptor_Curve c(e);
        const Standard_Real t0 = c.FirstParameter(), t1 = c.LastParameter();
        for (int k = 0; k <= 5; ++k) {
            const Standard_Real t = t0 + (t1 - t0) * k / 5.0;
            const gp_Pnt p = c.Value(t);
            gp_Dir nRef, nCand;
            if (!normalAtPoint(ref, p, nRef) || !normalAtPoint(cand, p, nCand))
                return false;
            const double d = nRef.Dot(nCand);
            minDot = std::min(minDot, d);
            maxDot = std::max(maxDot, d);
            sampled = true;
        }
    }
    if (!sampled) return false;

    const bool planar = BRepAdaptor_Surface(cand).GetType() == GeomAbs_Plane;
    if (planar) {
        // Chamfer: clearly not tangent, dihedral between 5 and 85 deg.
        return maxDot < std::cos(5.0 * kPi / 180.0) &&
               minDot > std::cos(85.0 * kPi / 180.0);
    }
    // Fillet: tangent to `ref` along the whole shared edge...
    if (minDot < std::cos(1.0 * kPi / 180.0)) return false;
    // ... and tangent to the face it rolls into on the far side.
    return hasTangentOpposite(cand, ref, edgeFaceMap);
}

// Reduce `res` to a single solid: close a lone shell, or lift the first
// solid out of a compound.
bool toSolid(TopoDS_Shape& res, std::string& error) {
    if (res.ShapeType() == TopAbs_SOLID) return true;
    if (res.ShapeType() == TopAbs_SHELL) {
        BRepBuilderAPI_MakeSolid mk(TopoDS::Shell(res));
        if (!mk.IsDone()) {
            error = "offset result is not a solid";
            return false;
        }
        res = mk.Solid();
        return true;
    }
    if (res.ShapeType() == TopAbs_COMPOUND) {
        TopExp_Explorer ex(res, TopAbs_SOLID);
        if (ex.More()) {
            res = ex.Current();
            return true;
        }
        TopExp_Explorer sx(res, TopAbs_SHELL);
        if (sx.More()) {
            BRepBuilderAPI_MakeSolid mk(TopoDS::Shell(sx.Current()));
            if (mk.IsDone()) {
                res = mk.Solid();
                return true;
            }
        }
    }
    error = "offset result is not a solid";
    return false;
}

} // namespace

std::vector<TopoDS_Face> findAdjacentBlends(const TopoDS_Shape& body, const TopoDS_Face& face,
                                            double maxWidth) {
    std::vector<TopoDS_Face> out;
    if (body.IsNull() || face.IsNull() || maxWidth <= 0.0) return out;

    TopTools_IndexedDataMapOfShapeListOfShape edgeFaceMap;
    TopExp::MapShapesAndAncestors(body, TopAbs_EDGE, TopAbs_FACE, edgeFaceMap);

    // Candidate neighbours: other faces sharing an edge with `face`, unique,
    // in TopExp map order (deterministic). Never `face` itself.
    std::vector<TopoDS_Face> candidates;
    for (Standard_Integer i = 1; i <= edgeFaceMap.Extent(); ++i) {
        const TopTools_ListOfShape& fl = edgeFaceMap.FindFromIndex(i);
        bool hasRef = false;
        for (TopTools_ListIteratorOfListOfShape it(fl); it.More(); it.Next())
            if (it.Value().IsSame(face)) { hasRef = true; break; }
        if (!hasRef) continue;
        for (TopTools_ListIteratorOfListOfShape it(fl); it.More(); it.Next()) {
            TopoDS_Face c = TopoDS::Face(it.Value());
            if (c.IsSame(face)) continue;
            bool dup = false;
            for (const TopoDS_Face& e : candidates)
                if (c.IsSame(e)) { dup = true; break; }
            if (!dup) candidates.push_back(c);
        }
    }

    for (const TopoDS_Face& c : candidates) {
        std::vector<TopoDS_Edge> shared;
        for (Standard_Integer i = 1; i <= edgeFaceMap.Extent(); ++i) {
            bool hasRef = false, hasC = false;
            for (TopTools_ListIteratorOfListOfShape it(edgeFaceMap.FindFromIndex(i));
                 it.More(); it.Next()) {
                if (it.Value().IsSame(face)) hasRef = true;
                else if (it.Value().IsSame(c)) hasC = true;
            }
            if (hasRef && hasC)
                shared.push_back(TopoDS::Edge(edgeFaceMap.FindKey(i)));
        }
        if (isBlend(c, face, shared, edgeFaceMap, maxWidth)) out.push_back(c);
    }
    return out;
}

bool offsetFaces(const TopoDS_Shape& body, const std::vector<TopoDS_Face>& faces,
                 double distance, const Options& opt, Result& r) {
    r = Result();
    if (body.IsNull()) { r.error = "empty body"; return false; }
    if (faces.empty()) { r.error = "no faces to offset"; return false; }
    if (std::abs(distance) <= Precision::Confusion()) {
        r.error = "offset distance too small";
        return false;
    }

    try {
        TopoDS_Shape shape = body;
        std::vector<TopoDS_Face> sel = faces;

        if (opt.removeAdjacentBlends && distance > 0.0) {
            // Blends touching ANY selected face (no duplicates, never a
            // selected face itself) get removed before the offset, so the
            // neighbours extend and the moved face lands on sharp edges.
            std::vector<TopoDS_Face> blends;
            for (const TopoDS_Face& f : sel) {
                for (const TopoDS_Face& b : findAdjacentBlends(body, f, opt.maxBlendWidth)) {
                    bool isSel = false;
                    for (const TopoDS_Face& s : sel)
                        if (b.IsSame(s)) { isSel = true; break; }
                    if (isSel) continue;
                    bool dup = false;
                    for (const TopoDS_Face& e : blends)
                        if (b.IsSame(e)) { dup = true; break; }
                    if (!dup) blends.push_back(b);
                }
            }
            if (!blends.empty()) {
                BRepAlgoAPI_Defeaturing df;
                df.SetShape(body);
                TopTools_ListOfShape toRemove;
                for (const TopoDS_Face& b : blends) toRemove.Append(b);
                df.AddFacesToRemove(toRemove);
                df.SetRunParallel(Standard_False);
                df.Build();
                if (df.HasErrors() || df.Shape().IsNull()) {
                    r.error = "could not remove adjacent blends";
                    return false;
                }
                shape = df.Shape();
                r.removedBlends = blends;
                // Re-find the selected faces in the defeaturing result via
                // its history: a non-empty Modified list means replaced, an
                // undeleted face means unchanged.
                std::vector<TopoDS_Face> rebound;
                for (const TopoDS_Face& f : sel) {
                    const TopTools_ListOfShape& mod = df.Modified(f);
                    if (!mod.IsEmpty())
                        rebound.push_back(TopoDS::Face(mod.First()));
                    else if (!df.IsDeleted(f))
                        rebound.push_back(f);
                }
                if (rebound.empty()) {
                    r.error = "offset faces lost after blend removal";
                    return false;
                }
                sel = rebound;
            }
        }

        BRepOffset_MakeOffset mo;
        mo.Initialize(shape, 0.0, Precision::Confusion(), BRepOffset_Skin,
                      Standard_False, Standard_False, GeomAbs_Intersection,
                      Standard_False, Standard_False);
        for (const TopoDS_Face& f : sel) mo.SetOffsetOnFace(f, distance);
        mo.MakeOffsetShape();
        if (!mo.IsDone()) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "offset failed (code %d)",
                          static_cast<int>(mo.Error()));
            r.error = buf;
            return false;
        }
        TopoDS_Shape res = mo.Shape();
        if (res.IsNull()) {
            r.error = "offset failed (empty result)";
            return false;
        }
        if (!toSolid(res, r.error)) return false;
        if (!BRepCheck_Analyzer(res).IsValid()) {
            r.error = "offset result is invalid";
            return false;
        }
        r.shape = res;
        return true;
    } catch (const Standard_Failure& e) {
        r.error = e.GetMessageString() ? e.GetMessageString() : "OCCT failure";
        return false;
    } catch (...) {
        r.error = "unknown error";
        return false;
    }
}

} // namespace offsetface
} // namespace materializr
