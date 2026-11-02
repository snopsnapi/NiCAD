#include "SurfaceOps.h"

#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Splitter.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepGProp.hxx>
#include <BRepOffsetAPI_MakePipe.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <IntCurvesFace_ShapeIntersector.hxx>
#include <gp_Lin.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Shell.hxx>
#include <gp_Trsf.hxx>

namespace materializr {
namespace surf {

namespace {

// Faces that share edges become one shell each; sewing with a tight
// tolerance only re-links what already touches, it never closes a gap.
TopoDS_Shape facesToShells(const std::vector<TopoDS_Face>& faces) {
    if (faces.empty()) return TopoDS_Shape();
    if (faces.size() == 1) return faces.front();
    try {
        BRepBuilderAPI_Sewing sew(1e-6);
        for (const TopoDS_Face& f : faces) sew.Add(f);
        sew.Perform();
        TopoDS_Shape out = sew.SewedShape();
        if (!out.IsNull()) return out;
    } catch (...) {}
    BRep_Builder b;
    TopoDS_Compound c;
    b.MakeCompound(c);
    for (const TopoDS_Face& f : faces) b.Add(c, f);
    return c;
}

TopoDS_Shape compoundOf(const std::vector<TopoDS_Shape>& parts) {
    if (parts.empty()) return TopoDS_Shape();
    if (parts.size() == 1) return parts.front();
    BRep_Builder b;
    TopoDS_Compound c;
    b.MakeCompound(c);
    for (const TopoDS_Shape& s : parts) b.Add(c, s);
    return c;
}

} // namespace

TopoDS_Shape removeFacesOpen(const TopoDS_Shape& body,
                             const std::vector<TopoDS_Face>& remove) {
    if (body.IsNull() || remove.empty()) return TopoDS_Shape();
    TopTools_IndexedMapOfShape drop;
    for (const TopoDS_Face& f : remove) drop.Add(f);
    std::vector<TopoDS_Face> keep;
    bool removedAny = false;
    TopTools_IndexedMapOfShape seen;
    for (TopExp_Explorer ex(body, TopAbs_FACE); ex.More(); ex.Next()) {
        const TopoDS_Face f = TopoDS::Face(ex.Current());
        if (seen.Contains(f)) continue;
        seen.Add(f);
        bool hit = false;
        for (int i = 1; i <= drop.Extent(); ++i)
            if (drop(i).IsSame(f)) { hit = true; break; }
        if (hit) { removedAny = true; continue; }
        keep.push_back(f);
    }
    if (!removedAny || keep.empty()) return TopoDS_Shape();
    return facesToShells(keep);
}

TopoDS_Shape extrudeCurves(const std::vector<TopoDS_Wire>& wires, const gp_Vec& v) {
    if (wires.empty() || v.Magnitude() < 1e-9) return TopoDS_Shape();
    std::vector<TopoDS_Shape> parts;
    try {
        for (const TopoDS_Wire& w : wires) {
            if (w.IsNull()) continue;
            BRepPrimAPI_MakePrism prism(w, v, /*Copy=*/Standard_True);
            if (!prism.IsDone()) return TopoDS_Shape();
            parts.push_back(prism.Shape());
        }
    } catch (...) {
        return TopoDS_Shape();
    }
    return compoundOf(parts);
}

TopoDS_Shape sweep(const TopoDS_Shape& profile, const TopoDS_Wire& path) {
    if (profile.IsNull() || path.IsNull()) return TopoDS_Shape();
    try {
        BRepOffsetAPI_MakePipe pipe(path, profile);
        pipe.Build();
        if (!pipe.IsDone()) return TopoDS_Shape();
        return pipe.Shape();
    } catch (...) {
        return TopoDS_Shape();
    }
}

TopoDS_Shape curveCutter(const std::vector<TopoDS_Wire>& wires,
                         const gp_Vec& normal, double extent) {
    if (normal.Magnitude() < 1e-12 || extent <= 0.0) return TopoDS_Shape();
    const gp_Vec n = normal.Normalized();
    std::vector<TopoDS_Wire> shifted;
    try {
        gp_Trsf t;
        t.SetTranslation(n * -extent);
        for (const TopoDS_Wire& w : wires) {
            if (w.IsNull()) continue;
            BRepBuilderAPI_Transform tr(w, t, /*Copy=*/Standard_True);
            shifted.push_back(TopoDS::Wire(tr.Shape()));
        }
    } catch (...) {
        return TopoDS_Shape();
    }
    return extrudeCurves(shifted, n * (2.0 * extent));
}

TopoDS_Shape splitByTool(const TopoDS_Shape& target, const TopoDS_Shape& tool) {
    if (target.IsNull() || tool.IsNull()) return TopoDS_Shape();
    try {
        BRepAlgoAPI_Splitter sp;
        TopTools_ListOfShape args, tools;
        args.Append(target);
        tools.Append(tool);
        sp.SetArguments(args);
        sp.SetTools(tools);
        sp.Build();
        if (sp.HasErrors()) return TopoDS_Shape();
        return sp.Shape();
    } catch (...) {
        return TopoDS_Shape();
    }
}

bool pickPoint(const TopoDS_Shape& shape, const gp_Pnt& origin, const gp_Dir& dir,
               gp_Pnt& hit) {
    if (shape.IsNull()) return false;
    try {
        IntCurvesFace_ShapeIntersector inter;
        inter.Load(shape, 1e-6);
        inter.Perform(gp_Lin(origin, dir), 0.0, 1e12);
        if (!inter.IsDone() || inter.NbPnt() < 1) return false;
        int best = 1;
        for (int i = 2; i <= inter.NbPnt(); ++i)
            if (inter.WParameter(i) < inter.WParameter(best)) best = i;
        hit = inter.Pnt(best);
        return true;
    } catch (...) {
        return false;
    }
}

TopoDS_Face nearestFace(const TopoDS_Shape& shape, const gp_Pnt& p) {
    TopoDS_Face best;
    double bestD = 1e300;
    try {
        const TopoDS_Shape v = BRepBuilderAPI_MakeVertex(p).Shape();
        for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
            BRepExtrema_DistShapeShape d(v, ex.Current());
            if (!d.IsDone()) continue;
            if (d.Value() < bestD) {
                bestD = d.Value();
                best = TopoDS::Face(ex.Current());
            }
        }
    } catch (...) {}
    return best;
}

gp_Pnt facePoint(const TopoDS_Face& face) {
    try {
        double u1, u2, v1, v2;
        BRepTools::UVBounds(face, u1, u2, v1, v2);
        BRepAdaptor_Surface s(face);
        const gp_Pnt2d mid((u1 + u2) * 0.5, (v1 + v2) * 0.5);
        BRepClass_FaceClassifier cls(face, mid, 1e-7);
        if (cls.State() == TopAbs_IN) return s.Value(mid.X(), mid.Y());
        // Centre in a hole (or outside a non-convex trim): fall back to the
        // area centroid, which lies on the surface for the usual pieces.
        GProp_GProps g;
        BRepGProp::SurfaceProperties(face, g);
        return g.CentreOfMass();
    } catch (...) {
        return gp_Pnt();
    }
}

TopoDS_Shape trim(const TopoDS_Shape& target, const TopoDS_Shape& tool,
                  const std::vector<gp_Pnt>& removeAt) {
    const TopoDS_Shape split = splitByTool(target, tool);
    if (split.IsNull()) return TopoDS_Shape();
    if (removeAt.empty()) return split;
    std::vector<TopoDS_Face> drop;
    for (const gp_Pnt& p : removeAt) {
        const TopoDS_Face f = nearestFace(split, p);
        if (!f.IsNull()) drop.push_back(f);
    }
    return removeFacesOpen(split, drop);
}

} // namespace surf
} // namespace materializr
