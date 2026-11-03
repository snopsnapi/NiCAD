#pragma once
// How every mesher call in Materializr asks OCCT to triangulate a shape.
//
// The one setting that matters is the algorithm. OCCT's default (Watson)
// slows down sharply on a planar face with many curved inner wires, which is
// exactly what hole patterns and SVG outlines produce: one face of a 400-hole
// plate took 1028 ms, the whole body 1095 ms, on every edit and every load.
// Delabella (in OCCT since 7.6; every platform here builds 7.9.3) meshes that
// face in 47 ms and the body in 84 ms, was never slower on spheres, tori,
// fillets or slotted plates, and gives the same triangle counts with equal or
// smaller achieved deflection.
//
// BRepMesh_IncrementalMesh's shape constructors already call Perform(); a
// second Perform() is a full extra pass over the model (25 ms on that plate),
// so callers must not add one.
#include <IMeshTools_Parameters.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <Adaptor3d_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAdaptor_Curve2d.hxx>
#include <gp_Pnt2d.hxx>
#include <set>
#include <map>
#include <vector>
#include <cmath>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <GCPnts_TangentialDeflection.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <Poly_Triangle.hxx>
#include <Poly_PolygonOnTriangulation.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <cmath>
#include <Geom_BSplineCurve.hxx>
#include <Geom_Curve.hxx>
#include <TopoDS_Edge.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Message_ProgressScope.hxx>
#include <Standard_Type.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom_Surface.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <algorithm>
#include <chrono>
#include <limits>
#include <cstdio>
#include <utility>

namespace materializr {

inline IMeshTools_Parameters meshParams(double deflection, double angularDeflection,
                                        bool inParallel)
{
    IMeshTools_Parameters p;
    p.Deflection = deflection;
    p.Angle = angularDeflection;
    p.Relative = false;
    p.InParallel = inParallel;
    p.MeshAlgo = IMeshTools_MeshAlgoType_Delabella;
    return p;
}


// Aborts a mesher run once its time budget is spent. OCCT polls UserBreak()
// all through BRepMesh, so this lands within milliseconds.
class MeshDeadline : public Message_ProgressIndicator {
public:
    DEFINE_STANDARD_RTTI_INLINE(MeshDeadline, Message_ProgressIndicator)
    explicit MeshDeadline(double seconds)
        : m_end(std::chrono::steady_clock::now() +
                std::chrono::milliseconds(static_cast<long long>(seconds * 1000.0))) {}
    Standard_Boolean UserBreak() override {
        return std::chrono::steady_clock::now() > m_end;
    }
protected:
    void Show(const Message_ProgressScope&, const Standard_Boolean) override {}
private:
    std::chrono::steady_clock::time_point m_end;
};

// Triangulate a plain swept wall directly as a strip along its directrix.
//
// A prism wall (SurfaceOfLinearExtrusion) is ruled: S(u,v) = C(u) + v*dir, so a
// strip of quads between C(u_k)+v0*dir and C(u_k)+v1*dir is the exact surface,
// and the only thing that sets its quality is how finely the directrix C is
// sampled. Delabella declines these faces when the directrix is a traced
// outline, and Watson then spends 25 s+ on one at Ultra quality (0.01 mm /
// 0.10 rad); this is O(n) and instant. Only a FULL wall qualifies - all four
// edges must be iso-lines in the face's UV domain (two along u, two along v) -
// so a wall trimmed by a boolean falls through to the real meshers.
inline bool meshExtrusionWallTrimmed(const TopoDS_Face& f, double deflection,
                                     double angularDeflection);

// A wall swept straight along one direction: S(u, v) = C(u) + (v - vRef) * perV.
// A prism wall is one (SurfaceOfLinearExtrusion). So is a B-spline that is
// degree 1 with two pole rows in v, where the second row is the first moved by
// one fixed vector - which is what a traced outline's wall becomes once a
// boolean or a union has rebuilt it (autumn.mzr: a degree-8, 3243-pole wall,
// rows exact translates). The type check used to accept only the prism kind,
// so those walls went to Delabella, which meshed one around a drilled hole as
// a fan of 16 mm chords across the curve - a 0.4 mm flat dent in the STL.
struct RuledWall {
    Handle(Adaptor3d_Curve) base;   // C(u), in the triangulation's (face-local) frame
    gp_Vec perV;
    double vRef = 0.0;
    gp_Pnt at(const gp_Pnt& c, double v) const { return c.Translated(perV * (v - vRef)); }
};

inline bool ruledWallOf(const TopoDS_Face& f, RuledWall& w) {
    BRepAdaptor_Surface s(f, Standard_False);
    if (s.GetType() == GeomAbs_SurfaceOfExtrusion) {
        w.base = s.BasisCurve();
        w.perV = gp_Vec(s.Direction());
        w.vRef = 0.0;
        return !w.base.IsNull();
    }
    if (s.GetType() != GeomAbs_BSplineSurface) return false;
    TopLoc_Location loc;
    Handle(Geom_Surface) gs = BRep_Tool::Surface(f, loc);
    while (Handle(Geom_RectangularTrimmedSurface) t = Handle(Geom_RectangularTrimmedSurface)::DownCast(gs))
        gs = t->BasisSurface();
    Handle(Geom_BSplineSurface) bs = Handle(Geom_BSplineSurface)::DownCast(gs);
    if (bs.IsNull() || bs->VDegree() != 1 || bs->NbVPoles() != 2 || bs->IsVPeriodic())
        return false;
    const gp_Vec d(bs->Pole(1, 1), bs->Pole(1, 2));
    const double len = d.Magnitude();
    if (len < 1e-9) return false;
    const bool rational = bs->IsURational() || bs->IsVRational();
    for (int i = 1; i <= bs->NbUPoles(); ++i) {
        if ((gp_Vec(bs->Pole(i, 1), bs->Pole(i, 2)) - d).Magnitude() > 1e-7 * len + 1e-9)
            return false;
        // A weight that differs between the two rows bends the ruling.
        if (rational && std::abs(bs->Weight(i, 1) - bs->Weight(i, 2)) > 1e-12 * bs->Weight(i, 1))
            return false;
    }
    const double v0 = bs->VKnot(1), v1 = bs->VKnot(bs->NbVKnots());
    if (!(v1 > v0)) return false;
    w.base = new GeomAdaptor_Curve(bs->VIso(v0));
    w.perV = d / (v1 - v0);
    w.vRef = v0;
    return true;
}

inline bool meshExtrusionWallStrip(const TopoDS_Face& f, double deflection,
                                   double angularDeflection,
                                   const std::vector<double>* uSamples = nullptr) {
    try {
        RuledWall w;
        if (!ruledWallOf(f, w)) return false;
        int alongU = 0, alongV = 0;
        for (TopExp_Explorer ex(f, TopAbs_EDGE); ex.More(); ex.Next()) {
            double a = 0.0, b = 0.0;
            Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(TopoDS::Edge(ex.Current()), f, a, b);
            if (pc.IsNull()) return false;
            if (Handle(Geom2d_TrimmedCurve) t = Handle(Geom2d_TrimmedCurve)::DownCast(pc))
                pc = t->BasisCurve();
            Handle(Geom2d_Line) ln = Handle(Geom2d_Line)::DownCast(pc);
            // Not a plain full wall (a boolean trimmed it, or its boundary
            // pcurves are B-splines): see whether it is still a rectilinear
            // region in (u, v) before giving up to the slow meshers.
            if (ln.IsNull()) return uSamples ? false : meshExtrusionWallTrimmed(f, deflection, angularDeflection);
            const gp_Dir2d d = ln->Direction();
            if (std::abs(d.Y()) < 1e-9) ++alongU;
            else if (std::abs(d.X()) < 1e-9) ++alongV;
            else return uSamples ? false : meshExtrusionWallTrimmed(f, deflection, angularDeflection);
        }
        if (alongU != 2 || alongV != 2)
            return uSamples ? false : meshExtrusionWallTrimmed(f, deflection, angularDeflection);

        double u0, u1, v0, v1;
        BRepTools::UVBounds(f, u0, u1, v0, v1);
        if (!(v1 > v0) || !(u1 > u0)) return false;

        // Directrix samples: its own deflection sampling, or (uSamples) the
        // parameters BRepMesh gave the shared edges, so the strip's boundary
        // vertices coincide with the neighbouring faces' instead of cracking.
        std::vector<double> us;
        if (uSamples) us = *uSamples;
        else {
            GCPnts_TangentialDeflection pts(*w.base, u0, u1, angularDeflection, deflection, 2);
            for (int k = 1; k <= pts.NbPoints(); ++k) us.push_back(pts.Parameter(k));
        }
        const int n = static_cast<int>(us.size());
        if (n < 2) return false;

        // UV nodes too, so meshSag can check this mesh like any other.
        Handle(Poly_Triangulation) tri = new Poly_Triangulation(2 * n, 2 * (n - 1), Standard_True);
        for (int k = 0; k < n; ++k) {
            const double u = us[k];
            const gp_Pnt c = w.base->Value(u);
            tri->SetNode(k + 1, w.at(c, v0));
            tri->SetNode(n + k + 1, w.at(c, v1));
            tri->SetUVNode(k + 1, gp_Pnt2d(u, v0));
            tri->SetUVNode(n + k + 1, gp_Pnt2d(u, v1));
        }
        for (int k = 1; k < n; ++k) {       // natural surface orientation (du x dv)
            tri->SetTriangle(2 * k - 1, Poly_Triangle(k, k + 1, n + k + 1));
            tri->SetTriangle(2 * k, Poly_Triangle(k, n + k + 1, n + k));
        }
        tri->Deflection(deflection);
        BRep_Builder().UpdateFace(f, tri);
        return true;
    } catch (...) {
        return false;
    }
}

// A swept wall that a boolean has TRIMMED, still meshed as ruled strips.
//
// meshExtrusionWallStrip only takes a wall whose four edges are iso-lines. Fuse
// a second body through a traced-outline wall and the wall keeps its ruling
// S(u,v) = C(u) + v*dir but its boundary grows to many edges, several of them
// B-spline pcurves and some oblique in (u, v) (autumn.mzr: 16 edges, 10 of
// them B-spline, 4 slanted where the other body's curved wall meets this one).
// That sent it to Watson: 1.7 s at Medium, 11 s + 9 s of escalation at Ultra,
// which is why a union made the lettering vanish for about a minute while the
// background mesh ground away.
//
// Cut the face into thin columns along u. Within a column every boundary edge
// is a single-valued v(u), so the wall is just the stretches between
// consecutive boundary curves (even-odd, as in any trapezoidal decomposition),
// and each stretch is a quad strip - ruled, so exact. Boundary pcurves are
// sampled and split into runs that are monotone in u; constant-u pieces (the
// cuts parallel to dir) only contribute a column break. Column breaks also
// include every sample of every run and the directrix's own deflection
// sampling, so v is linear per column and the outline stays as fine as it
// would have been untrimmed. A face whose columns do not pair up (an odd
// count: a shape this does not understand) is declined and falls through to
// the real meshers exactly as before.
inline bool meshExtrusionWallTrimmed(const TopoDS_Face& f, double deflection,
                                     double angularDeflection) {
    try {
        RuledWall w;
        if (!ruledWallOf(f, w)) return false;
        double u0, u1, v0, v1;
        BRepTools::UVBounds(f, u0, u1, v0, v1);
        if (!(v1 > v0) || !(u1 > u0)) return false;
        const double du = u1 - u0;
        const double vertical = 1e-7 * du;   // |du| below this: a cut parallel to dir

        struct Run { std::vector<double> u, v; };       // u strictly increasing
        std::vector<Run> runs;
        std::set<double> breaks;
        breaks.insert(u0); breaks.insert(u1);

        for (TopExp_Explorer ex(f, TopAbs_EDGE); ex.More(); ex.Next()) {
            BRepAdaptor_Curve2d c(TopoDS::Edge(ex.Current()), f);
            const double a = c.FirstParameter(), b = c.LastParameter();
            int n = 24;
            if (c.GetType() == GeomAbs_BSplineCurve)
                n = std::max(24, std::min(512, 8 * c.NbKnots()));
            else if (c.GetType() == GeomAbs_Line)
                n = 2;
            std::vector<gp_Pnt2d> p(n + 1);
            for (int k = 0; k <= n; ++k) p[k] = c.Value(a + (b - a) * k / double(n));

            // Split into runs that are monotone in u and non-vertical.
            size_t i = 0;
            while (i < p.size() - 1) {
                const double d0 = p[i + 1].X() - p[i].X();
                if (std::abs(d0) <= vertical) { breaks.insert(p[i].X()); breaks.insert(p[i + 1].X()); ++i; continue; }
                const int dirSign = d0 > 0 ? 1 : -1;
                Run r;
                r.u.push_back(p[i].X()); r.v.push_back(p[i].Y());
                size_t j = i;
                while (j < p.size() - 1) {
                    const double dj = p[j + 1].X() - p[j].X();
                    if (std::abs(dj) <= vertical || (dj > 0 ? 1 : -1) != dirSign) break;
                    r.u.push_back(p[j + 1].X()); r.v.push_back(p[j + 1].Y());
                    ++j;
                }
                if (dirSign < 0) { std::reverse(r.u.begin(), r.u.end()); std::reverse(r.v.begin(), r.v.end()); }
                for (double x : r.u) breaks.insert(x);
                runs.push_back(std::move(r));
                i = j;
            }
        }
        if (runs.empty()) return false;

        GCPnts_TangentialDeflection pts(*w.base, u0, u1, angularDeflection, deflection, 2);
        for (int k = 1; k <= pts.NbPoints(); ++k) breaks.insert(pts.Parameter(k));

        std::vector<double> U;
        for (double x : breaks)
            if (x >= u0 - vertical && x <= u1 + vertical && (U.empty() || x - U.back() > vertical))
                U.push_back(x);
        if (U.size() < 2) return false;

        // v of a run at u (u inside the run's span), by linear interpolation.
        auto vAt = [](const Run& r, double u) {
            size_t k = std::upper_bound(r.u.begin(), r.u.end(), u) - r.u.begin();
            if (k == 0) return r.v.front();
            if (k >= r.u.size()) return r.v.back();
            const double t = (u - r.u[k - 1]) / (r.u[k] - r.u[k - 1]);
            return r.v[k - 1] + t * (r.v[k] - r.v[k - 1]);
        };

        std::vector<gp_Pnt> colPt(U.size());
        for (size_t i = 0; i < U.size(); ++i) colPt[i] = w.base->Value(U[i]);

        std::vector<gp_Pnt> nodes;
        std::vector<gp_Pnt2d> uvs;
        std::map<std::pair<size_t, long long>, int> nodeOf;   // (column, v) -> node, shared
        auto node = [&](size_t i, double v) {
            auto key = std::make_pair(i, std::llround(v * 1e6));
            auto it = nodeOf.find(key);
            if (it != nodeOf.end()) return it->second;
            nodes.push_back(w.at(colPt[i], v));
            uvs.emplace_back(U[i], v);
            return nodeOf[key] = int(nodes.size());
        };
        std::vector<Poly_Triangle> tris;

        std::vector<std::pair<double, const Run*>> hit;
        for (size_t i = 0; i + 1 < U.size(); ++i) {
            const double um = 0.5 * (U[i] + U[i + 1]);
            hit.clear();
            for (const Run& r : runs)
                if (r.u.front() <= um && um <= r.u.back()) hit.push_back({vAt(r, um), &r});
            if (hit.empty()) continue;                    // wholly outside the face
            if (hit.size() % 2) return false;             // does not pair up: not understood
            std::sort(hit.begin(), hit.end(),
                      [](const auto& x, const auto& y) { return x.first < y.first; });
            for (size_t k = 0; k + 1 < hit.size(); k += 2) {
                const Run& lo = *hit[k].second;
                const Run& hi = *hit[k + 1].second;
                const double lo0 = vAt(lo, U[i]), lo1 = vAt(lo, U[i + 1]);
                const double hi0 = vAt(hi, U[i]), hi1 = vAt(hi, U[i + 1]);
                if (hi0 - lo0 < 1e-9 && hi1 - lo1 < 1e-9) continue;   // sliver of zero height
                const int a = node(i, lo0), b = node(i + 1, lo1), c = node(i + 1, hi1), d = node(i, hi0);
                tris.emplace_back(a, b, c);               // natural surface orientation (du x dv)
                tris.emplace_back(a, c, d);
            }
        }
        if (tris.empty()) return false;

        Handle(Poly_Triangulation) tri =
            new Poly_Triangulation(int(nodes.size()), int(tris.size()), Standard_True);
        for (size_t k = 0; k < nodes.size(); ++k) {
            tri->SetNode(int(k) + 1, nodes[k]);
            tri->SetUVNode(int(k) + 1, uvs[k]);
        }
        for (size_t k = 0; k < tris.size(); ++k) tri->SetTriangle(int(k) + 1, tris[k]);
        tri->Deflection(deflection);
        BRep_Builder().UpdateFace(f, tri);
        return true;
    } catch (...) {
        return false;
    }
}

// Watson-mesh one bare face, giving up after `seconds`.
//
// Surface-deflection control is OFF here. On the swept wall of a traced
// 450-point outline (an extrusion surface over a 600-pole directrix) it made
// Watson insert 92,806 nodes and take 6.5 s; without it the same wall meshes
// in 0.8 s with 14,820 nodes (0.2 s at angle 1.0). A wall like that is ruled in
// the sweep direction, so the check buys nothing, and this path only runs for
// faces Delabella already failed on. (~100 s on the UI thread before the time
// box existed.) A timed-out attempt is discarded so the caller can escalate.
inline bool meshFaceWatsonBounded(const TopoDS_Face& f, double deflection,
                                  double angularDeflection, double seconds) {
    if (meshExtrusionWallStrip(f, deflection, angularDeflection)) return true;
    TopLoc_Location loc;
    try {
        IMeshTools_Parameters wp = meshParams(deflection, angularDeflection, false);
        wp.MeshAlgo = IMeshTools_MeshAlgoType_Watson;
        wp.ControlSurfaceDeflection = false;
        opencascade::handle<MeshDeadline> pi = new MeshDeadline(seconds);
        BRepMesh_IncrementalMesh retry(f, wp, pi->Start());
        if (pi->UserBreak()) {                 // budget spent: drop any partial mesh
            BRepTools::Clean(f);
            return false;
        }
    } catch (...) {
        return false;
    }
    return !BRep_Tool::Triangulation(f, loc).IsNull();
}

constexpr double kBareFaceBudgetSec = 3.0;

// Cheap guess at how slow a shape will be to mesh, for a body that has no
// timing history yet (a fresh extrude). Total B-spline poles over its edges:
// a traced 450-point outline carries thousands, an ordinary part a few dozen.
// Walks the edges only - no geometry is evaluated.
inline int meshComplexityHint(const TopoDS_Shape& shape) {
    int poles = 0;
    try {
        for (TopExp_Explorer ex(shape, TopAbs_EDGE); ex.More(); ex.Next()) {
            double f = 0.0, l = 0.0;
            Handle(Geom_Curve) c = BRep_Tool::Curve(TopoDS::Edge(ex.Current()), f, l);
            if (c.IsNull()) continue;
            if (Handle(Geom_BSplineCurve) b = Handle(Geom_BSplineCurve)::DownCast(c))
                poles += b->NbPoles();
        }
    } catch (...) {}
    return poles;
}
// Above this a body is meshed off-thread even with no old mesh to keep on
// screen: it shows up when the worker lands rather than freezing the UI.
constexpr int kHeavyPoleHint = 400;

// A face where even the Watson retry above fails - real and reproducible on
// an entirely ordinary planar face (10-edge boundary, no degenerate edges),
// not just a theoretical edge case: confirmed mesh-failing under BOTH
// Delabella and Watson at a project's "High"/"Ultra" quality settings
// (0.03/0.15 and 0.01/0.10) while succeeding fine at "Medium" (0.10/0.30)
// and coarser. Whatever OCCT's samplers dislike about that fine a spacing on
// this face, backing off clears it - recovered at 2x deflection for the
// High-quality case above, 4x for Ultra. A locally coarser single face is a
// far smaller defect than a hole clean through the model, so escalate
// through a few factors (capping angular deflection around 34 degrees,
// where facets stop looking meaningfully different) before giving up.
inline bool meshBareFaceEscalating(const TopoDS_Face& f, double deflection,
                                   double angularDeflection) {
    TopLoc_Location loc;
    for (double factor : {2.0, 4.0, 8.0, 16.0, 32.0}) {
        meshFaceWatsonBounded(f, deflection * factor,
                              std::min(angularDeflection * factor, 0.6),
                              kBareFaceBudgetSec);
        if (!BRep_Tool::Triangulation(f, loc).IsNull()) return true;
    }
    return false;
}

// How far a face's mesh strays from its surface, in mm: the worst gap between a
// triangle's centroid and the surface point at that triangle's UV centroid,
// over the `cap` LONGEST triangles (sag grows with size, so a bad region is
// always among them - a fan of chords is long by definition). Planes are 0 by
// construction, as is a mesh without UV nodes (nothing to compare against).
inline double meshSag(const TopoDS_Face& f, const Handle(Poly_Triangulation)& t,
                      int cap = 2000) {
    if (t.IsNull() || !t->HasUVNodes() || t->NbTriangles() == 0) return 0.0;
    try {
        TopLoc_Location sl;
        Handle(Geom_Surface) gs = BRep_Tool::Surface(f, sl);
        if (gs.IsNull()) return 0.0;
        Handle(Geom_Surface) basis = gs;
        while (Handle(Geom_RectangularTrimmedSurface) tr = Handle(Geom_RectangularTrimmedSurface)::DownCast(basis))
            basis = tr->BasisSurface();
        if (!Handle(Geom_Plane)::DownCast(basis).IsNull()) return 0.0;
        TopLoc_Location tl;
        BRep_Tool::Triangulation(f, tl);
        const bool sameFrame = tl.IsEqual(sl);

        const int n = t->NbTriangles();
        std::vector<std::pair<double, int>> bySize(n);
        for (int k = 1; k <= n; ++k) {
            int a, b, c; t->Triangle(k).Get(a, b, c);
            const gp_Pnt A = t->Node(a), B = t->Node(b), C = t->Node(c);
            bySize[k - 1] = {std::max({A.SquareDistance(B), B.SquareDistance(C), C.SquareDistance(A)}), k};
        }
        if (n > cap) {
            std::nth_element(bySize.begin(), bySize.begin() + cap, bySize.end(),
                             [](const auto& x, const auto& y) { return x.first > y.first; });
            bySize.resize(cap);
        }
        double worst = 0.0;
        for (const auto& [sz, k] : bySize) {
            (void)sz;
            int a, b, c; t->Triangle(k).Get(a, b, c);
            gp_XYZ m = (t->Node(a).XYZ() + t->Node(b).XYZ() + t->Node(c).XYZ()) / 3.0;
            const gp_XY uv = (t->UVNode(a).XY() + t->UVNode(b).XY() + t->UVNode(c).XY()) / 3.0;
            gp_Pnt onSurf = gs->Value(uv.X(), uv.Y());
            gp_Pnt onMesh(m);
            if (!sameFrame) {
                onSurf.Transform(sl.Transformation());
                onMesh.Transform(tl.Transformation());
            }
            worst = std::max(worst, onSurf.Distance(onMesh));
        }
        return worst;
    } catch (...) {
        return 0.0;
    }
}

// Re-mesh any face whose mesh strays far from its surface - a mesher that
// "succeeded" but fanned long chords across a curve (Delabella does this: it
// adds no interior points, so nothing makes it follow the curvature between
// distant boundary nodes). Bare faces are the fallback loop's job, not this.
// A ruled wall goes to the exact strip mesher; anything else gets a bounded
// Watson pass with surface-deflection control on. Whichever mesh is truer
// wins, so this never leaves a face worse than it found it. Returns how many
// faces it improved.
inline int repairInaccurateFaces(const TopoDS_Shape& shape, double deflection,
                                 double angularDeflection) {
    // Generous on purpose: a legitimate mesh can sit a little over the chord
    // deflection at a centroid (cones and offset faces at 1.05x were tripping a
    // 4x limit and paying for a Watson pass each), and this exists for the
    // gross misses - the autumn wall fan was 3.9 mm off at 0.01.
    const double tol = std::max(10.0 * deflection, 0.05);
    int improved = 0;
    for (TopExp_Explorer fx(shape, TopAbs_FACE); fx.More(); fx.Next()) {
        const TopoDS_Face& f = TopoDS::Face(fx.Current());
        TopLoc_Location loc;
        const Handle(Poly_Triangulation) old = BRep_Tool::Triangulation(f, loc);
        if (old.IsNull()) continue;
        const double sag = meshSag(f, old);
        if (sag <= tol) continue;

        double best = sag;
        const char* how = "kept";
        if (meshExtrusionWallStrip(f, deflection, angularDeflection)) {
            const double s2 = meshSag(f, BRep_Tool::Triangulation(f, loc));
            if (s2 < best) { best = s2; how = "ruled strip"; }
            else BRep_Builder().UpdateFace(f, old);
        }
        if (best > tol) {
            const Handle(Poly_Triangulation) before = BRep_Tool::Triangulation(f, loc);
            try {
                IMeshTools_Parameters wp = meshParams(deflection, angularDeflection, false);
                wp.MeshAlgo = IMeshTools_MeshAlgoType_Watson;
                wp.ControlSurfaceDeflection = true;
                BRepTools::Clean(f);
                opencascade::handle<MeshDeadline> pi = new MeshDeadline(kBareFaceBudgetSec);
                BRepMesh_IncrementalMesh retry(f, wp, pi->Start());
                if (pi->UserBreak()) BRepTools::Clean(f);
            } catch (...) {
                BRepTools::Clean(f);
            }
            const Handle(Poly_Triangulation) w = BRep_Tool::Triangulation(f, loc);
            const double s3 = w.IsNull() ? std::numeric_limits<double>::infinity() : meshSag(f, w);
            if (s3 < best) { best = s3; how = "Watson"; }
            else BRep_Builder().UpdateFace(f, before);
        }
        if (best < sag) ++improved;
        // One line: the units audit classifies line by line, and a wrapped
        // format string's second half reads as a user-facing "mm" readout.
        std::fprintf(stderr, "[Mesh] %s face strayed %.3f mm from its surface (limit %.3f): %s -> %.3f mm\n",
                     BRep_Tool::Surface(f)->DynamicType()->Name(), sag, tol, how, best);
    }
    return improved;
}

// Delabella (meshParams()'s algorithm, chosen for speed on many-holed faces)
// can leave a face with zero triangles on otherwise fully BRepCheck-valid,
// closed geometry - confirmed on real boolean-result bodies (issue #117).
// Mesh with it, then retry any bare face with Watson (OCCT's default, slower
// but far more robust) so a real hole never reaches the screen. Every caller
// that meshes a shape for DISPLAY (as opposed to STL/OBJ/glTF export, which
// tolerate a slower, non-Delabella pass across the board) should go through
// this instead of constructing BRepMesh_IncrementalMesh directly - a plain
// Delabella call has already had to be patched into this fallback more than
// once (the main render path, and again for the async worker a live
// interactive op's result gets pre-meshed on) precisely because it is easy to
// add a new meshing call site without remembering the fallback.
// Mesh every curved ruled wall with the exact strip mesher BEFORE the general
// pass, which keeps a face whose triangulation already meets the requested
// deflection. Delabella spent 82 s (in parallel) on autumn.mzr's 3243-pole
// lettering wall at export quality, and fanned it wrong; the strip is exact
// and takes milliseconds. Straight-sided ruled faces (a flat quad patch) are
// left to Delabella - nothing to gain there. Returns how many faces it took.
inline int premeshRuledWalls(const TopoDS_Shape& shape, double deflection,
                             double angularDeflection) {
    int n = 0;
    for (TopExp_Explorer fx(shape, TopAbs_FACE); fx.More(); fx.Next()) {
        const TopoDS_Face& f = TopoDS::Face(fx.Current());
        RuledWall w;
        if (!ruledWallOf(f, w)) continue;
        const GeomAbs_CurveType ct = w.base->GetType();
        if (ct == GeomAbs_Line ||
            (ct == GeomAbs_BSplineCurve && w.base->Degree() == 1 && w.base->NbPoles() == 2))
            continue;
        if (meshExtrusionWallStrip(f, deflection, angularDeflection)) ++n;
    }
    return n;
}

// After BRepMesh, re-strip every full ruled wall on the sample parameters that
// BRepMesh gave its directrix edges (read off the neighbouring faces), so the
// wall's top and bottom rims share vertices with the caps. Premeshing sampled
// the directrix independently (115 points where the caps' edge had 221), and a
// spline extrude exported with every one of its 668 edges open. Walls whose
// two rims were discretized differently, or that have no meshed neighbour, are
// left as premeshed.
inline int alignRuledWallsToEdges(const TopoDS_Shape& shape, double deflection,
                                  double angularDeflection) {
    TopTools_IndexedDataMapOfShapeListOfShape edgeFaces;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeFaces);
    int redone = 0;
    for (TopExp_Explorer fx(shape, TopAbs_FACE); fx.More(); fx.Next()) {
        const TopoDS_Face f = TopoDS::Face(fx.Current());
        RuledWall w;
        if (!ruledWallOf(f, w)) continue;
        TopLoc_Location fl;
        if (BRep_Tool::Triangulation(f, fl).IsNull()) continue;
        std::vector<std::vector<double>> rims;
        for (TopExp_Explorer ex(f, TopAbs_EDGE); ex.More(); ex.Next()) {
            const TopoDS_Edge e = TopoDS::Edge(ex.Current());
            double a = 0.0, b = 0.0;
            Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(e, f, a, b);
            if (pc.IsNull()) continue;
            if (Handle(Geom2d_TrimmedCurve) t = Handle(Geom2d_TrimmedCurve)::DownCast(pc))
                pc = t->BasisCurve();
            Handle(Geom2d_Line) ln = Handle(Geom2d_Line)::DownCast(pc);
            if (ln.IsNull() || std::abs(ln->Direction().Y()) >= 1e-9) continue;   // along v
            std::vector<double> prm;
            if (!edgeFaces.Contains(e)) continue;
            for (const TopoDS_Shape& gs : edgeFaces.FindFromKey(e)) {
                if (gs.IsSame(f)) continue;
                TopLoc_Location gl;
                Handle(Poly_Triangulation) gt = BRep_Tool::Triangulation(TopoDS::Face(gs), gl);
                if (gt.IsNull()) continue;
                Handle(Poly_PolygonOnTriangulation) po = BRep_Tool::PolygonOnTriangulation(e, gt, gl);
                if (po.IsNull() || !po->HasParameters()) continue;
                for (int i = 1; i <= po->NbNodes(); ++i) prm.push_back(po->Parameter(i));
                break;
            }
            if (prm.size() < 2) { rims.clear(); break; }
            std::sort(prm.begin(), prm.end());
            rims.push_back(std::move(prm));
        }
        if (rims.size() != 2 || rims[0].size() != rims[1].size()) continue;
        bool same = true;
        for (std::size_t i = 0; i < rims[0].size() && same; ++i)
            same = std::abs(rims[0][i] - rims[1][i]) < 1e-9;
        if (!same) continue;
        if (meshExtrusionWallStrip(f, deflection, angularDeflection, &rims[0])) ++redone;
    }
    return redone;
}

// Fork: edges next to a ruled wall whose two faces sample them differently -
// each one a visible crack in the shaded body (the wall was strip-meshed on
// its own, its neighbours by BRepMesh). Compares the 3D nodes of the edge's
// polygon on each face's triangulation.
inline int countWallCracks(const TopoDS_Shape& shape) {
    TopTools_IndexedDataMapOfShapeListOfShape edgeFaces;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeFaces);
    std::vector<TopoDS_Face> walls;
    for (TopExp_Explorer fx(shape, TopAbs_FACE); fx.More(); fx.Next()) {
        RuledWall w;
        if (ruledWallOf(TopoDS::Face(fx.Current()), w))
            walls.push_back(TopoDS::Face(fx.Current()));
    }
    if (walls.empty()) return 0;
    auto isWall = [&](const TopoDS_Shape& f) {
        for (const TopoDS_Face& w : walls) if (w.IsSame(f)) return true;
        return false;
    };
    int cracks = 0;
    for (int i = 1; i <= edgeFaces.Extent(); ++i) {
        const TopoDS_Edge e = TopoDS::Edge(edgeFaces.FindKey(i));
        const TopTools_ListOfShape& fl = edgeFaces.FindFromIndex(i);
        std::vector<TopoDS_Face> fs;
        for (const TopoDS_Shape& s : fl) {
            bool dup = false;
            for (const TopoDS_Face& g : fs) if (g.IsSame(s)) { dup = true; break; }
            if (!dup) fs.push_back(TopoDS::Face(s));
        }
        if (fs.size() != 2 || (!isWall(fs[0]) && !isWall(fs[1]))) continue;
        std::vector<gp_Pnt> pts[2];
        bool ok = true;
        for (int k = 0; k < 2 && ok; ++k) {
            TopLoc_Location loc;
            Handle(Poly_Triangulation) t = BRep_Tool::Triangulation(fs[k], loc);
            if (t.IsNull()) { ok = false; break; }
            Handle(Poly_PolygonOnTriangulation) po = BRep_Tool::PolygonOnTriangulation(e, t, loc);
            if (po.IsNull()) { ok = false; break; }
            const gp_Trsf tr = loc.Transformation();
            for (int j = 1; j <= po->NbNodes(); ++j)
                pts[k].push_back(t->Node(po->Node(j)).Transformed(tr));
        }
        if (!ok) continue;
        if (pts[0].size() != pts[1].size()) { ++cracks; continue; }
        const std::size_t n = pts[0].size();
        // Same edge, possibly walked in opposite senses by the two faces.
        bool fwd = true, rev = true;
        for (std::size_t j = 0; j < n && (fwd || rev); ++j) {
            if (pts[0][j].Distance(pts[1][j]) > 1e-4) fwd = false;
            if (pts[0][j].Distance(pts[1][n - 1 - j]) > 1e-4) rev = false;
        }
        if (!fwd && !rev) ++cracks;
    }
    return cracks;
}

// Fork: close the cracks between strip-meshed ruled walls and their
// neighbours on the DISPLAY paths (export already aligns, see
// meshWithFallback). First the cheap fix - re-strip each wall on the
// neighbours' samples; any crack left (a wall trimmed by a boolean, rims
// sampled differently) means the whole shape is re-meshed by BRepMesh alone,
// which shares every edge's discretization between its faces. Call after
// BRepMesh, before the bare-face fallback. Returns true when it re-meshed.
inline bool sealRuledWallCracks(const TopoDS_Shape& shape, double deflection,
                                double angularDeflection, bool inParallel) {
    if (countWallCracks(shape) == 0) return false;
    alignRuledWallsToEdges(shape, deflection, angularDeflection);
    const int left = countWallCracks(shape);
    if (left == 0) return false;
    std::fprintf(stderr, "[mesh] %d cracked edge(s) at ruled walls - re-meshing "
                         "the shape without the strip mesher\n", left);
    BRepTools::Clean(shape);
    BRepMesh_IncrementalMesh(shape, meshParams(deflection, angularDeflection, inParallel));
    return true;
}

inline void meshWithFallback(const TopoDS_Shape& shape, double deflection,
                             double angularDeflection, bool inParallel)
{
    premeshRuledWalls(shape, deflection, angularDeflection);
    BRepMesh_IncrementalMesh(shape, meshParams(deflection, angularDeflection, inParallel));
    alignRuledWallsToEdges(shape, deflection, angularDeflection);
    for (TopExp_Explorer fx(shape, TopAbs_FACE); fx.More(); fx.Next()) {
        const TopoDS_Face& f = TopoDS::Face(fx.Current());
        TopLoc_Location loc;
        if (!BRep_Tool::Triangulation(f, loc).IsNull()) continue;
        // A swept wall of a traced outline (whole, or trimmed by a boolean) is
        // exactly a ruled grid; Watson takes seconds to minutes on it (the
        // autumn lettering: 131 s at Ultra). Same shortcut the display path uses.
        if (meshExtrusionWallStrip(f, deflection, angularDeflection)) continue;
        try {
            IMeshTools_Parameters wp = meshParams(deflection, angularDeflection, false);
            wp.MeshAlgo = IMeshTools_MeshAlgoType_Watson;
            BRepMesh_IncrementalMesh(f, wp);
        } catch (...) {
            // Fall through to the escalating retry below.
        }
        if (BRep_Tool::Triangulation(f, loc).IsNull())
            meshBareFaceEscalating(f, deflection, angularDeflection);
    }
    repairInaccurateFaces(shape, deflection, angularDeflection);
}

} // namespace materializr
