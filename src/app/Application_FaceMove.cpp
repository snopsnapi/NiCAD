// Fork: Move > Faces - Fusion's Move/Copy with "Faces": the same combined
// gizmo as bodies (arrows, quarter-arc rotations, plane squares) on a picked
// planar face. The face moves rigidly and only its neighbours are rebuilt
// (FaceTweak - re-intersect, retrim), the rest of the body stays put. The live
// drag re-runs the tweak on the original body every frame; release records one
// FaceTweakOp.

#include "app/Application.h"
#include "core/Document.h"
#include "core/History.h"
#include "core/SelectionManager.h"
#include "modeling/FaceTweak.h"
#include "modeling/FaceTweakOp.h"
#include "i18n.h"
#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <Geom_Plane.hxx>
#include <Geom_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <cmath>
#include <limits>

namespace materializr {

namespace {

bool planarFace(const TopoDS_Face& f, gp_Pln& out) {
    try {
        BRepAdaptor_Surface s(f, Standard_False);
        if (s.GetType() != GeomAbs_Plane) return false;
        out = s.Plane();
        if (f.Orientation() == TopAbs_REVERSED) {
            gp_Ax3 a = out.Position();
            a.ZReverse();
            out.SetPosition(a);
        }
        return true;
    } catch (...) {
        return false;
    }
}

glm::vec3 faceCentre(const TopoDS_Face& f) {
    GProp_GProps g;
    BRepGProp::SurfaceProperties(f, g);
    const gp_Pnt c = g.Mass() > 1e-12 ? g.CentreOfMass() : gp_Pnt();
    return glm::vec3(static_cast<float>(c.X()), static_cast<float>(c.Y()),
                     static_cast<float>(c.Z()));
}

} // namespace

bool Application::faceMoveTarget(int& bodyId, TopoDS_Face& face) const {
    if (!m_moveFacesMode || !m_selection) return false;
    for (const auto& e : m_selection->getSelection()) {
        if (e.type != SelectionType::Face || e.bodyId < 0 || e.shape.IsNull()) continue;
        gp_Pln pl;
        const TopoDS_Face f = TopoDS::Face(e.shape);
        if (!planarFace(f, pl)) continue;
        bodyId = e.bodyId;
        face = f;
        return true;
    }
    return false;
}

glm::vec3 Application::faceMoveGizmoCentre() const {
    int bid = -1;
    TopoDS_Face f;
    if (m_faceDrag.active) return m_faceDrag.centre;
    if (!faceMoveTarget(bid, f)) return glm::vec3(0.0f);
    const void* key = f.TShape().get();
    if (key != m_faceCentreKey) {
        m_faceCentreKey = key;
        m_faceCentreCache = faceCentre(f);
    }
    return m_faceCentreCache;
}

void Application::beginFaceDrag() {
    m_faceDrag = FaceDrag{};
    int bid = -1;
    TopoDS_Face f;
    if (!faceMoveTarget(bid, f)) return;
    try {
        m_faceDrag.orig = m_document->getBody(bid);
    } catch (...) {
        return;
    }
    if (m_faceDrag.orig.IsNull()) return;
    m_faceDrag.bodyId = bid;
    m_faceDrag.face = f;
    m_faceDrag.centre = faceCentre(f);
    m_faceDrag.active = true;
    m_faceDrag.haveGood = false;
    m_faceDrag.refusal.clear();
}

void Application::applyFaceDrag(const gp_Trsf& t) {
    if (!m_faceDrag.active) return;
    const tweak::Result r = tweak::moveFace(m_faceDrag.orig, m_faceDrag.face, t);
    if (r.ok()) {
        try { m_document->updateBody(m_faceDrag.bodyId, r.shape); } catch (...) { return; }
        m_faceDrag.haveGood = true;
        m_faceDrag.refusal.clear();
    } else {
        // Keep the last good pose on screen; say why at release.
        m_faceDrag.refusal = tweak::refusalText(r.refusal);
        if (!m_faceDrag.haveGood) {
            try { m_document->updateBody(m_faceDrag.bodyId, m_faceDrag.orig); } catch (...) {}
        }
    }
    markBodyDirty(m_faceDrag.bodyId);
}

void Application::commitFaceDrag(const gp_Trsf& t) {
    if (!m_faceDrag.active) return;
    const FaceDrag d = m_faceDrag;
    m_faceDrag = FaceDrag{};
    // History runs the op against the un-previewed body.
    try { m_document->updateBody(d.bodyId, d.orig); } catch (...) {}
    markBodyDirty(d.bodyId);
    if (t.Form() == gp_Identity) return;

    auto op = std::make_unique<FaceTweakOp>();
    op->setBody(d.bodyId);
    op->setFace(d.face);
    op->setTransform(t);
    auto trackBodies = trackBodyChanges();
    // The op is gone once a push is refused - the reason comes from the
    // engine run on the same input.
    if (!m_history->pushOperation(std::move(op), *m_document)) {
        const tweak::Result r = tweak::moveFace(d.orig, d.face, t);
        showToast(std::string(materializr::tr("Couldn't move the face: ")) +
                  tweak::refusalText(r.refusal));
        return;
    }
    markBodyDirty(d.bodyId);
    markDirty();

    // Keep the moved face selected, so the next drag continues from it: the
    // rebuilt body's planar face nearest the moved centre, facing the moved way.
    gp_Pln before;
    if (!planarFace(d.face, before)) { m_selection->clear(); return; }
    gp_Dir n = before.Axis().Direction();
    n.Transform(t);
    gp_Pnt c(d.centre.x, d.centre.y, d.centre.z);
    c.Transform(t);
    TopoDS_Shape body;
    try { body = m_document->getBody(d.bodyId); } catch (...) { m_selection->clear(); return; }
    int idx = 0, bestIdx = -1;
    double best = std::numeric_limits<double>::max();
    TopoDS_Face bestFace;
    for (TopExp_Explorer ex(body, TopAbs_FACE); ex.More(); ex.Next(), ++idx) {
        const TopoDS_Face f = TopoDS::Face(ex.Current());
        gp_Pln pl;
        if (!planarFace(f, pl)) continue;
        if (pl.Axis().Direction().Dot(n) < 0.999) continue;
        const glm::vec3 fc = faceCentre(f);
        const double dist = c.Distance(gp_Pnt(fc.x, fc.y, fc.z));
        if (dist < best) { best = dist; bestIdx = idx; bestFace = f; }
    }
    m_selection->clear();
    if (bestIdx >= 0) {
        SelectionEntry e;
        e.type = SelectionType::Face;
        e.bodyId = d.bodyId;
        e.subShapeIndex = bestIdx;
        e.shape = bestFace;
        m_selection->select(e);
    }
}

void Application::cancelFaceDrag() {
    if (!m_faceDrag.active) return;
    try { m_document->updateBody(m_faceDrag.bodyId, m_faceDrag.orig); } catch (...) {}
    markBodyDirty(m_faceDrag.bodyId);
    m_faceDrag = FaceDrag{};
}

} // namespace materializr
