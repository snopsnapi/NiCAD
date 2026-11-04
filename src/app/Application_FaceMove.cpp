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
#include "modeling/MoveOp.h"
#include "viewport/Gizmo.h"
#include <algorithm>
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

    auto trackBodies = trackBodyChanges();
    // Fusion: one Move step per session - a further drag of the same face
    // composes into it.
    if (MoveOp* step = moveStepFor(true, {d.bodyId}, d.face)) {
        if (!replayMoveStep(step, t * step->trsf())) return;
    } else {
        auto op = std::make_unique<MoveOp>();
        op->setFace(d.bodyId, d.face);
        op->setPivot(d.centre.x, d.centre.y, d.centre.z);
        op->setFromTrsf(t);
        if (!m_history->pushOperation(std::move(op), *m_document)) {
            const tweak::Result r = tweak::moveFace(d.orig, d.face, t);
            showToast(std::string(materializr::tr("Couldn't move the face: ")) +
                      tweak::refusalText(r.refusal));
            return;
        }
        m_moveStep = m_history->currentStep();
        m_moveStepEditing = false;
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
        m_moveStepFace = bestFace;
    }
}

// ── Parametric Move session ────────────────────────────────────────────────

MoveOp* Application::activeMoveStep() const {
    if (m_moveStep < 0 || !m_history) return nullptr;
    if (!m_moveStepEditing && m_moveStep != m_history->currentStep()) return nullptr;
    if (m_moveStep > m_history->currentStep()) return nullptr;
    return dynamic_cast<MoveOp*>(const_cast<Operation*>(m_history->getStep(m_moveStep)));
}

MoveOp* Application::moveStepFor(bool face, const std::vector<int>& bodyIds,
                                 const TopoDS_Face& f) const {
    MoveOp* op = activeMoveStep();
    if (!op) return nullptr;
    if ((op->target() == MoveOp::Target::Face) != face) return nullptr;
    std::vector<int> a = op->bodyIds(), b = bodyIds;
    std::sort(a.begin(), a.end());
    std::sort(b.begin(), b.end());
    if (a != b) return nullptr;
    if (face && (m_moveStepFace.IsNull() || !m_moveStepFace.IsSame(f))) return nullptr;
    return op;
}

bool Application::replayMoveStep(MoveOp* op, const gp_Trsf& newTotal) {
    if (!op || m_moveStep < 0) return false;
    const double t0[3] = {op->translation()[0], op->translation()[1], op->translation()[2]};
    const double r0[3] = {op->rotationDeg()[0], op->rotationDeg()[1], op->rotationDeg()[2]};
    op->setFromTrsf(newTotal);
    auto trackBodies = trackBodyChanges();
    if (!m_history->editStep(m_moveStep, *m_document, /*transactional=*/true)) {
        op->setParams(t0, r0);
        m_history->editStep(m_moveStep, *m_document, true);
        showToast(materializr::tr("Couldn't apply the move - the result is invalid or later steps depend on it."));
        return false;
    }
    for (int id : op->bodyIds()) markBodyDirty(id);
    markDirty();
    return true;
}

void Application::commitMoveBodies(const std::vector<std::pair<int, std::vector<int>>>& bodies,
                                   const gp_Trsf& d, const glm::vec3& pivot) {
    std::vector<int> ids;
    for (const auto& b : bodies) ids.push_back(b.first);
    if (MoveOp* step = moveStepFor(false, ids, TopoDS_Face())) {
        replayMoveStep(step, d * step->trsf());
        return;
    }
    auto op = std::make_unique<MoveOp>();
    for (const auto& [id, det] : bodies) op->addBody(id, det);
    op->setPivot(pivot.x, pivot.y, pivot.z);
    op->setFromTrsf(d);
    auto trackBodies = trackBodyChanges();
    if (!m_history->pushOperation(std::move(op), *m_document)) {
        showToast(materializr::tr("Couldn't move the body."));
        return;
    }
    m_moveStep = m_history->currentStep();
    m_moveStepEditing = false;
    for (int id : ids) markBodyDirty(id);
}

bool Application::moveStepMatchesSelection(MoveOp* op) const {
    if (!op) return false;
    if (m_moveStepEditing) return true;
    if (op->target() == MoveOp::Target::Face) {
        int b = -1;
        TopoDS_Face f;
        return faceMoveTarget(b, f) && !m_moveStepFace.IsNull() && m_moveStepFace.IsSame(f);
    }
    std::vector<int> sel;
    for (const auto& e : m_selection->getSelection())
        if (e.type == SelectionType::Body && e.bodyId >= 0) sel.push_back(e.bodyId);
    std::vector<int> ids = op->bodyIds();
    std::sort(sel.begin(), sel.end());
    std::sort(ids.begin(), ids.end());
    return sel == ids;
}

bool Application::applyMoveStepValues(MoveOp* op) {
    if (!op) return false;
    // User axes (Z up) -> world: user Y = world Z, user Z = world Y.
    const double t[3] = {m_moveStepVals[0], m_moveStepVals[2], m_moveStepVals[1]};
    const double r[3] = {m_moveStepVals[3], m_moveStepVals[5], m_moveStepVals[4]};
    const double t0[3] = {op->translation()[0], op->translation()[1], op->translation()[2]};
    const double r0[3] = {op->rotationDeg()[0], op->rotationDeg()[1], op->rotationDeg()[2]};
    op->setParams(t, r);
    auto trackBodies = trackBodyChanges();
    if (!m_history->editStep(m_moveStep, *m_document, /*transactional=*/true)) {
        op->setParams(t0, r0);
        m_history->editStep(m_moveStep, *m_document, true);
        showToast(materializr::tr("Couldn't apply the move - the result is invalid or later steps depend on it."));
        return false;
    }
    for (int id : op->bodyIds()) markBodyDirty(id);
    markDirty();
    // The moved face handle changed with the rebuild; the next drag starts a
    // new step unless the user picks the face again.
    if (op->target() == MoveOp::Target::Face) m_moveStepFace.Nullify();
    return true;
}

bool Application::beginMoveStepEdit(int historyIndex) {
    auto* op = dynamic_cast<MoveOp*>(const_cast<Operation*>(m_history->getStep(historyIndex)));
    if (!op) return false;
    m_moveStep = historyIndex;
    m_moveStepEditing = true;
    m_moveStepFace.Nullify();
    m_selection->clear();
    if (op->target() == MoveOp::Target::Bodies) {
        m_moveFacesMode = false;
        for (int id : op->bodyIds()) {
            SelectionEntry e;
            e.type = SelectionType::Body;
            e.bodyId = id;
            try { e.shape = m_document->getBody(id); } catch (...) {}
            m_selection->addToSelection(e);
        }
    } else {
        m_moveFacesMode = true;
    }
    m_gizmo->setCombinedMove();
    m_selection->setNavigationOnly(false);
    return true;
}

void Application::cancelFaceDrag() {
    if (!m_faceDrag.active) return;
    try { m_document->updateBody(m_faceDrag.bodyId, m_faceDrag.orig); } catch (...) {}
    markBodyDirty(m_faceDrag.bodyId);
    m_faceDrag = FaceDrag{};
}

} // namespace materializr
