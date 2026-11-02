// Fork: Studio Spline (3D curve) points - listing, picking, and the Move
// gizmo's drag of them. The points live in their StudioSplineOp; moving them
// rewrites the op and replays history from it, so a sweep along the curve
// follows.
#include "app/Application.h"
#include "core/Document.h"
#include "core/History.h"
#include "core/SelectionManager.h"
#include "modeling/StudioSplineOp.h"
#include "viewport/Camera.h"
#include "viewport/Viewport.h"
#include "i18n.h"
#include <algorithm>

namespace materializr {

std::vector<Application::CurvePointRef> Application::curvePoints() const {
    std::vector<CurvePointRef> out;
    if (!m_history || !m_document) return out;
    const int upTo = m_history->currentStep();
    for (int s = 0; s <= upTo; ++s) {
        const auto* op = dynamic_cast<const StudioSplineOp*>(m_history->getStep(s));
        if (!op || !op->isEnabled() || op->createdBodyId() < 0) continue;
        const int body = op->createdBodyId();
        if (!m_document->isBodyVisible(body)) continue;   // false when gone too
        const auto& pts = op->points();
        for (size_t i = 0; i < pts.size(); ++i) {
            CurvePointRef r;
            r.step = s;
            r.bodyId = body;
            r.index = static_cast<int>(i);
            r.p = glm::vec3(pts[i].X(), pts[i].Y(), pts[i].Z());
            out.push_back(r);
        }
    }
    return out;
}

bool Application::pickCurvePoint(float x, float y, float vpW, float vpH, float radiusPx,
                                 CurvePointRef& out) const {
    if (!m_viewport || vpW <= 0.0f || vpH <= 0.0f) return false;
    const Camera& cam = m_viewport->getCamera();
    const glm::mat4 vp = cam.getProjectionMatrix() * cam.getViewMatrix();
    float best = radiusPx;
    bool found = false;
    for (const CurvePointRef& r : curvePoints()) {
        const glm::vec4 c = vp * glm::vec4(r.p, 1.0f);
        if (c.w <= 1e-5f) continue;
        const float sx = (c.x / c.w * 0.5f + 0.5f) * vpW;
        const float sy = (0.5f - c.y / c.w * 0.5f) * vpH;
        const float d = std::hypot(sx - x, sy - y);
        if (d < best) { best = d; out = r; found = true; }
    }
    return found;
}

bool Application::hasSelectedCurvePoints() const {
    if (!m_selection) return false;
    for (const auto& e : m_selection->getSelection())
        if (e.type == SelectionType::CurvePoint) return true;
    return false;
}

glm::vec3 Application::selectedCurvePointsCentre() const {
    glm::vec3 sum(0.0f);
    int n = 0;
    if (!m_selection) return sum;
    const std::vector<CurvePointRef> all = curvePoints();
    for (const auto& e : m_selection->getSelection()) {
        if (e.type != SelectionType::CurvePoint) continue;
        for (const CurvePointRef& r : all)
            if (r.bodyId == e.bodyId && r.index == e.subShapeIndex) { sum += r.p; ++n; break; }
    }
    return n ? sum / static_cast<float>(n) : sum;
}

void Application::beginCurveDrag() {
    m_curveDrag.clear();
    if (!m_selection) return;
    const std::vector<CurvePointRef> all = curvePoints();
    for (const auto& e : m_selection->getSelection()) {
        if (e.type != SelectionType::CurvePoint) continue;
        for (const CurvePointRef& r : all)
            if (r.bodyId == e.bodyId && r.index == e.subShapeIndex) {
                m_curveDrag.push_back({r.step, r.index, gp_Pnt(r.p.x, r.p.y, r.p.z)});
                break;
            }
    }
}

void Application::applyCurveDrag(const gp_Trsf& t) {
    if (m_curveDrag.empty() || !m_history || !m_document) return;
    auto trackBodies = trackBodyChanges();
    std::vector<int> steps;
    for (const CurveDragPt& d : m_curveDrag) {
        auto* op = dynamic_cast<StudioSplineOp*>(
            const_cast<Operation*>(m_history->getStep(d.step)));
        if (!op) continue;
        op->setPoint(static_cast<size_t>(d.index), d.orig.Transformed(t));
        if (std::find(steps.begin(), steps.end(), d.step) == steps.end())
            steps.push_back(d.step);
    }
    // Live: rebuild just the curves (same body ids); downstream steps catch
    // up once, on release.
    for (int s : steps)
        if (auto* op = dynamic_cast<StudioSplineOp*>(
                const_cast<Operation*>(m_history->getStep(s))))
            op->execute(*m_document);
}

void Application::commitCurveDrag() {
    if (m_curveDrag.empty() || !m_history || !m_document) { m_curveDrag.clear(); return; }
    int first = m_curveDrag.front().step;
    for (const CurveDragPt& d : m_curveDrag) first = std::min(first, d.step);
    auto trackBodies = trackBodyChanges();
    if (!m_history->editStep(first, *m_document, /*transactional=*/true)) {
        // A downstream step can't follow - put the points back.
        for (const CurveDragPt& d : m_curveDrag)
            if (auto* op = dynamic_cast<StudioSplineOp*>(
                    const_cast<Operation*>(m_history->getStep(d.step))))
                op->setPoint(static_cast<size_t>(d.index), d.orig);
        m_history->editStep(first, *m_document, /*transactional=*/true);
        showToast(materializr::tr("A later step can't follow this move - the points went back."));
    }
    m_curveDrag.clear();
    markDirty();
}

} // namespace materializr
