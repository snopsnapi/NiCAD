#include "StudioSplineController.h"
#include "../core/Document.h"
#include "../core/SelectionManager.h"
#include "../modeling/StudioSplineOp.h"
#include "../modeling/SurfaceOps.h"
#include "../ui_scale.h"
#include <imgui.h>
#include "../i18n.h"

namespace materializr {

int StudioSplineController::onBegin(const IopContext& ctx) {
    m_points.clear();
    m_closed = false;
    ctx.selection.clear();
    return kNoTargetBody;   // the curve is a new body
}

std::unique_ptr<Operation> StudioSplineController::buildOp(const IopContext&) {
    auto op = std::make_unique<StudioSplineOp>();
    op->setPoints(m_points);
    op->setClosed(m_closed);
    return op;
}

bool StudioSplineController::syncLiveOp(Operation& op) {
    auto& s = static_cast<StudioSplineOp&>(op);
    s.setPoints(m_points);
    s.setClosed(m_closed);
    return m_points.size() >= 2;
}

void StudioSplineController::onViewportInput(const IopViewport& vp, const IopContext& ctx) {
    if (!active()) return;
    if (vp.clicked) {
        m_pressed = true;
        m_moved = false;
        m_pressAt = vp.mouse;
    }
    if (m_pressed && glm::length(vp.mouse - m_pressAt) > 12.0f * uiScale()) m_moved = true;
    if (!(m_pressed && vp.released)) return;
    m_pressed = false;
    if (m_moved || vp.uiCaptured) return;   // a drag orbits; only taps place

    const glm::vec3 dir = glm::normalize(vp.rayDir);
    const gp_Pnt o(vp.rayOrigin.x, vp.rayOrigin.y, vp.rayOrigin.z);
    const gp_Dir d(dir.x, dir.y, dir.z);
    // On a body: the nearest surface hit under the finger (the live curve
    // itself has no faces, so it never catches the ray).
    bool have = false;
    gp_Pnt best;
    double bestT = 1e300;
    for (int id : ctx.doc.getAllBodyIds()) {
        if (!ctx.doc.isBodyVisible(id)) continue;
        gp_Pnt hit;
        try {
            if (!surf::pickPoint(ctx.doc.getBody(id), o, d, hit)) continue;
        } catch (...) { continue; }
        const double t = o.Distance(hit);
        if (t < bestT) { bestT = t; best = hit; have = true; }
    }
    if (!have) {
        // Empty space: the plane through the previous point (or the view
        // target) facing the camera.
        const glm::vec3 anchor = m_points.empty()
            ? vp.camTarget
            : glm::vec3(m_points.back().X(), m_points.back().Y(), m_points.back().Z());
        const glm::vec3 n = glm::normalize(vp.camTarget - vp.camPos);
        const float denom = glm::dot(dir, n);
        if (std::abs(denom) < 1e-6f) return;
        const float t = glm::dot(anchor - vp.rayOrigin, n) / denom;
        if (t <= 0.0f) return;
        const glm::vec3 p = vp.rayOrigin + dir * t;
        best = gp_Pnt(p.x, p.y, p.z);
    }
    if (!m_points.empty() && m_points.back().Distance(best) < 1e-4) return;
    m_points.push_back(best);
    update(ctx);
}

void StudioSplineController::panelBody(const IopContext&, bool& changed) {
    ImGui::Text(materializr::tr("Points: %d"), static_cast<int>(m_points.size()));
    const float h = 44.0f * uiScale();
    ImGui::BeginDisabled(m_points.empty());
    if (ImGui::Button(materializr::tr("Undo point"), ImVec2(-1.0f, h))) {
        m_points.pop_back();
        changed = true;
    }
    ImGui::EndDisabled();
    if (ImGui::Checkbox(materializr::tr("Closed"), &m_closed)) changed = true;
}

void StudioSplineController::drawOverlay(const IopOverlay& ov) const {
    if (!active() || !ov.toScreen || !ov.disc) return;
    const float r = 5.0f * uiScale();
    for (const gp_Pnt& p : m_points) {
        glm::vec2 sp;
        if (ov.toScreen(glm::vec3(p.X(), p.Y(), p.Z()), sp)) {
            ov.disc(sp, r + 1.5f * uiScale(), 0xE61E1614u);
            ov.disc(sp, r, 0xFFFAF0EBu);
        }
    }
}

void StudioSplineController::onCleanup() {
    m_points.clear();
    m_closed = false;
    m_pressed = m_moved = false;
}

} // namespace materializr
