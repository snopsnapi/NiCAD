#include "TrimSurfaceController.h"
#include "../core/Document.h"
#include "../core/SelectionManager.h"
#include "../modeling/SurfaceOps.h"
#include "../modeling/TrimSurfaceOp.h"
#include "../ui_scale.h"
#include <imgui.h>
#include "../i18n.h"

namespace materializr {

int TrimSurfaceController::onBegin(const IopContext& ctx) {
    // First body = the one to trim; a second body or any sketch = the tool.
    int target = -1;
    m_toolBody = m_toolSketch = -1;
    m_removeAt.clear();
    for (const auto& e : ctx.selection.getSelection()) {
        if (e.type == SelectionType::Body && e.bodyId >= 0) {
            if (target < 0) target = e.bodyId;
            else if (m_toolBody < 0 && e.bodyId != target) m_toolBody = e.bodyId;
        } else if ((e.type == SelectionType::Sketch ||
                    e.type == SelectionType::SketchRegion) && e.sketchId >= 0) {
            if (m_toolSketch < 0) m_toolSketch = e.sketchId;
        }
    }
    if (m_toolBody >= 0) m_toolSketch = -1;   // a body tool wins
    if (target < 0 || (m_toolBody < 0 && m_toolSketch < 0)) {
        if (ctx.toast)
            ctx.toast(materializr::tr("Trim needs the body to trim and a tool: another body or a sketch."));
        return -1;
    }
    ctx.selection.clear();
    return target;
}

std::unique_ptr<Operation> TrimSurfaceController::buildOp(const IopContext&) {
    auto op = std::make_unique<TrimSurfaceOp>();
    op->setTarget(bodyId());
    if (m_toolBody >= 0) op->setToolBody(m_toolBody);
    else op->setToolSketch(m_toolSketch);
    // Nothing tapped yet: the preview is the split itself, so the pieces'
    // borders show where to tap.
    op->setRemovePoints(m_removeAt);
    return op;
}

// A tap (press and release without moving) on the shown body removes the
// piece under it. Drags fall through to the camera.
void TrimSurfaceController::onViewportInput(const IopViewport& vp,
                                            const IopContext& ctx) {
    if (!active()) return;
    if (vp.clicked) {
        m_pressed = true;
        m_moved = false;
        m_pressAt = vp.mouse;
    }
    if (m_pressed && glm::length(vp.mouse - m_pressAt) > 12.0f * uiScale())
        m_moved = true;
    if (m_pressed && vp.released) {
        m_pressed = false;
        if (m_moved || vp.uiCaptured) return;
        TopoDS_Shape shown;
        try { shown = ctx.doc.getBody(bodyId()); } catch (...) { return; }
        gp_Pnt hit;
        const glm::vec3 d = glm::normalize(vp.rayDir);
        if (!surf::pickPoint(shown, gp_Pnt(vp.rayOrigin.x, vp.rayOrigin.y, vp.rayOrigin.z),
                             gp_Dir(d.x, d.y, d.z), hit))
            return;
        m_removeAt.push_back(hit);
        update(ctx);
        if (!previewOk()) {
            // That tap took the last piece, or the trim failed - undo it.
            m_removeAt.pop_back();
            update(ctx);
        }
    }
}

void TrimSurfaceController::panelBody(const IopContext&, bool& changed) {
    ImGui::Text(materializr::tr("Pieces removed: %d"), static_cast<int>(m_removeAt.size()));
    ImGui::BeginDisabled(m_removeAt.empty());
    if (ImGui::Button(materializr::tr("Reset"), ImVec2(-1.0f, 44.0f * uiScale()))) {
        m_removeAt.clear();
        changed = true;
    }
    ImGui::EndDisabled();
}

void TrimSurfaceController::drawOverlay(const IopOverlay& ov) const {
    if (!active() || !ov.toScreen || !ov.disc) return;
    // Where each removed piece was tapped.
    const float r = 5.0f * uiScale();
    for (const gp_Pnt& p : m_removeAt) {
        glm::vec2 sp;
        if (ov.toScreen(glm::vec3(p.X(), p.Y(), p.Z()), sp))
            ov.disc(sp, r, 0xFF3C3CE6u);   // red (0xAABBGGRR)
    }
}

void TrimSurfaceController::onCleanup() {
    m_toolBody = m_toolSketch = -1;
    m_removeAt.clear();
    m_pressed = m_moved = false;
}

} // namespace materializr
