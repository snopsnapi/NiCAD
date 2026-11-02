#include "SurfaceEditControllers.h"
#include "../core/Document.h"
#include "../core/SelectionManager.h"
#include "../modeling/SurfaceEditOps.h"
#include "../ui/LengthField.h"
#include <TopoDS.hxx>
#include <cmath>
#include <imgui.h>
#include "../i18n.h"

namespace materializr {

// ── Extend ──────────────────────────────────────────────────────────────────

int ExtendSurfaceController::onBegin(const IopContext& ctx) {
    m_edges.clear();
    int body = -1;
    for (const auto& e : ctx.selection.getSelection()) {
        if (e.type != SelectionType::Edge || e.bodyId < 0 || e.shape.IsNull()) continue;
        if (body < 0) body = e.bodyId;
        if (e.bodyId != body) continue;   // one body per step
        m_edges.push_back(TopoDS::Edge(e.shape));
    }
    if (m_edges.empty()) return -1;
    return body;
}

std::unique_ptr<Operation> ExtendSurfaceController::buildOp(const IopContext&) {
    if (m_edges.empty() || m_length < 0.01f) return nullptr;
    auto op = std::make_unique<ExtendSurfaceOp>();
    op->setBody(bodyId());
    op->setEdges(m_edges);
    op->setLength(m_length);
    return op;
}

void ExtendSurfaceController::panelBody(const IopContext& ctx, bool& changed) {
    if (ctx.cornerCommitUi) {
        if (materializr::amountLengthField("extendAmt", materializr::tr("Distance"),
                                           &m_length, /*allowSign=*/false, 0.1f, 50.0f))
            changed = true;
    } else if (materializr::lengthField(materializr::tr("Distance"), &m_length)) {
        changed = true;
    }
    if (!previewOk())
        ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.4f, 1.0f), "%s",
                           materializr::tr("Only free edges extend."));
}

// ── Offset ──────────────────────────────────────────────────────────────────

int OffsetSurfaceController::onBegin(const IopContext& ctx) {
    m_faces.clear();
    m_srcBody = -1;
    for (const auto& e : ctx.selection.getSelection()) {
        if (e.type != SelectionType::Face || e.bodyId < 0 || e.shape.IsNull()) continue;
        if (m_srcBody < 0) m_srcBody = e.bodyId;
        if (e.bodyId != m_srcBody) continue;
        m_faces.push_back(TopoDS::Face(e.shape));
    }
    if (m_faces.empty()) return -1;
    return kNoTargetBody;   // the offset surface is a new body
}

std::unique_ptr<Operation> OffsetSurfaceController::buildOp(const IopContext&) {
    auto op = std::make_unique<OffsetSurfaceOp>();
    op->setSource(m_srcBody);
    op->setFaces(m_faces);
    op->setDistance(m_distance);
    return op;
}

bool OffsetSurfaceController::syncLiveOp(Operation& op) {
    static_cast<OffsetSurfaceOp&>(op).setDistance(m_distance);
    return std::abs(m_distance) >= 0.01f;
}

void OffsetSurfaceController::panelBody(const IopContext& ctx, bool& changed) {
    if (ctx.cornerCommitUi) {
        if (materializr::amountLengthField("offsetAmt", materializr::tr("Distance"),
                                           &m_distance, /*allowSign=*/true, -50.0f, 50.0f))
            changed = true;
    } else if (materializr::lengthField(materializr::tr("Distance"), &m_distance)) {
        changed = true;
    }
    if (ImGui::Button(materializr::tr("Flip"), ImVec2(-1.0f, 0.0f))) {
        m_distance = -m_distance;
        changed = true;
    }
}

} // namespace materializr
