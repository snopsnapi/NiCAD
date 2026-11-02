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

// ── Studio Spline editor ────────────────────────────────────────────────────

#include "../core/History.h"
#include "../core/BodyChanges.h"
#include <BRep_Tool.hxx>
#include <Geom_Curve.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

namespace materializr {

StudioSplineOp* StudioSplineEditController::op(const IopContext& ctx) const {
    return dynamic_cast<StudioSplineOp*>(const_cast<Operation*>(ctx.history.getStep(m_step)));
}

bool StudioSplineEditController::beginEdit(const IopContext& ctx, int historyIndex) {
    m_step = historyIndex;
    if (!op(ctx)) { m_step = -1; return false; }
    if (!begin(ctx)) { m_step = -1; return false; }
    return true;
}

int StudioSplineEditController::onBegin(const IopContext& ctx) {
    StudioSplineOp* o = op(ctx);
    if (!o) return -1;
    m_before = o->serializeParams();
    m_points = o->points();
    m_closed = o->closed();
    m_conds.clear();
    for (size_t i = 0; i < m_points.size(); ++i) m_conds.push_back(o->condition(i));
    m_sel = -1;
    m_preview.begin(ctx.doc, ctx.history);
    ctx.selection.clear();
    return o->createdBodyId() >= 0 ? o->createdBodyId() : kNoTargetBody;
}

void StudioSplineEditController::writeParams(const IopContext& ctx) const {
    StudioSplineOp* o = op(ctx);
    if (!o) return;
    o->setPoints(m_points);
    o->setClosed(m_closed);
    for (size_t i = 0; i < m_points.size(); ++i)
        o->setCondition(i, i < m_conds.size() ? m_conds[i] : StudioSplineOp::PointCond());
}

void StudioSplineEditController::update(const IopContext& ctx) {
    if (!active() || m_points.size() < 2) return;
    materializr::BodyChangeScope trackBodies(ctx.doc, ctx.markBodyDirty, ctx.markMeshesDirty);
    writeParams(ctx);
    setPreviewOk(m_preview.replay(m_step, ctx.doc, ctx.history));
}

void StudioSplineEditController::finish(const IopContext& ctx) {
    m_preview.clear();
    ctx.selection.clear();
    if (ctx.markMeshesDirty) ctx.markMeshesDirty();
    teardown();
}

void StudioSplineEditController::commit(const IopContext& ctx) {
    if (!active()) { finish(ctx); return; }
    materializr::BodyChangeScope trackBodies(ctx.doc, ctx.markBodyDirty, ctx.markMeshesDirty);
    writeParams(ctx);
    const int step = m_step;
    const std::string before = m_before;
    if (m_points.size() >= 2 && m_preview.replay(m_step, ctx.doc, ctx.history)) {
        finish(ctx);
        if (onCommitted) onCommitted(step, before);
        return;
    }
    if (StudioSplineOp* o = op(ctx)) o->deserializeParams(before);
    m_preview.replay(m_step, ctx.doc, ctx.history);
    if (ctx.toast)
        ctx.toast(materializr::tr("A later step can't follow this edit - the curve went back."));
    finish(ctx);
}

void StudioSplineEditController::cancel(const IopContext& ctx) {
    if (active()) {
        materializr::BodyChangeScope trackBodies(ctx.doc, ctx.markBodyDirty, ctx.markMeshesDirty);
        if (StudioSplineOp* o = op(ctx)) o->deserializeParams(m_before);
        m_preview.replay(m_step, ctx.doc, ctx.history);
    }
    finish(ctx);
}

namespace {

// The curve's tangent and curvature vector at the point nearest `p`.
bool curveFrameAt(const std::vector<gp_Pnt>& pts, bool closed,
                  const std::vector<StudioSplineOp::PointCond>& conds, const gp_Pnt& p,
                  gp_Vec& dir, gp_Vec& curv) {
    const TopoDS_Shape w = StudioSplineOp::buildCurve(pts, closed, gp_Vec(), gp_Vec(), conds);
    if (w.IsNull()) return false;
    TopExp_Explorer ex(w, TopAbs_EDGE);
    if (!ex.More()) return false;
    double f, l;
    Handle(Geom_Curve) c = BRep_Tool::Curve(TopoDS::Edge(ex.Current()), f, l);
    if (c.IsNull()) return false;
    double tBest = f, dBest = 1e300;
    for (int i = 0; i <= 2000; ++i) {
        const double t = f + (l - f) * i / 2000.0;
        const double d = c->Value(t).SquareDistance(p);
        if (d < dBest) { dBest = d; tBest = t; }
    }
    gp_Pnt q;
    gp_Vec d1, d2;
    c->D2(tBest, q, d1, d2);
    const double s = d1.Magnitude();
    if (s < 1e-12) return false;
    dir = d1 / s;
    // kappa * N = (d2 - (d2.T) T) / |d1|^2
    curv = (d2 - dir * d2.Dot(dir)) / (s * s);
    return true;
}

} // namespace

void StudioSplineEditController::onViewportInput(const IopViewport& vp, const IopContext& ctx) {
    if (!active()) return;
    const float pick = 24.0f * uiScale();
    auto nearestPoint = [&](glm::vec2 at) {
        int best = -1;
        float bd = pick;
        for (size_t i = 0; i < m_points.size(); ++i) {
            glm::vec2 sp;
            const gp_Pnt& p = m_points[i];
            if (!vp.toScreen(glm::vec3(p.X(), p.Y(), p.Z()), sp)) continue;
            const float d = glm::length(sp - at);
            if (d < bd) { bd = d; best = static_cast<int>(i); }
        }
        return best;
    };
    if (vp.clicked) {
        m_pressed = true;
        m_moved = false;
        m_pressAt = vp.mouse;
        m_pressPoint = nearestPoint(vp.mouse);
        // Pressing the picked point claims the drag (no orbit).
        m_draggingPoint = (m_pressPoint >= 0 && m_pressPoint == m_sel);
        setDraggingHandle(m_draggingPoint);
    }
    if (m_pressed && glm::length(vp.mouse - m_pressAt) > 12.0f * uiScale()) m_moved = true;

    if (m_draggingPoint && vp.down && m_sel >= 0) {
        // The point follows the finger on the plane through it facing the
        // camera; only the curve rebuilds per frame, the replay runs on lift.
        const gp_Pnt& p0 = m_points[static_cast<size_t>(m_sel)];
        const glm::vec3 anchor(p0.X(), p0.Y(), p0.Z());
        const glm::vec3 n = glm::normalize(vp.camTarget - vp.camPos);
        const glm::vec3 dir = glm::normalize(vp.rayDir);
        const float denom = glm::dot(dir, n);
        if (std::abs(denom) > 1e-6f) {
            const float t = glm::dot(anchor - vp.rayOrigin, n) / denom;
            if (t > 0.0f) {
                const glm::vec3 q = vp.rayOrigin + dir * t;
                m_points[static_cast<size_t>(m_sel)] = gp_Pnt(q.x, q.y, q.z);
                writeParams(ctx);
                if (StudioSplineOp* o = op(ctx)) {
                    materializr::BodyChangeScope tb(ctx.doc, ctx.markBodyDirty, ctx.markMeshesDirty);
                    o->execute(ctx.doc);
                }
            }
        }
    }

    if (!(m_pressed && vp.released)) return;
    m_pressed = false;
    if (m_draggingPoint) {
        m_draggingPoint = false;
        setDraggingHandle(false);
        update(ctx);   // replay downstream once
        return;
    }
    if (m_moved || vp.uiCaptured) return;   // an orbit, or a panel tap
    if (m_pressPoint >= 0) {                 // tap on a point: pick it
        m_sel = (m_sel == m_pressPoint) ? -1 : m_pressPoint;
        return;
    }
    // Tap on the curve between points: insert one there.
    const TopoDS_Shape w = StudioSplineOp::buildCurve(m_points, m_closed, gp_Vec(), gp_Vec(), m_conds);
    if (w.IsNull()) return;
    TopExp_Explorer ex(w, TopAbs_EDGE);
    if (!ex.More()) return;
    double f, l;
    Handle(Geom_Curve) c = BRep_Tool::Curve(TopoDS::Edge(ex.Current()), f, l);
    if (c.IsNull()) return;
    float bd = pick;
    gp_Pnt hit;
    bool found = false;
    for (int i = 0; i <= 600; ++i) {
        const gp_Pnt q = c->Value(f + (l - f) * i / 600.0);
        glm::vec2 sp;
        if (!vp.toScreen(glm::vec3(q.X(), q.Y(), q.Z()), sp)) continue;
        const float d = glm::length(sp - vp.mouse);
        if (d < bd) { bd = d; hit = q; found = true; }
    }
    if (!found) return;
    // Between the neighbours whose chord passes closest to the hit.
    size_t at = 1;
    double best = 1e300;
    const size_t segs = m_closed ? m_points.size() : m_points.size() - 1;
    for (size_t i = 0; i < segs; ++i) {
        const gp_Pnt& a = m_points[i];
        const gp_Pnt& b = m_points[(i + 1) % m_points.size()];
        const double d = hit.Distance(a) + hit.Distance(b) - a.Distance(b);
        if (d < best) { best = d; at = i + 1; }
    }
    m_points.insert(m_points.begin() + static_cast<long>(at), hit);
    if (m_conds.size() < m_points.size() - 1) m_conds.resize(m_points.size() - 1);
    m_conds.insert(m_conds.begin() + static_cast<long>(at), StudioSplineOp::PointCond());
    m_sel = static_cast<int>(at);
    update(ctx);
}

void StudioSplineEditController::panelBody(const IopContext& ctx, bool& changed) {
    (void)ctx;
    const float h = 40.0f * uiScale();
    if (m_conds.size() < m_points.size()) m_conds.resize(m_points.size());
    if (m_sel >= 0 && m_sel < static_cast<int>(m_points.size())) {
        auto& c = m_conds[static_cast<size_t>(m_sel)];
        ImGui::Text(materializr::tr("Point %d"), m_sel + 1);
        static const char* kNames[3] = {"Free", "G1", "G2"};
        for (int k = 0; k < 3; ++k) {
            if (k) ImGui::SameLine();
            const bool on = static_cast<int>(c.type) == k;
            if (on) ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
            if (ImGui::Button(kNames[k], ImVec2(64.0f * uiScale(), h)) && !on) {
                if (k == 0) {
                    c = StudioSplineOp::PointCond();
                } else {
                    // Lock what the curve does there now.
                    gp_Vec dir, curv;
                    if (curveFrameAt(m_points, m_closed, m_conds,
                                     m_points[static_cast<size_t>(m_sel)], dir, curv)) {
                        c.type = (k == 1) ? StudioSplineOp::Cond::G1 : StudioSplineOp::Cond::G2;
                        c.dir = dir;
                        c.curv = (k == 2) ? curv : gp_Vec();
                    }
                }
                changed = true;
            }
            if (on) ImGui::PopStyleColor();
        }
        ImGui::BeginDisabled(m_points.size() <= 2);
        if (ImGui::Button(materializr::tr("Delete point"), ImVec2(-1.0f, h))) {
            m_points.erase(m_points.begin() + m_sel);
            m_conds.erase(m_conds.begin() + m_sel);
            m_sel = -1;
            changed = true;
        }
        ImGui::EndDisabled();
    } else {
        ImGui::Text(materializr::tr("Points: %d"), static_cast<int>(m_points.size()));
    }
    bool anyCond = false;
    for (const auto& c : m_conds) anyCond |= (c.type != StudioSplineOp::Cond::Free);
    ImGui::BeginDisabled(!anyCond);
    if (ImGui::Button(materializr::tr("Unlink"), ImVec2(-1.0f, h))) {
        for (auto& c : m_conds) c = StudioSplineOp::PointCond();
        changed = true;
    }
    ImGui::EndDisabled();
    if (ImGui::Checkbox(materializr::tr("Closed"), &m_closed)) changed = true;
}

void StudioSplineEditController::drawOverlay(const IopOverlay& ov) const {
    if (!active() || !ov.toScreen || !ov.disc) return;
    const float s = uiScale();
    for (size_t i = 0; i < m_points.size(); ++i) {
        glm::vec2 sp;
        const gp_Pnt& p = m_points[i];
        if (!ov.toScreen(glm::vec3(p.X(), p.Y(), p.Z()), sp)) continue;
        const bool sel = static_cast<int>(i) == m_sel;
        const bool cond = i < m_conds.size() && m_conds[i].type != StudioSplineOp::Cond::Free;
        const float r = (sel ? 7.0f : 5.0f) * s;
        ov.disc(sp, r + 1.5f * s, 0xE61E1614u);
        ov.disc(sp, r, sel ? 0xFF1AD8FFu : cond ? 0xFFFFB050u : 0xFFFAF0EBu);
    }
}

void StudioSplineEditController::onCleanup() {
    m_step = -1;
    m_before.clear();
    m_points.clear();
    m_conds.clear();
    m_sel = -1;
    m_pressed = m_moved = m_draggingPoint = false;
    m_pressPoint = -1;
    m_preview.clear();
}

} // namespace materializr
