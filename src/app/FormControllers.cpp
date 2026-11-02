#include "FormControllers.h"
#include "../core/BodyChanges.h"
#include "../core/Document.h"
#include "../core/History.h"
#include "../core/SelectionManager.h"
#include "../modeling/FormOp.h"
#include "../ui/LengthField.h"
#include "../ui_scale.h"
#include <imgui.h>
#include <algorithm>
#include <cmath>
#include "../i18n.h"

namespace materializr {

// ── Primitives ──────────────────────────────────────────────────────────────

const char* FormPrimitiveController::title() const {
    switch (m_kind) {
    case Kind::Plane: return "Plane";
    case Kind::Cylinder: return "Cylinder";
    case Kind::Sphere: return "Sphere";
    default: return "Box";
    }
}

bool FormPrimitiveController::beginPrimitive(const IopContext& ctx, Kind kind) {
    m_kind = kind;
    return begin(ctx);
}

int FormPrimitiveController::onBegin(const IopContext& ctx) {
    ctx.selection.clear();
    return kNoTargetBody;   // the form is not a body
}

std::string FormPrimitiveController::cageText() const {
    const double s = std::max(0.1f, m_size);
    switch (m_kind) {
    case Kind::Plane: return form::plane(s, s, m_segs, m_segs).serialize();
    case Kind::Cylinder:
        return form::cylinder(s * 0.5, std::max(0.1f, m_height), std::max(3, m_segs * 4),
                              m_segs).serialize();
    case Kind::Sphere: return form::sphere(s * 0.5, m_segs).serialize();
    default: return form::box(s, s, s, m_segs).serialize();
    }
}

std::unique_ptr<Operation> FormPrimitiveController::buildOp(const IopContext&) {
    auto op = std::make_unique<FormOp>();
    op->setCage(cageText());
    return op;
}

bool FormPrimitiveController::syncLiveOp(Operation& op) {
    static_cast<FormOp&>(op).setCage(cageText());
    return true;
}

void FormPrimitiveController::panelBody(const IopContext& ctx, bool& changed) {
    const char* sizeLabel = (m_kind == Kind::Box || m_kind == Kind::Plane)
                                ? materializr::tr("Size") : materializr::tr("Diameter");
    if (ctx.cornerCommitUi) {
        if (materializr::amountLengthField("formSize", sizeLabel, &m_size, false, 0.1f, 10000.0f))
            changed = true;
        if (m_kind == Kind::Cylinder &&
            materializr::amountLengthField("formH", materializr::tr("Height"), &m_height,
                                           false, 0.1f, 10000.0f))
            changed = true;
    } else {
        if (materializr::lengthField(sizeLabel, &m_size)) changed = true;
        if (m_kind == Kind::Cylinder && materializr::lengthField(materializr::tr("Height"), &m_height))
            changed = true;
    }
    const float h = 40.0f * uiScale();
    ImGui::Text(materializr::tr("Segments: %d"), m_segs);
    ImGui::SameLine();
    if (ImGui::Button("-", ImVec2(h, h)) && m_segs > 1) { --m_segs; changed = true; }
    ImGui::SameLine();
    if (ImGui::Button("+", ImVec2(h, h)) && m_segs < 8) { ++m_segs; changed = true; }
}

// ── Edit Form ───────────────────────────────────────────────────────────────

namespace {

glm::vec3 g(const gp_Pnt& p) { return glm::vec3(p.X(), p.Y(), p.Z()); }
glm::vec3 g(const gp_Vec& v) { return glm::vec3(v.X(), v.Y(), v.Z()); }

float segDist(glm::vec2 p, glm::vec2 a, glm::vec2 b) {
    const glm::vec2 ab = b - a;
    const float l2 = glm::dot(ab, ab);
    if (l2 < 1e-6f) return glm::length(p - a);
    const float t = std::clamp(glm::dot(p - a, ab) / l2, 0.0f, 1.0f);
    return glm::length(p - (a + ab * t));
}

bool insidePoly(glm::vec2 p, const std::vector<glm::vec2>& poly) {
    bool in = false;
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        const glm::vec2 a = poly[i], b = poly[j];
        if (((a.y > p.y) != (b.y > p.y)) &&
            (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y + 1e-12f) + a.x))
            in = !in;
    }
    return in;
}

} // namespace

::FormOp* EditFormController::op(const IopContext& ctx) const {
    return dynamic_cast<::FormOp*>(const_cast<Operation*>(ctx.history.getStep(m_step)));
}

bool EditFormController::beginEdit(const IopContext& ctx, int historyIndex) {
    m_step = historyIndex;
    if (!op(ctx)) { m_step = -1; return false; }
    if (!begin(ctx)) { m_step = -1; return false; }
    return true;
}

int EditFormController::onBegin(const IopContext& ctx) {
    ::FormOp* o = op(ctx);
    if (!o || !m_cage.deserialize(o->cage())) return -1;
    m_before = o->serializeParams();
    clearSelection();
    m_preview.begin(ctx.doc, ctx.history);
    ctx.selection.clear();
    return kNoTargetBody;
}

void EditFormController::writeLive(const IopContext& ctx) {
    ::FormOp* o = op(ctx);
    if (!o) return;
    o->setCage(m_cage.serialize());
    o->execute(ctx.doc);   // just the form table; downstream replays on lift
}

void EditFormController::update(const IopContext& ctx) {
    if (!active()) return;
    materializr::BodyChangeScope trackBodies(ctx.doc, ctx.markBodyDirty, ctx.markMeshesDirty);
    if (::FormOp* o = op(ctx)) o->setCage(m_cage.serialize());
    setPreviewOk(m_preview.replay(m_step, ctx.doc, ctx.history));
}

void EditFormController::commit(const IopContext& ctx) {
    if (!active()) { teardown(); return; }
    materializr::BodyChangeScope trackBodies(ctx.doc, ctx.markBodyDirty, ctx.markMeshesDirty);
    const int step = m_step;
    const std::string before = m_before;
    if (::FormOp* o = op(ctx)) o->setCage(m_cage.serialize());
    if (m_preview.replay(m_step, ctx.doc, ctx.history)) {
        m_preview.clear();
        teardown();
        if (onCommitted) onCommitted(step, before);
        return;
    }
    if (::FormOp* o = op(ctx)) o->deserializeParams(before);
    m_preview.replay(m_step, ctx.doc, ctx.history);
    if (ctx.toast)
        ctx.toast(materializr::tr("A later step can't follow this edit - the form went back."));
    m_preview.clear();
    teardown();
}

void EditFormController::cancel(const IopContext& ctx) {
    if (active()) {
        materializr::BodyChangeScope trackBodies(ctx.doc, ctx.markBodyDirty, ctx.markMeshesDirty);
        if (::FormOp* o = op(ctx)) o->deserializeParams(m_before);
        m_preview.replay(m_step, ctx.doc, ctx.history);
    }
    m_preview.clear();
    teardown();
}

std::vector<int> EditFormController::selectedVerts() const {
    std::set<int> v(m_selV.begin(), m_selV.end());
    for (const form::Edge& e : m_selE) { v.insert(e.first); v.insert(e.second); }
    for (int f : m_selF)
        if (f >= 0 && f < static_cast<int>(m_cage.faces.size()))
            for (int x : m_cage.faces[static_cast<size_t>(f)]) v.insert(x);
    std::vector<int> out;
    for (int x : v) if (x >= 0 && x < static_cast<int>(m_cage.verts.size())) out.push_back(x);
    return out;
}

bool EditFormController::frame(glm::vec3& o, glm::vec3 ax[3]) const {
    const std::vector<int> vs = selectedVerts();
    if (vs.empty()) return false;
    glm::vec3 c(0.0f);
    for (int v : vs) c += g(m_cage.verts[static_cast<size_t>(v)]);
    o = c / static_cast<float>(vs.size());
    ax[0] = glm::vec3(1, 0, 0);
    ax[1] = glm::vec3(0, 1, 0);
    ax[2] = glm::vec3(0, 0, 1);
    if (!m_local) return true;
    // Local: Z along the selection's normal - "90 degrees to the surface".
    gp_Vec n(0, 0, 0);
    for (int f : m_selF) n += m_cage.faceNormal(f);
    if (n.Magnitude() < 1e-9)
        for (int v : vs) n += m_cage.vertexNormal(v);
    if (n.Magnitude() < 1e-9) return true;
    const glm::vec3 z = glm::normalize(g(n));
    glm::vec3 x = glm::vec3(1, 0, 0) - z * glm::dot(z, glm::vec3(1, 0, 0));
    if (glm::length(x) < 1e-3f) x = glm::vec3(0, 0, 1) - z * glm::dot(z, glm::vec3(0, 0, 1));
    x = glm::normalize(x);
    ax[0] = x;
    ax[1] = glm::normalize(glm::cross(z, x));
    ax[2] = z;
    return true;
}

float EditFormController::handleLength() const {
    glm::vec3 o, ax[3];
    if (!frame(o, ax)) return 1.0f;
    return std::max(0.01f, glm::length(m_camPos - o) * 0.15f);
}

// 0..2 arrows, 3..5 plane squares (by their normal axis), 6 centre disc.
int EditFormController::pickHandle(const IopViewport& vp) const {
    glm::vec3 o, ax[3];
    if (!frame(o, ax)) return -1;
    const float L = handleLength();
    const float r = 24.0f * uiScale();
    glm::vec2 so;
    if (!vp.toScreen(o, so)) return -1;
    if (glm::length(vp.mouse - so) < 16.0f * uiScale()) return 6;
    for (int k = 0; k < 3; ++k) {
        // Plane square between axes (k+1)%3 and (k+2)%3, normal k.
        const glm::vec3 a = ax[(k + 1) % 3], b = ax[(k + 2) % 3];
        std::vector<glm::vec2> q(4);
        const glm::vec3 c[4] = {o + (a + b) * (0.22f * L), o + (a * 0.42f + b * 0.22f) * L,
                                o + (a + b) * (0.42f * L), o + (a * 0.22f + b * 0.42f) * L};
        bool ok = true;
        for (int i = 0; i < 4; ++i) ok &= vp.toScreen(c[i], q[static_cast<size_t>(i)]);
        if (ok && insidePoly(vp.mouse, q)) return 3 + k;
    }
    int best = -1;
    float bd = r;
    for (int k = 0; k < 3; ++k) {
        glm::vec2 st;
        if (!vp.toScreen(o + ax[k] * L, st)) continue;
        const float d = segDist(vp.mouse, so, st);
        if (d < bd) { bd = d; best = k; }
    }
    return best;
}

void EditFormController::pickElement(const IopViewport& vp) {
    const float s = uiScale();
    // Vertex first, then edge, then face.
    int bestV = -1;
    float bd = 20.0f * s;
    std::vector<glm::vec2> sp(m_cage.verts.size());
    std::vector<char> ok(m_cage.verts.size(), 0);
    for (size_t i = 0; i < m_cage.verts.size(); ++i) {
        ok[i] = vp.toScreen(g(m_cage.verts[i]), sp[i]);
        if (!ok[i]) continue;
        const float d = glm::length(sp[i] - vp.mouse);
        if (d < bd) { bd = d; bestV = static_cast<int>(i); }
    }
    if (bestV >= 0) {
        if (!m_selV.erase(bestV)) m_selV.insert(bestV);
        return;
    }
    form::Edge bestE{-1, -1};
    bd = 14.0f * s;
    for (const form::Edge& e : m_cage.edges()) {
        const size_t a = static_cast<size_t>(e.first), b = static_cast<size_t>(e.second);
        if (!ok[a] || !ok[b]) continue;
        const float d = segDist(vp.mouse, sp[a], sp[b]);
        if (d < bd) { bd = d; bestE = e; }
    }
    if (bestE.first >= 0) {
        if (!m_selE.erase(bestE)) m_selE.insert(bestE);
        return;
    }
    // Faces under the finger: the one nearest the camera.
    int bestF = -1;
    float bestDepth = 1e30f;
    for (size_t f = 0; f < m_cage.faces.size(); ++f) {
        std::vector<glm::vec2> poly;
        bool all = true;
        for (int v : m_cage.faces[f]) {
            if (!ok[static_cast<size_t>(v)]) { all = false; break; }
            poly.push_back(sp[static_cast<size_t>(v)]);
        }
        if (!all || !insidePoly(vp.mouse, poly)) continue;
        const float depth = glm::length(g(m_cage.faceCentre(static_cast<int>(f))) - m_camPos);
        if (depth < bestDepth) { bestDepth = depth; bestF = static_cast<int>(f); }
    }
    if (bestF >= 0) {
        if (!m_selF.erase(bestF)) m_selF.insert(bestF);
        return;
    }
    clearSelection();   // a tap on nothing
}

void EditFormController::onViewportInput(const IopViewport& vp, const IopContext& ctx) {
    if (!active()) return;
    m_camPos = vp.camPos;
    if (vp.clicked) {
        m_pressed = true;
        m_moved = false;
        m_pressAt = vp.mouse;
        m_handle = pickHandle(vp);
        setDraggingHandle(m_handle >= 0);
        if (m_handle >= 0) {
            m_dragStart = m_cage;
            frame(m_frameO, m_frameAx);
            m_axisTravel = 0.0f;
            if (m_handle >= 3 && m_handle <= 5) {
                const glm::vec3 n = m_frameAx[m_handle - 3];
                const float den = glm::dot(vp.rayDir, n);
                m_planeStart = std::abs(den) > 1e-6f
                    ? vp.rayOrigin + vp.rayDir * (glm::dot(m_frameO - vp.rayOrigin, n) / den)
                    : m_frameO;
            }
        }
    }
    if (m_pressed && glm::length(vp.mouse - m_pressAt) > 12.0f * uiScale()) m_moved = true;

    if (m_handle >= 0 && vp.down && (vp.mouseDelta.x != 0.0f || vp.mouseDelta.y != 0.0f)) {
        gp_Trsf t;
        if (m_handle <= 2) {
            m_axisTravel += vp.dragAlongAxis(m_frameO, m_frameAx[m_handle], vp.mouseDelta);
            const glm::vec3 d = m_frameAx[m_handle] * m_axisTravel;
            t.SetTranslation(gp_Vec(d.x, d.y, d.z));
        } else if (m_handle <= 5) {
            const glm::vec3 n = m_frameAx[m_handle - 3];
            const float den = glm::dot(vp.rayDir, n);
            if (std::abs(den) > 1e-6f) {
                const glm::vec3 hit =
                    vp.rayOrigin + vp.rayDir * (glm::dot(m_frameO - vp.rayOrigin, n) / den);
                const glm::vec3 d = hit - m_planeStart;
                t.SetTranslation(gp_Vec(d.x, d.y, d.z));
            }
        } else {
            const float f = std::exp((m_pressAt.y - vp.mouse.y) * 0.005f);
            t.SetScale(gp_Pnt(m_frameO.x, m_frameO.y, m_frameO.z), std::clamp(f, 0.02f, 50.0f));
        }
        m_cage = m_dragStart;
        m_cage.transformVertices(selectedVerts(), t);
        writeLive(ctx);
    }

    if (!(m_pressed && vp.released)) return;
    m_pressed = false;
    if (m_handle >= 0) {
        m_handle = -1;
        setDraggingHandle(false);
        update(ctx);   // downstream steps follow once, on lift
        return;
    }
    if (m_moved || vp.uiCaptured) return;
    pickElement(vp);
}

void EditFormController::panelBody(const IopContext& ctx, bool& changed) {
    const float s = uiScale();
    const float h = 40.0f * s;
    const float half = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f;
    // World / Local frame.
    for (int k = 0; k < 2; ++k) {
        if (k) ImGui::SameLine();
        const bool on = (m_local == (k == 1));
        if (on) ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        if (ImGui::Button(materializr::tr(k ? "Local" : "World"), ImVec2(half, h))) m_local = (k == 1);
        if (on) ImGui::PopStyleColor();
    }
    ImGui::Text(materializr::tr("%d vertices, %d edges, %d faces"),
                static_cast<int>(m_selV.size()), static_cast<int>(m_selE.size()),
                static_cast<int>(m_selF.size()));
    auto edit = [&](bool after) {
        if (!after) return;
        changed = true;
    };
    ImGui::BeginDisabled(m_selE.empty());
    if (ImGui::Button(materializr::tr("Crease"), ImVec2(half, h))) {
        m_cage.setCrease(std::vector<form::Edge>(m_selE.begin(), m_selE.end()), true);
        edit(true);
    }
    ImGui::SameLine();
    if (ImGui::Button(materializr::tr("Uncrease"), ImVec2(half, h))) {
        m_cage.setCrease(std::vector<form::Edge>(m_selE.begin(), m_selE.end()), false);
        edit(true);
    }
    if (ImGui::Button(materializr::tr("Insert Edge"), ImVec2(-1.0f, h))) {
        m_cage.insertEdgeLoop(*m_selE.begin());
        clearSelection();
        edit(true);
    }
    ImGui::EndDisabled();
    ImGui::BeginDisabled(m_selF.empty());
    if (ImGui::Button(materializr::tr("Subdivide"), ImVec2(half, h))) {
        // Highest index first: subdivideFace keeps lower indices in place.
        for (auto it = m_selF.rbegin(); it != m_selF.rend(); ++it) m_cage.subdivideFace(*it);
        clearSelection();
        edit(true);
    }
    ImGui::SameLine();
    if (ImGui::Button(materializr::tr("Extrude"), ImVec2(half, h))) {
        // A short first step along the faces' normal; drag the arrow from
        // there (the moved faces stay selected, Local frame on).
        double len = 0.0;
        const auto es = m_cage.edges();
        for (const auto& e : es)
            len += m_cage.verts[static_cast<size_t>(e.first)].Distance(
                m_cage.verts[static_cast<size_t>(e.second)]);
        len = es.empty() ? 1.0 : len / static_cast<double>(es.size());
        const auto moved = m_cage.extrudeFaces(std::vector<int>(m_selF.begin(), m_selF.end()),
                                               0.3 * len);
        m_selV.clear();
        m_selE.clear();
        m_selF = std::set<int>(moved.begin(), moved.end());
        m_local = true;
        edit(true);
    }
    if (ImGui::Button(materializr::tr("Delete faces"), ImVec2(-1.0f, h))) {
        if (m_selF.size() < m_cage.faces.size()) {
            m_cage.deleteFaces(std::vector<int>(m_selF.begin(), m_selF.end()));
            clearSelection();
            edit(true);
        }
    }
    ImGui::EndDisabled();
    (void)ctx;
}

void EditFormController::drawOverlay(const IopOverlay& ov) const {
    if (!active() || !ov.toScreen || !ov.line || !ov.disc) return;
    const float s = uiScale();
    std::vector<glm::vec2> sp(m_cage.verts.size());
    std::vector<char> ok(m_cage.verts.size(), 0);
    for (size_t i = 0; i < m_cage.verts.size(); ++i) ok[i] = ov.toScreen(g(m_cage.verts[i]), sp[i]);
    // Cage edges: grey, creases orange, picked yellow; picked faces outlined.
    std::set<form::Edge> faceEdges;
    for (int f : m_selF)
        if (f >= 0 && f < static_cast<int>(m_cage.faces.size())) {
            const auto& fv = m_cage.faces[static_cast<size_t>(f)];
            for (size_t i = 0; i < fv.size(); ++i)
                faceEdges.insert(form::edgeKey(fv[i], fv[(i + 1) % fv.size()]));
        }
    for (const form::Edge& e : m_cage.edges()) {
        const size_t a = static_cast<size_t>(e.first), b = static_cast<size_t>(e.second);
        if (!ok[a] || !ok[b]) continue;
        const bool sel = m_selE.count(e) || faceEdges.count(e);
        const bool cr = m_cage.creases.count(e) != 0;
        ov.line(sp[a], sp[b], sel ? 0xFF1AD8FFu : cr ? 0xFF2C8CFFu : 0xC8C8BEB4u,
                (sel ? 3.0f : cr ? 2.5f : 1.4f) * s);
    }
    for (size_t i = 0; i < m_cage.verts.size(); ++i) {
        if (!ok[i]) continue;
        const bool sel = m_selV.count(static_cast<int>(i)) != 0;
        ov.disc(sp[i], (sel ? 6.0f : 3.5f) * s, sel ? 0xFF1AD8FFu : 0xFFF0F0F0u);
    }
    // Manipulator.
    glm::vec3 o, ax[3];
    if (!frame(o, ax)) return;
    const float L = handleLength();
    glm::vec2 so;
    if (!ov.toScreen(o, so)) return;
    const unsigned cols[3] = {0xFF3C3CE6u, 0xFF3CE63Cu, 0xFFE6823Cu};   // X red, Y green, Z blue
    for (int k = 0; k < 3; ++k) {
        glm::vec2 st;
        if (!ov.toScreen(o + ax[k] * L, st)) continue;
        const bool hot = m_handle == k;
        ov.line(so, st, cols[k], (hot ? 5.0f : 3.5f) * s);
        ov.disc(st, 6.0f * s, cols[k]);
        if (ov.triangle) {
            const glm::vec3 a = ax[(k + 1) % 3], b = ax[(k + 2) % 3];
            glm::vec2 q[4];
            const glm::vec3 c[4] = {o + (a + b) * (0.22f * L), o + (a * 0.42f + b * 0.22f) * L,
                                    o + (a + b) * (0.42f * L), o + (a * 0.22f + b * 0.42f) * L};
            bool all = true;
            for (int i = 0; i < 4; ++i) all &= ov.toScreen(c[i], q[i]);
            if (all) {
                const unsigned pc = (m_handle == 3 + k) ? cols[k] : ((cols[k] & 0x00FFFFFFu) | 0x9A000000u);
                ov.triangle(q[0], q[1], q[2], pc);
                ov.triangle(q[0], q[2], q[3], pc);
            }
        }
    }
    ov.disc(so, 9.0f * s, m_handle == 6 ? 0xFFFFFFFFu : 0xE6DCDCDCu);
}

void EditFormController::onCleanup() {
    m_step = -1;
    m_before.clear();
    m_cage = form::Cage();
    clearSelection();
    m_pressed = m_moved = false;
    m_handle = -1;
    m_preview.clear();
}

} // namespace materializr
