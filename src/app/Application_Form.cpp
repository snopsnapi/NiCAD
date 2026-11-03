// Fork: Form workspace glue - showing forms, picking them, opening Edit Form.
#include "app/Application.h"
#include "core/Document.h"
#include "core/History.h"
#include "core/SelectionManager.h"
#include "modeling/Form.h"
#include "modeling/FormOp.h"
#include "viewport/Camera.h"
#include "viewport/FormRenderer.h"
#include "viewport/Viewport.h"
#include <functional>

namespace materializr {

void Application::refreshFormMeshes() {
    if (!m_document || !m_formRenderer) return;
    size_t sig = 1469598103934665603ull;
    auto mix = [&](size_t v) { sig = (sig ^ v) * 1099511628211ull; };
    const int sel = selectedFormId();
    std::map<int, FormMeshCache> next;
    for (int id : m_document->getAllFormIds()) {
        if (!m_document->isFormVisible(id)) continue;
        const std::string* text = m_document->getForm(id);
        if (!text) continue;
        const size_t h = std::hash<std::string>{}(*text);
        auto it = m_formMeshes.find(id);
        if (it != m_formMeshes.end() && it->second.hash == h) {
            next[id] = std::move(it->second);
        } else {
            form::Cage c;
            FormMeshCache e;
            e.hash = h;
            if (c.deserialize(*text)) e.tris = form::displayTriangles(c, 3);
            next[id] = std::move(e);
        }
        mix(static_cast<size_t>(id));
        mix(h);
        mix(id == sel ? 1 : 0);
    }
    m_formMeshes = std::move(next);
    if (sig == m_formSceneSig) return;
    m_formSceneSig = sig;
    std::vector<FormRenderer::Mesh> meshes;
    for (const auto& [id, e] : m_formMeshes) {
        FormRenderer::Mesh m;
        m.vertices = e.tris;
        m.color = (id == sel) ? glm::vec3(0.95f, 0.80f, 0.40f) : glm::vec3(0.80f, 0.83f, 0.90f);
        meshes.push_back(std::move(m));
    }
    m_formRenderer->setMeshes(meshes);
}

bool Application::pickForm(float x, float y, float vpW, float vpH, int& formId) const {
    if (!m_viewport || vpW <= 0.0f || vpH <= 0.0f || m_formMeshes.empty()) return false;
    const Camera& cam = m_viewport->getCamera();
    const glm::mat4 inv = glm::inverse(cam.getProjectionMatrix() * cam.getViewMatrix());
    const float nx = x / vpW * 2.0f - 1.0f, ny = 1.0f - y / vpH * 2.0f;
    glm::vec4 a = inv * glm::vec4(nx, ny, -1.0f, 1.0f);
    glm::vec4 b = inv * glm::vec4(nx, ny, 1.0f, 1.0f);
    a /= a.w;
    b /= b.w;
    const gp_Pnt o(a.x, a.y, a.z);
    const gp_Vec d(b.x - a.x, b.y - a.y, b.z - a.z);
    double best = 1e300;
    bool hit = false;
    for (const auto& [id, e] : m_formMeshes) {
        double t = 0;
        if (form::rayHit(e.tris, o, d, t) && t < best) { best = t; formId = id; hit = true; }
    }
    return hit;
}

int Application::formStepForId(int formId) const {
    if (!m_history || formId < 0) return -1;
    const int upTo = m_history->currentStep();
    for (int st = 0; st <= upTo; ++st) {
        const auto* op = dynamic_cast<const FormOp*>(m_history->getStep(st));
        if (op && op->isEnabled() && op->formId() == formId) return st;
    }
    return -1;
}

int Application::selectedFormId() const {
    if (!m_selection) return -1;
    for (const auto& e : m_selection->getSelection())
        if (e.type == SelectionType::Form) return e.bodyId;
    return -1;
}

bool Application::beginEditForm(int step) {
    cancelAllInteractivePreviews();
    m_editFormCtl.onCommitted = [this](int st, const std::string& before) {
        recordParamEdit({{st, before}});
        markDirty();
    };
    const bool ok = m_editFormCtl.beginEdit(iopContext(), step);
    m_meshesDirty = true;
    return ok;
}

// ── Origin ──────────────────────────────────────────────────────────────────

bool Application::originVisible() const {
    if (m_inSketchMode) return false;
    if (m_showOrigin) return true;
    return m_pendingCmd.active && pendingAllows(SelectionType::OriginPlane);
}

float Application::originHalfSize() const {
    if (!m_viewport) return 20.0f;
    const glm::vec3 eye = m_viewport->getCamera().getPosition();
    return std::max(0.5f, glm::length(eye) * 0.05f);
}

bool Application::pickOriginPlane(float x, float y, float vpW, float vpH, int& which,
                                  double& tOut) const {
    if (!m_viewport || vpW <= 0.0f || vpH <= 0.0f) return false;
    const Camera& cam = m_viewport->getCamera();
    const glm::mat4 inv = glm::inverse(cam.getProjectionMatrix() * cam.getViewMatrix());
    const float nx = x / vpW * 2.0f - 1.0f, ny = 1.0f - y / vpH * 2.0f;
    glm::vec4 a = inv * glm::vec4(nx, ny, -1.0f, 1.0f);
    glm::vec4 b = inv * glm::vec4(nx, ny, 1.0f, 1.0f);
    a /= a.w;
    b /= b.w;
    const glm::vec3 o(a), d = glm::normalize(glm::vec3(b) - glm::vec3(a));
    const float h = originHalfSize();
    // XY: normal Y (world), XZ: normal Z, YZ: normal X.
    const int normalAxis[3] = {1, 2, 0};
    bool hit = false;
    double best = 1e300;
    for (int k = 0; k < 3; ++k) {
        const int n = normalAxis[k];
        if (std::abs(d[n]) < 1e-6f) continue;
        const float t = -o[n] / d[n];
        if (t <= 0.0f) continue;
        const glm::vec3 p = o + d * t;
        bool inside = true;
        for (int c = 0; c < 3; ++c)
            if (c != n && std::abs(p[c]) > h) inside = false;
        if (inside && t < best) { best = t; which = k; hit = true; }
    }
    if (hit) tOut = best;
    return hit;
}

} // namespace materializr
