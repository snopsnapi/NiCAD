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
#include "ui/TouchTheme.h"
#include "ui/Toolbar.h"
#include "ui_scale.h"
#include <BRepAdaptor_Curve.hxx>
#include <BRepLProp_CLProps.hxx>
#include <BRep_Tool.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomLProp_SLProps.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Surface.hxx>
#include <TopoDS.hxx>
#include <imgui.h>
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
    m_curveDragBefore.clear();
    if (!m_selection) return;
    const std::vector<CurvePointRef> all = curvePoints();
    for (const auto& e : m_selection->getSelection()) {
        if (e.type != SelectionType::CurvePoint) continue;
        for (const CurvePointRef& r : all)
            if (r.bodyId == e.bodyId && r.index == e.subShapeIndex) {
                m_curveDrag.push_back({r.step, r.index, gp_Pnt(r.p.x, r.p.y, r.p.z)});
                bool have = false;
                for (const auto& b : m_curveDragBefore) if (b.first == r.step) have = true;
                if (!have)
                    if (const Operation* op = m_history->getStep(r.step))
                        m_curveDragBefore.push_back({r.step, op->serializeParams()});
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
    } else {
        recordParamEdit(m_curveDragBefore);
    }
    m_curveDrag.clear();
    m_curveDragBefore.clear();
    markDirty();
}

// ── Parameter-edit undo ────────────────────────────────────────────────────

void Application::recordParamEdit(const std::vector<std::pair<int, std::string>>& before) {
    if (before.empty() || !m_history) return;
    ParamEdit e;
    e.before = before;
    for (const auto& [step, blob] : before)
        if (const Operation* op = m_history->getStep(step))
            e.after.push_back({step, op->serializeParams()});
    e.stepCount = m_history->stepCount();
    e.current = m_history->currentStep();
    m_paramUndo.push_back(std::move(e));
    m_paramRedo.clear();
    if (m_paramUndo.size() > 64) m_paramUndo.erase(m_paramUndo.begin());
}

bool Application::paramUndoAvailable() const {
    return m_history && !m_paramUndo.empty() &&
           m_paramUndo.back().stepCount == m_history->stepCount() &&
           m_paramUndo.back().current == m_history->currentStep();
}

bool Application::paramRedoAvailable() const {
    return m_history && !m_paramRedo.empty() &&
           m_paramRedo.back().stepCount == m_history->stepCount() &&
           m_paramRedo.back().current == m_history->currentStep();
}

namespace {
// Write each step's params and replay from the earliest one.
bool applyParams(History& hist, Document& doc,
                 const std::vector<std::pair<int, std::string>>& params) {
    int first = -1;
    for (const auto& [step, blob] : params) {
        Operation* op = const_cast<Operation*>(hist.getStep(step));
        if (!op) continue;
        op->deserializeParams(blob);
        first = (first < 0) ? step : std::min(first, step);
    }
    return first >= 0 && hist.editStep(first, doc, /*transactional=*/true);
}
} // namespace

void Application::paramUndo() {
    if (!paramUndoAvailable()) return;
    ParamEdit e = m_paramUndo.back();
    m_paramUndo.pop_back();
    auto trackBodies = trackBodyChanges();
    applyParams(*m_history, *m_document, e.before);
    m_paramRedo.push_back(std::move(e));
    markDirty();
}

void Application::paramRedo() {
    if (!paramRedoAvailable()) return;
    ParamEdit e = m_paramRedo.back();
    m_paramRedo.pop_back();
    auto trackBodies = trackBodyChanges();
    applyParams(*m_history, *m_document, e.after);
    m_paramUndo.push_back(std::move(e));
    markDirty();
}

// ── Point conditions ────────────────────────────────────────────────────────

void Application::renderCurvePointPanel() {
    if (m_inSketchMode || !m_selection || m_pendingCmd.active || m_gizmoDragging) return;
    int body = -1, index = -1, n = 0;
    for (const auto& e : m_selection->getSelection())
        if (e.type == SelectionType::CurvePoint) { body = e.bodyId; index = e.subShapeIndex; ++n; }
    if (n != 1) return;
    const StudioSplineOp* op = nullptr;
    for (const CurvePointRef& r : curvePoints())
        if (r.bodyId == body && r.index == index) {
            op = dynamic_cast<const StudioSplineOp*>(m_history->getStep(r.step));
            break;
        }
    if (!op) return;
    const float s = uiScale();
    const ImVec2 wp = ImGui::GetWindowPos();
    ImGui::SetNextWindowPos(ImVec2(wp.x + ImGui::GetWindowWidth() - 250.0f * s,
                                   wp.y + 320.0f * s), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(230.0f * s, 0.0f));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, touchui::panelBg());
    if (ImGui::Begin("##CurvePointPanel", nullptr,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings |
                     ImGuiWindowFlags_AlwaysAutoResize)) {
        static const char* kNames[3] = {"Free", "G1", "G2"};
        const auto cond = op->condition(static_cast<size_t>(index));
        ImGui::TextColored(touchui::textPrimary(), materializr::tr("Point %d"), index + 1);
        ImGui::SameLine();
        ImGui::TextColored(touchui::textDim(), "%s", kNames[static_cast<int>(cond.type)]);
        const ImVec2 bsz(-1.0f, 40.0f * s);
        struct Btn { const char* label; int kind; unsigned mask; };
        const unsigned F = 1u << static_cast<unsigned>(SelectionType::Face);
        const unsigned E = 1u << static_cast<unsigned>(SelectionType::Edge);
        const Btn btns[4] = {{"Normal to face", 0, F}, {"Tangent to face", 1, F},
                             {"Tangent to edge", 2, E}, {"Curvature of edge", 3, E}};
        for (const Btn& b : btns) {
            if (ImGui::Button(materializr::tr(b.label), bsz)) {
                m_curveCondBody = body;
                m_curveCondIndex = index;
                m_curveCondKind = b.kind;
                PendingCommand pc;
                pc.label = b.label;
                pc.action = static_cast<int>(ToolAction::CurvePointCondition);
                pc.typeMask = b.mask;
                pc.minCount = 1;
                pc.single = true;
                startWorkspaceCommand(pc);
            }
        }
        ImGui::BeginDisabled(cond.type == StudioSplineOp::Cond::Free);
        if (ImGui::Button(materializr::tr("Free"), bsz)) {
            m_curveCondBody = body;
            m_curveCondIndex = index;
            m_curveCondKind = -1;
            applyCurvePointCondition();
        }
        ImGui::EndDisabled();
    }
    ImGui::End();
    ImGui::PopStyleColor();
}

void Application::applyCurvePointCondition() {
    if (!m_history || !m_document) return;
    int step = -1;
    gp_Pnt at;
    for (const CurvePointRef& r : curvePoints())
        if (r.bodyId == m_curveCondBody && r.index == m_curveCondIndex) {
            step = r.step;
            at = gp_Pnt(r.p.x, r.p.y, r.p.z);
            break;
        }
    auto* op = step >= 0 ? dynamic_cast<StudioSplineOp*>(
                               const_cast<Operation*>(m_history->getStep(step)))
                         : nullptr;
    if (!op) return;
    StudioSplineOp::PointCond c;
    if (m_curveCondKind >= 0) {
        TopoDS_Shape ref;
        for (const auto& e : m_selection->getSelection())
            if ((e.type == SelectionType::Face || e.type == SelectionType::Edge) &&
                !e.shape.IsNull()) { ref = e.shape; break; }
        if (ref.IsNull()) return;
        try {
            if (m_curveCondKind <= 1 && ref.ShapeType() == TopAbs_FACE) {
                const TopoDS_Face f = TopoDS::Face(ref);
                Handle(Geom_Surface) surf = BRep_Tool::Surface(f);
                GeomAPI_ProjectPointOnSurf pr(at, surf);
                if (pr.NbPoints() < 1) return;
                double u, v;
                pr.LowerDistanceParameters(u, v);
                GeomLProp_SLProps props(surf, u, v, 1, 1e-7);
                if (!props.IsNormalDefined()) return;
                gp_Vec n(props.Normal());
                if (f.Orientation() == TopAbs_REVERSED) n.Reverse();
                c.type = StudioSplineOp::Cond::G1;
                if (m_curveCondKind == 0) {
                    c.dir = n;
                } else {
                    // Tangent to the face: the curve's own run through the
                    // point, flattened into the face's tangent plane.
                    const auto& pts = op->points();
                    const size_t i = static_cast<size_t>(m_curveCondIndex);
                    const gp_Pnt a = pts[i > 0 ? i - 1 : i];
                    const gp_Pnt b = pts[i + 1 < pts.size() ? i + 1 : i];
                    gp_Vec run(a, b);
                    run -= n.Normalized() * run.Dot(n.Normalized());
                    if (run.Magnitude() < 1e-9) return;
                    c.dir = run;
                }
            } else if (m_curveCondKind >= 2 && ref.ShapeType() == TopAbs_EDGE) {
                const TopoDS_Edge e = TopoDS::Edge(ref);
                double f0, f1;
                Handle(Geom_Curve) curve = BRep_Tool::Curve(e, f0, f1);
                if (curve.IsNull()) return;
                GeomAPI_ProjectPointOnCurve pr(at, curve, f0, f1);
                const double u = pr.NbPoints() > 0 ? pr.LowerDistanceParameter()
                                                   : 0.5 * (f0 + f1);
                BRepAdaptor_Curve ac(e);
                BRepLProp_CLProps props(ac, u, 2, 1e-7);
                if (!props.IsTangentDefined()) return;
                gp_Dir t;
                props.Tangent(t);
                c.type = StudioSplineOp::Cond::G1;
                c.dir = gp_Vec(t);
                if (m_curveCondKind == 3) {
                    c.type = StudioSplineOp::Cond::G2;
                    const double k = props.Curvature();
                    if (k > 1e-9) {
                        gp_Dir nrm;
                        props.Normal(nrm);
                        c.curv = gp_Vec(nrm) * k;
                    }
                }
            } else {
                return;
            }
        } catch (...) {
            return;
        }
    }
    const std::string before = op->serializeParams();
    op->setCondition(static_cast<size_t>(m_curveCondIndex), c);
    auto trackBodies = trackBodyChanges();
    if (m_history->editStep(step, *m_document, /*transactional=*/true)) {
        recordParamEdit({{step, before}});
    } else {
        op->deserializeParams(before);
        m_history->editStep(step, *m_document, /*transactional=*/true);
        showToast(materializr::tr("This condition can't be met here - left as it was."));
    }
    // Keep the point selected so its panel stays up.
    SelectionEntry pe;
    pe.type = SelectionType::CurvePoint;
    pe.bodyId = m_curveCondBody;
    pe.subShapeIndex = m_curveCondIndex;
    m_selection->select(pe);
    markDirty();
}

} // namespace materializr
