#pragma once
#include "InteractiveOpController.h"
#include <gp_Pnt.hxx>
#include <vector>

namespace materializr {

// Studio Spline creation (3D curve, NX Studio Spline / Fusion 3D sketch).
// Each tap adds a point: on a body it lands on the surface under the finger;
// in empty space it lands on the plane through the previous point (or the
// view target) facing the camera. The curve previews live; Undo point, Closed,
// Confirm. Afterwards the points are moved with the regular Move gizmo.
class StudioSplineController : public InteractiveOpController {
protected:
    const char* title() const override { return "Spline 3D"; }
    PreviewModel previewModel() const override { return PreviewModel::LiveOp; }
    int onBegin(const IopContext& ctx) override;
    std::unique_ptr<Operation> buildOp(const IopContext& ctx) override;
    bool syncLiveOp(Operation& op) override;
    void panelBody(const IopContext& ctx, bool& changed) override;
    void onCleanup() override;
    bool wantsViewportInput() const override { return true; }
    void onViewportInput(const IopViewport& vp, const IopContext& ctx) override;
    void drawOverlay(const IopOverlay& ov) const override;

private:
    std::vector<gp_Pnt> m_points;
    bool m_closed = false;
    glm::vec2 m_pressAt{0.0f};
    bool m_pressed = false;
    bool m_moved = false;
};

} // namespace materializr

#include "HistoryEditPreview.h"
#include "../modeling/StudioSplineOp.h"
#include <functional>
#include <string>

namespace materializr {

// Studio Spline editor (NX "edit Studio Spline"): re-opens the step - by a
// double tap on the curve or a tap on its timeline step. Tap a point to pick
// it, drag a picked point to move it (in the plane through it facing the
// camera), tap the curve between points to insert one. Panel: the picked
// point's continuity (Free / G1 / G2 locking the curve's current tangent /
// curvature there), Delete point, Unlink (drop every condition), Closed.
// The preview is the history replay from the step (HistoryEdit); Cancel
// puts the parameters back.
class StudioSplineEditController : public InteractiveOpController {
public:
    bool beginEdit(const IopContext& ctx, int historyIndex);
    // Fired after a successful commit with the step and its params before
    // the edit - the app records it for Undo.
    std::function<void(int step, const std::string& before)> onCommitted;

    void update(const IopContext& ctx) override;
    void commit(const IopContext& ctx) override;
    void cancel(const IopContext& ctx) override;

protected:
    const char* title() const override { return "Studio Spline"; }
    PreviewModel previewModel() const override { return PreviewModel::HistoryEdit; }
    int onBegin(const IopContext& ctx) override;
    std::unique_ptr<Operation> buildOp(const IopContext&) override { return nullptr; }
    void panelBody(const IopContext& ctx, bool& changed) override;
    void onCleanup() override;
    bool wantsViewportInput() const override { return true; }
    void onViewportInput(const IopViewport& vp, const IopContext& ctx) override;
    void drawOverlay(const IopOverlay& ov) const override;

private:
    StudioSplineOp* op(const IopContext& ctx) const;
    void writeParams(const IopContext& ctx) const;
    void finish(const IopContext& ctx);

    int m_step = -1;
    std::string m_before;
    std::vector<gp_Pnt> m_points;
    std::vector<StudioSplineOp::PointCond> m_conds;
    bool m_closed = false;
    int m_sel = -1;
    HistoryEditPreview m_preview;
    // gesture
    bool m_pressed = false, m_moved = false, m_draggingPoint = false;
    glm::vec2 m_pressAt{0.0f};
    int m_pressPoint = -1;
};

} // namespace materializr
