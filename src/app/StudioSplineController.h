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
