#pragma once
#include "InteractiveOpController.h"
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <vector>

namespace materializr {

// Trim ("scissors", Surface workspace). Begun with the target body plus a
// tool - a second body or a sketch - selected. The target is shown split by
// the tool; each tap on a piece removes it (the preview is the TrimSurfaceOp
// itself), Reset brings them all back, Confirm records one Trim step.
class TrimSurfaceController : public InteractiveOpController {
protected:
    const char* title() const override { return "Trim"; }
    int onBegin(const IopContext& ctx) override;
    std::unique_ptr<Operation> buildOp(const IopContext& ctx) override;
    void panelBody(const IopContext& ctx, bool& changed) override;
    void onCleanup() override;
    bool wantsViewportInput() const override { return true; }
    void onViewportInput(const IopViewport& vp, const IopContext& ctx) override;
    void drawOverlay(const IopOverlay& ov) const override;

private:
    int m_toolBody = -1;
    int m_toolSketch = -1;
    std::vector<gp_Pnt> m_removeAt;
    glm::vec2 m_pressAt{0.0f};
    bool m_pressed = false;
    bool m_moved = false;
};

} // namespace materializr
