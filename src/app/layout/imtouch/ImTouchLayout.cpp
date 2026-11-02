// im-touch layout (UiLayout::ImTouch - the name is an homage to ImGui):
// near-zero chrome. The viewport fills the whole work rect; everything else
// floats over it - project/selection chip (top-left), undo + keyboard + menu
// (top-right), the contextual tool catalogue on the left edge, the
// Fusion-style history timeline (bottom-center), a "+" create FAB
// (bottom-right), and an fps readout.
//
// Everything fundamental (menus, tool catalogue, history editing) is shared
// code - see layout/LayoutCommon.h for the keep-in-lockstep contract.

#include "core/Units.h"
#include "core/ItemsPanelActions.h"  // setBodyMaterialAndMark / setBodyColorAndMark
#include "app/Application.h"
#include "app/layout/LayoutCommon.h"
#include <algorithm>   // std::min - viewport-capped popup height
#include "core/Document.h"
#include "core/History.h"
#include "core/Operation.h"
#include "core/SelectionManager.h"
#include "modeling/DeleteOp.h"          // Items tree: body delete via History
#include "modeling/SketchEditOp.h"      // timeline: Apply cascade targets
#include "modeling/SketchTransformOp.h"
#include "ui/UiTheme.h"                 // viewportTopChromeBottom
#include "modeling/SketchTool.h"   // SketchToolMode for the select-mode gate
#include "plugin/PluginContext.h"
#include "plugin/PluginRegistry.h"   // command-first rail: plugin by name
#include "ui/HistoryPanel.h"
#include "ui/MaterialPicker.h"
#include "ui/Toolbar.h"       // ToolAction for the tool-bar entries
#include "ui/TouchIcons.h"
#include "ui/TouchTheme.h"
#include "ui/TouchWidgets.h"
#include "touch_mode.h"
#include "ui_scale.h"

#include <cfloat> // FLT_MAX (tool bar height constraint)
#include <cstring> // strncpy (rename buffers)
#include <imgui.h>
#include <memory>  // make_unique (DeleteOp)
#include <set>
#include <string>
#include "../../../i18n.h"
#include "../../../i18n.h"
#include "../../../i18n.h"
#include "../../../i18n.h"
#include "../../../i18n.h"
#include "../../../i18n.h"

namespace materializr {

namespace {

// Icon for a history step's box in the timeline, by op typeId. Reloaded
// steps (ReplayOp) report the ORIGINAL op's typeId, so they map the same.
const char* stepIcon(const std::string& t) {
    if (t == "extrude")                    return MZ_ICON_EXTRUDE;
    if (t == "pushpull" || t == "moveface" || t == "move_hole")
                                           return MZ_ICON_PUSHPULL;
    if (t == "revolve")                    return MZ_ICON_LATHE;
    if (t == "loft" || t == "sweep")       return MZ_ICON_SPLINE;
    if (t == "fillet")                     return MZ_ICON_FILLET;
    if (t == "chamfer")                    return MZ_ICON_CHAMFER;
    if (t == "shell")                      return MZ_ICON_SHELL;
    if (t == "boolean")                    return MZ_ICON_SUBTRACT;
    if (t == "split_body")                 return MZ_ICON_TRIM;
    if (t == "delete")                     return MZ_ICON_DELETE;
    if (t == "defeature")                  return MZ_ICON_REPAIR;
    if (t == "offset_face")                return MZ_ICON_PUSHPULL;
    if (t == "mirror")                     return MZ_ICON_MIRROR;
    if (t == "pattern")                    return MZ_ICON_PATTERN;
    if (t == "copy" || t == "duplicate_sketch")
                                           return MZ_ICON_COPY;
    if (t == "scale_face" || t == "resize_cylindrical" || t == "taper")
                                           return MZ_ICON_SCALE;
    if (t == "primitive")                  return MZ_ICON_PRIMITIVE;
    if (t == "thread")                     return MZ_ICON_THREAD;
    if (t == "construction_plane" || t == "construction_axis")
                                           return MZ_ICON_AXES;
    if (t == "sketchedit" || t == "combine_sketches" || t == "project_sketch")
                                           return MZ_ICON_SKETCH;
    if (t == "transform" || t == "axis_transform" || t == "plane_transform" ||
        t == "sketchtransform" || t == "align")
                                           return MZ_ICON_MOVE;
    return MZ_ICON_EDIT;
}

// ── Fork: command-first workspace rail ────────────────────────────────────
// Selection-type bits for WsCmd::mask (bit = 1 << SelectionType).
constexpr unsigned kSelB = 1u << static_cast<unsigned>(SelectionType::Body);
constexpr unsigned kSelF = 1u << static_cast<unsigned>(SelectionType::Face);
constexpr unsigned kSelE = 1u << static_cast<unsigned>(SelectionType::Edge);
constexpr unsigned kSelS = 1u << static_cast<unsigned>(SelectionType::Sketch);
constexpr unsigned kSelR = 1u << static_cast<unsigned>(SelectionType::SketchRegion);
constexpr unsigned kSelP = 1u << static_cast<unsigned>(SelectionType::Plane);
constexpr unsigned kSelA = 1u << static_cast<unsigned>(SelectionType::Axis);
constexpr unsigned kSelC = 1u << static_cast<unsigned>(SelectionType::CurvePoint);

// One rail command: a ToolAction, or a plugin contribution by name. minCount
// 0 = no input (fires at once); single = fires on the first matching pick.
struct WsCmd {
    const char* icon;
    const char* label;
    ToolAction action;
    const char* plugin;
    unsigned mask;
    int minCount;
    bool single;
};
// label == nullptr: the commands sit flat on the rail; otherwise a flyout.
struct WsGroup {
    const char* label;
    const char* icon;
    std::vector<WsCmd> cmds;
};

const WsCmd kCmdSketch  {MZ_ICON_SKETCH,  "Sketch",  ToolAction::SketchOnFace, nullptr, kSelF | kSelP, 1, true};
const WsCmd kCmdMove    {MZ_ICON_MOVE,    "Move",    ToolAction::Move, nullptr,
                         kSelB | kSelF | kSelE | kSelS | kSelR | kSelP | kSelA | kSelC, 1, false};
const WsCmd kCmdMirror  {MZ_ICON_MIRROR,  "Mirror",  ToolAction::Mirror, nullptr, kSelB, 1, false};
const WsCmd kCmdSplit   {MZ_ICON_SPLIT,   "Split",   ToolAction::Split, nullptr, kSelB, 1, true};
const WsCmd kCmdLoft    {MZ_ICON_SPLINE,  "Loft",    ToolAction::None, "Loft", kSelS | kSelR | kSelF | kSelB, 2, false};
const WsCmd kCmdRemove  {MZ_ICON_REPAIR,  "Remove",  ToolAction::RemoveFace, nullptr, kSelF, 1, false};
const WsCmd kCmdDelete  {MZ_ICON_DELETE,  "Delete",  ToolAction::None, "Delete", kSelB, 1, false};
const WsCmd kCmdMeasure {MZ_ICON_MEASURE, "Measure", ToolAction::Measure, nullptr, 0, 0, false};
// Sweep: the profile sketch first, then the path sketch.
const WsCmd kCmdSweep   {MZ_ICON_SPLINE,  "Sweep",   ToolAction::Sweep, nullptr, kSelS | kSelR | kSelB, 2, false};

const std::vector<WsGroup>& solidWorkspace() {
    static const std::vector<WsGroup> g = {
        {nullptr, nullptr, {
            kCmdSketch,
            {MZ_ICON_EXTRUDE,  "Extrude", ToolAction::ExtrudeSketch, nullptr, kSelS | kSelR | kSelF, 1, false},
            {MZ_ICON_PUSHPULL, "Push",    ToolAction::PushPull, nullptr, kSelR | kSelF, 1, true},
            {MZ_ICON_FILLET,   "Fillet",  ToolAction::Fillet, nullptr, kSelE, 1, false},
            {MZ_ICON_CHAMFER,  "Chamfer", ToolAction::Chamfer, nullptr, kSelE, 1, false},
        }},
        {"Create", MZ_ICON_LATHE, {
            {MZ_ICON_LATHE,    "Revolve", ToolAction::Revolve, nullptr, kSelS | kSelR, 1, true},
            kCmdLoft,
            kCmdSweep,
        }},
        {"Modify", MZ_ICON_SHELL, {
            {MZ_ICON_SHELL,    "Shell",       ToolAction::Shell, nullptr, kSelF, 1, false},
            {MZ_ICON_SPLIT,    "Draft",       ToolAction::Taper, nullptr, kSelF, 1, false},
            {MZ_ICON_PUSHPULL, "Offset Face", ToolAction::OffsetFace, nullptr, kSelF, 1, false},
            {MZ_ICON_SCALE,    "Scale Face",  ToolAction::ScaleFace, nullptr, kSelF, 1, true},
            {MZ_ICON_THREAD,   "Thread",      ToolAction::Thread, nullptr, kSelF, 1, true},
            kCmdRemove,
        }},
        {"Combine", MZ_ICON_UNION, {
            {MZ_ICON_UNION,     "Union",     ToolAction::None, "Union", kSelB, 2, false},
            {MZ_ICON_SUBTRACT,  "Subtract",  ToolAction::None, "Subtract", kSelB, 2, false},
            {MZ_ICON_INTERSECT, "Intersect", ToolAction::None, "Intersect", kSelB, 2, false},
        }},
        {nullptr, nullptr, { kCmdMove }},
        {"Pattern", MZ_ICON_COPY, {
            {MZ_ICON_COPY,             "Duplicate", ToolAction::None, "Duplicate", kSelB, 1, false},
            kCmdMirror,
            {MZ_ICON_PATTERN_LINEAR,   "Linear",    ToolAction::None, "Linear Pattern", kSelB, 1, false},
            {MZ_ICON_PATTERN_CIRCULAR, "Circular",  ToolAction::None, "Circular Pattern", kSelB, 1, false},
            {MZ_ICON_SCALE,            "Scale",     ToolAction::Scale, nullptr, kSelB, 1, false},
            kCmdSplit,
        }},
        {nullptr, nullptr, { kCmdDelete, kCmdMeasure }},
    };
    return g;
}

const std::vector<WsGroup>& surfaceWorkspace() {
    static const std::vector<WsGroup> g = {
        {nullptr, nullptr, {
            kCmdSketch,
            {MZ_ICON_SPLINE, "Spline 3D", ToolAction::StudioSpline, nullptr, 0, 0, false},
            {MZ_ICON_EXTRUDE, "Extrude", ToolAction::SurfaceExtrude, nullptr, kSelS | kSelR, 1, true},
            kCmdLoft,
            kCmdSweep,
            {MZ_ICON_PATCH, "Patch", ToolAction::None, "Patch", kSelE, 1, false},
            // Trim: the body to trim first, then the tool (body or sketch).
            {MZ_ICON_TRIM,  "Trim",  ToolAction::TrimSurface, nullptr, kSelB | kSelS | kSelR, 2, false},
            {MZ_ICON_SEW,   "Sew",   ToolAction::None, "Sew", kSelB, 1, false},
            kCmdRemove,
        }},
        {"Modify", MZ_ICON_PUSHPULL, {
            {MZ_ICON_PUSHPULL, "Extend", ToolAction::ExtendSurface, nullptr, kSelE, 1, false},
            {MZ_ICON_SHELL,    "Offset", ToolAction::OffsetSurface, nullptr, kSelF, 1, false},
        }},
        {"Edit", MZ_ICON_MOVE, {
            kCmdMove,
            kCmdMirror,
            kCmdSplit,
        }},
        {nullptr, nullptr, { kCmdDelete, kCmdMeasure }},
    };
    return g;
}

// Does the workspace rail already carry this catalogue tool?
bool wsCovers(const std::vector<WsGroup>& groups, const Toolbar::RailTool& t) {
    for (const WsGroup& g : groups)
        for (const WsCmd& c : g.cmds) {
            if (t.pluginIndex >= 0) {
                if (c.plugin && t.label &&
                    (std::strcmp(c.plugin, t.label) == 0 ||
                     std::strcmp(c.label, t.label) == 0))
                    return true;
            } else if (c.action != ToolAction::None && c.action == t.action) {
                return true;
            }
        }
    return false;
}

bool surfaceOnlyTool(const Toolbar::RailTool& t) {
    return t.pluginIndex >= 0 && t.label &&
           (std::strcmp(t.label, "Patch") == 0 || std::strcmp(t.label, "Sew") == 0);
}

bool solidOnlyTool(const Toolbar::RailTool& t) {
    if (t.pluginIndex >= 0)
        return t.label && (std::strcmp(t.label, "Union") == 0 ||
                           std::strcmp(t.label, "Subtract") == 0 ||
                           std::strcmp(t.label, "Intersect") == 0);
    switch (t.action) {
    case ToolAction::ExtrudeSketch: case ToolAction::PushPull:
    case ToolAction::SubtractSketch: case ToolAction::Shell:
    case ToolAction::ScaleFace: case ToolAction::Taper:
    case ToolAction::Fillet: case ToolAction::Chamfer:
    case ToolAction::EditFilletChamfer: case ToolAction::Thread:
    case ToolAction::EditDiameter: case ToolAction::OffsetFace:
    case ToolAction::ProjectSketch: case ToolAction::MergeFaces:
        return true;
    default:
        return false;
    }
}

} // namespace

void Application::renderImTouchLayout() {
    touchui::Scope style;

    const float s = uiScale();
    auto tip = [&](const char* text) {   // same policy as the modern layout
        if (!m_showToolbarTooltips || !text || touchMode()) return;
        if (ImGui::BeginItemTooltip()) {
            ImGui::PushTextWrapPos(ImGui::GetFontSize() * 22.0f);
            ImGui::TextUnformatted(text);
            ImGui::PopTextWrapPos();
            ImGui::EndTooltip();
        }
    };
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    const ImVec2 wp = vp->WorkPos;
    const ImVec2 ws = vp->WorkSize;
    const float m = 12.0f * s; // float margin from the work-rect edges

    // Reserve the strip the top-left chips and top-right cluster occupy, so the
    // overlays drawn INSIDE the viewport window (the op banner, the toast) start
    // below them instead of underneath. Both clusters are one 44*s box tall and
    // sit at wp.y + m. See viewportTopChromeBottom.
    materializr::viewportTopChromeBottom() = wp.y + m + 44.0f * s;

    // Viewport underneath everything.
    m_touchVpX = wp.x;
    m_touchVpY = wp.y;
    m_touchVpW = ws.x;
    m_touchVpH = ws.y;

    // These overlays float ON TOP of the full-screen viewport window, which is
    // NoBringToFrontOnFocus (pinned to the back). They must NOT share that flag:
    // if they do, z-order falls to ImGui's persistent creation order - which is
    // fine when the app LAUNCHES straight into im-touch (overlays created
    // early), but when the user TOGGLES to it at runtime the viewport was
    // created first and stays in front, burying every overlay (the "invisible
    // shell" bug). Dropping the flag makes them ordinary foreground windows
    // that come to front on appearance, so they render above the back-pinned
    // viewport every time.
    const ImGuiWindowFlags kFloat =
        (layoutui::kShellWindowFlags & ~ImGuiWindowFlags_NoBringToFrontOnFocus) |
        ImGuiWindowFlags_AlwaysAutoResize;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, touchui::radius(14.0f * s));
    // No window borders on any of the floating overlays - the 1px frame reads
    // as a faint "ghost" rectangle around transparent windows (the +, the
    // chip, the buttons). Their rounded fill is the only chrome we want.
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);

    // ── Top-left: [Logo] [Menu] [Project name] - three INDIVIDUAL boxes of
    //    equal height (matching the top-right cluster's button boxes), on a
    //    fully transparent host window so each box carries its own fill.
    ImGui::SetNextWindowPos(ImVec2(wp.x + m, wp.y + m));
    ImGui::SetNextWindowBgAlpha(0.0f);
    if (ImGui::Begin("##LiteChip", nullptr, kFloat)) {
        const float bh = 44.0f * s;
        ImDrawList* dl = ImGui::GetWindowDrawList();

        // Logo box.
        {
            const ImVec2 p = ImGui::GetCursorScreenPos();
            dl->AddRectFilled(p, ImVec2(p.x + bh, p.y + bh),
                              ImGui::GetColorU32(touchui::rowBg()),
                              touchui::radius(10.0f * s));
            const float lg = 26.0f * s;
            const ImVec2 c0(p.x + (bh - lg) * 0.5f, p.y + (bh - lg) * 0.5f);
            dl->AddImageRounded(layoutui::logoTexture(), c0,
                                ImVec2(c0.x + lg, c0.y + lg),
                                ImVec2(0, 0), ImVec2(1, 1),
                                IM_COL32_WHITE, touchui::radius(4.0f * s));
            ImGui::Dummy(ImVec2(bh, bh));
        }
        ImGui::SameLine(0.0f, 8.0f * s);

        // Menu box.
        if (touchui::iconButton("menu", MZ_ICON_MENU_BARS, bh))
            ImGui::OpenPopup("##TouchOverflow");
        tip(materializr::tr("Menu: file, edit, view, help and settings"));
        renderTouchOverflowPopup();
        ImGui::SameLine(0.0f, 8.0f * s);

        // Project-name box (name + selection summary, e.g. "mug.mzr · Face (1)").
        {
            std::string pn = projectDisplayName();
            std::string sel;
            if (m_selection && m_selection->hasSelection()) {
                const SelectionType t = m_selection->primaryType();
                int n = 0;
                for (const auto& e : m_selection->getSelection())
                    if (e.type == t) ++n;
                static const char* kNames[] = { "None", "Body", "Face", "Edge",
                                                "Vertex", "Sketch", "Region",
                                                "Plane", "Axis", "Point" };
                const int ti = static_cast<int>(t);
                if (ti > 0 && ti < 10) {
                    sel = std::string("  ·  ") + kNames[ti] +
                          " (" + std::to_string(n) + ")";
                }
            }
            // Tab affordance (Steve's popout): more than one open project adds
            // a chevron and the chip becomes the button that opens the tabs
            // sheet - no extra chrome when only one project is open (the chip
            // still opens the sheet then too, for "+ New Project").
            std::string chev = "  " ICON_IC_NAV_ARROW_DOWN;
            if (sessionDirty(m_activeSession)) pn += " \xe2\x80\xa2";
            const float padX = 14.0f * s;
            const ImVec2 tn = ImGui::CalcTextSize(pn.c_str());
            const ImVec2 tsl = sel.empty() ? ImVec2(0.0f, 0.0f)
                                           : ImGui::CalcTextSize(sel.c_str());
            const ImVec2 tc = ImGui::CalcTextSize(chev.c_str());
            const float bw = padX * 2.0f + tn.x + tsl.x + tc.x;
            const ImVec2 p = ImGui::GetCursorScreenPos();
            dl->AddRectFilled(p, ImVec2(p.x + bw, p.y + bh),
                              ImGui::GetColorU32(touchui::rowBg()),
                              touchui::radius(10.0f * s));
            dl->AddText(ImVec2(p.x + padX, p.y + (bh - tn.y) * 0.5f),
                        ImGui::GetColorU32(touchui::textPrimary()), pn.c_str());
            if (!sel.empty())
                dl->AddText(ImVec2(p.x + padX + tn.x, p.y + (bh - tsl.y) * 0.5f),
                            ImGui::GetColorU32(touchui::textDim()), sel.c_str());
            dl->AddText(ImVec2(p.x + padX + tn.x + tsl.x,
                               p.y + (bh - tc.y) * 0.5f),
                        ImGui::GetColorU32(touchui::textDim()), chev.c_str());
            if (ImGui::InvisibleButton("##projTabsChip", ImVec2(bw, bh)))
                ImGui::OpenPopup("##TouchTabs");
            renderTouchTabsSheet();
        }

        // Fork: Fusion-style workspace tabs. They pick which tools the left
        // rail offers (see the rail below); a sketch has its own tool set.
        if (!m_inSketchMode) {
            ImGui::SameLine(0.0f, 8.0f * s);
            static const char* const kWorkspaces[] = { "Solid", "Surface" };
            const char* items[2] = { materializr::tr(kWorkspaces[0]),
                                     materializr::tr(kWorkspaces[1]) };
            // segmented() sizes itself to the available width, which an
            // auto-resizing window doesn't have - give it a fixed-width child
            // on the same rowBg plate as the chips beside it.
            float segW = 0.0f;
            for (const char* it : items)
                segW += ImGui::CalcTextSize(it).x + 24.0f * s;
            segW += 16.0f * s;
            const ImVec2 p = ImGui::GetCursorScreenPos();
            dl->AddRectFilled(p, ImVec2(p.x + segW, p.y + bh),
                              ImGui::GetColorU32(touchui::rowBg()),
                              touchui::radius(10.0f * s));
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
            ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0, 0, 0, 0));
            if (ImGui::BeginChild("##workspaceTabs", ImVec2(segW, bh), false,
                                  ImGuiWindowFlags_NoScrollbar |
                                      ImGuiWindowFlags_NoScrollWithMouse)) {
                const int ws0 = m_imTouchWorkspace;
                m_imTouchWorkspace = touchui::segmented("workspace", items, 2, ws0);
                if (m_imTouchWorkspace != ws0) saveAppSettings();
            }
            ImGui::EndChild();
            ImGui::PopStyleColor();
            ImGui::PopStyleVar();
        }
    }
    ImGui::End();

    // ── Undo / keyboard / menu (top-right) ──────────────────────────────────
    ImGui::SetNextWindowPos(ImVec2(wp.x + ws.x - m, wp.y + m),
                            ImGuiCond_Always, ImVec2(1.0f, 0.0f));
    ImGui::SetNextWindowBgAlpha(0.0f);
    if (ImGui::Begin("##LiteTopRight", nullptr, kFloat)) {
        const float bh = 44.0f * s;
        // Multi-Select (moved off the bottom-left viewport bar): 3D selection and
        // sketch Select/move mode only.
        const bool showMulti = !m_inSketchMode ||
            (m_sketchTool && m_sketchTool->getMode() == SketchToolMode::Select);
        if (showMulti) {
            if (touchui::pillButton("multi", MZ_ICON_SELECT, materializr::tr("Multi"),
                                    m_multiSelectToggle))
                m_multiSelectToggle = !m_multiSelectToggle;
            tip(materializr::tr("Multi-select: add taps to the current selection\n(the touch equivalent of holding Ctrl)"));
            ImGui::SameLine(0.0f, 8.0f * s);
        }
        // Inference level, in sketch mode - the same live cycle the modern
        // layout puts in its top bar and the classic toolbar puts in its rail.
        // It IS in this layout's tool catalogue, but only inside the "More"
        // flyout at the BOTTOM of a dock that scrolls once the sketch tools
        // outgrow the screen, so on a tablet it sits below the fold and reads
        // as missing (Steve: "i cannot seem to find it"). Beside the snap pill
        // is where it belongs - the two are the same kind of drawing aid, and
        // this cluster never scrolls. Honours the same "show the toggle"
        // setting the other layouts gate on.
        if (m_inSketchMode && m_showInferenceToolbarToggle && m_sketchTool) {
            const char* infLbl = "Full";
            switch (m_sketchTool->getInferenceLevel()) {
                case SketchTool::InferenceLevel::Full:    infLbl = "Full";    break;
                case SketchTool::InferenceLevel::Reduced: infLbl = "Reduced"; break;
                case SketchTool::InferenceLevel::Off:     infLbl = "Off";     break;
                case SketchTool::InferenceLevel::Max:     infLbl = "Max";     break;
            }
            if (touchui::twoRowButton("inflvl", materializr::tr("Inference"), infLbl))
                handleToolAction(static_cast<int>(ToolAction::SketchCycleInference));
            tip(materializr::tr("Sketch inference level (snapping / guides)\nTap to cycle: Full \xE2\x86\x92 Reduced \xE2\x86\x92 Off \xE2\x86\x92 Max\nMax widens the catch ranges for fingertips."));
            ImGui::SameLine(0.0f, 8.0f * s);
        }
        // Snap-to-grid - the corner square's im-touch home (renderSnapWidget
        // skips itself in this layout). Fork: grid snap removed, so the
        // button is gone too.
        if (false) {
            char snapLbl[16];
            // Display unit, like the presets that set it and like the desktop
            // badge. Raw mm here read "10" after picking the "1" preset under cm.
            std::snprintf(snapLbl, sizeof(snapLbl), "%.3g",
                          materializr::toDisplay(m_effectiveGridStepMm));
            if (touchui::pillButton("snap", MZ_ICON_GUIDES, snapLbl,
                                    m_snapToGrid))
                ImGui::OpenPopup("SnapSettings");
            tip(materializr::tr(m_snapToGrid ? "Snap ON - tap for step / toggle"
                                             : "Snap off - tap for step / toggle"));
            renderSnapSettingsPopup();
            ImGui::SameLine(0.0f, 8.0f * s);
        }
        // Items (model tree) reveal/hide - moved up from the right-edge rail
        // button so the whole toggle row lives in one place.
        if (touchui::pillButton("items", MZ_ICON_ITEMS, nullptr,
                                m_imTouchTree)) {
            m_imTouchTree = !m_imTouchTree;
            saveAppSettings();
        }
        tip(materializr::tr(m_imTouchTree
                ? "Hide the model tree"
                : "Show the model tree (bodies, sketches, construction)"));
        ImGui::SameLine(0.0f, 8.0f * s);
        const bool histLocked = anyInteractivePreviewActive();
        ImGui::BeginDisabled(histLocked || !touchCanUndo());
        if (touchui::iconButton("undo", MZ_ICON_UNDO, bh)) touchUndo();
        ImGui::EndDisabled();
        tip(materializr::tr("Undo (in a sketch: backs out the in-progress shape first)"));
        ImGui::SameLine(0.0f, 8.0f * s);
        ImGui::BeginDisabled(histLocked || !(m_history->canRedo() || paramRedoAvailable()));
        if (touchui::iconButton("redo", MZ_ICON_REDO, bh)) {
            if (paramRedoAvailable()) paramRedo();
            else redoWithCascade();
        }
        ImGui::EndDisabled();
        tip(materializr::tr("Redo"));
        if (materializr::touchMode()) {
            ImGui::SameLine(0.0f, 8.0f * s);
            if (touchui::iconButton("kb", MZ_ICON_KEYBOARD, bh))
                m_softKeyboardForced = !m_softKeyboardForced;
            tip(materializr::tr("Toggle the on-screen keyboard"));
        }
        // (History has its own bottom toggle; the ⋯ menu lives on the
        // top-left chip.)
    }
    ImGui::End();

    // (The Items toggle now lives in the top cluster; railBtnW is still the
    // History toggle's size at the bottom.)
    const float railBtnW = 60.0f * s;

    // ── Transparent model tree (right edge) - the structure the modern
    //    layout's Items panel shows: visibility eye + name + tap-to-select,
    //    plus press-and-hold context menus (rename / delete / move-to-folder)
    //    and folder grouping, mirroring ItemsPanel but touch-native.
    m_imTouchTreeHovered = false;   // fed to the long-press gate every frame
    if (m_imTouchTree && m_document) {
        // Flush to the right edge, below the top cluster's reach.
        ImGui::SetNextWindowPos(
            ImVec2(wp.x + ws.x - m, wp.y + ws.y * 0.5f),
            ImGuiCond_Always, ImVec2(1.0f, 0.5f));
        const float treeW = 260.0f * s;
        ImGui::SetNextWindowSizeConstraints(ImVec2(treeW, 0),
                                            ImVec2(treeW, ws.y - 2.0f * m));
        ImGui::SetNextWindowBgAlpha(0.25f);   // Fusion-browser translucency
        if (ImGui::Begin("##LiteTree", nullptr,
                         kFloat & ~ImGuiWindowFlags_NoScrollbar)) {
            // Report hover (incl. while a row is the active item) so a
            // stationary long-press over the tree arms a synthetic right-click.
            m_imTouchTreeHovered = ImGui::IsWindowHovered(
                ImGuiHoveredFlags_RootAndChildWindows |
                ImGuiHoveredFlags_AllowWhenBlockedByActiveItem);
            // (Project name intentionally omitted here - the top-left chip
            // already shows it; dropping the browser's root row reclaims space.)

            // Selected ids per kind, collected once.
            std::set<int> selB, selS, selP, selA;
            if (m_selection)
                for (const auto& e : m_selection->getSelection()) {
                    if (e.type == SelectionType::Body   && e.bodyId   >= 0) selB.insert(e.bodyId);
                    if (e.type == SelectionType::Sketch && e.sketchId >= 0) selS.insert(e.sketchId);
                    if (e.type == SelectionType::Plane  && e.planeId  >= 0) selP.insert(e.planeId);
                    if (e.type == SelectionType::Axis   && e.axisId   >= 0) selA.insert(e.axisId);
                }
            // Plain tap = single-select; with the Multi toggle armed, bodies
            // toggle in/out of the selection (same semantics as the Items
            // panel's Ctrl+click). Body picks are navigation-only, exactly
            // like the Items panel's rows; other kinds select plainly.
            auto pick = [&](SelectionEntry entry, bool multiOk) {
                if (!m_selection) return;
                if (multiOk && m_multiSelectToggle) m_selection->toggleSelection(entry);
                else                                m_selection->select(entry);
                if (entry.type == SelectionType::Body)
                    m_selection->setNavigationOnly(true);
            };
            // Rename is a native-keyboard modal (raised after the tree). The
            // key is namespaced so kinds never collide (see Application.h).
            auto startRename = [&](int key, const std::string& cur) {
                m_imTouchRenameKey = key;
                std::strncpy(m_imTouchRenameBuf, cur.c_str(),
                             sizeof(m_imTouchRenameBuf) - 1);
                m_imTouchRenameBuf[sizeof(m_imTouchRenameBuf) - 1] = '\0';
                m_imTouchRenameOpen = true;
            };

            // One body row + its long-press context menu. Returns false when a
            // Delete made the surrounding id list stale (caller stops looping).
            // Hue-wheel colour popup (opened on a swatch tap). Returns the new
            // colour via `out` when the user drags it. Called inside the row's
            // PushID scope, so the popup id is unique per row.
            auto colorPopup = [&](const char* popupId, const char* title,
                                  glm::vec3 cur, glm::vec3& out) -> bool {
                bool changed = false;
                if (ImGui::BeginPopup(popupId)) {
                    ImGui::TextUnformatted(materializr::tr(title));
                    ImGui::SetNextItemWidth(materializr::uiSz(220, 0).x);
                    if (ImGui::ColorPicker3(
                            "##pick", &cur.x,
                            ImGuiColorEditFlags_NoInputs |
                            ImGuiColorEditFlags_NoSidePreview |
                            ImGuiColorEditFlags_PickerHueWheel)) {
                        out = cur;
                        changed = true;
                    }
                    ImGui::EndPopup();
                }
                return changed;
            };

            auto renderBody = [&](int id) -> bool {
                bool gone = false;
                ImGui::PushID(id);
                bool visible = m_document->isBodyVisible(id);
                glm::vec3 bcol = m_document->getBodyColor(id);
                auto act = touchui::treeLeaf(
                    "body", MZ_ICON_BODY,
                    m_document->getBodyName(id).c_str(), &visible,
                    selB.count(id) > 0, &bcol.x);
                if (act.eyeToggled) {
                    m_document->setBodyVisible(id, visible);
                    // The viewport only filters hidden bodies when it rebuilds
                    // their meshes - without this the flag flips but the stale
                    // mesh keeps drawing (#37; desktop's ItemsPanel returns
                    // colorChanged for the same reason). Just this body: the
                    // partial pass drops it or meshes it, whichever way the
                    // eye went.
                    markBodyDirty(id);
                    markDirty();
                }
                // Material + colour, the same popup as the desktop Items
                // panel's swatch. Both bake into the render mesh.
                if (act.swatchClicked) ImGui::OpenPopup("bodyColor");
                if (ImGui::BeginPopup("bodyColor")) {
                    const auto pick = materializr::bodyAppearancePicker(
                        m_document->getBodyMaterial(id), m_document->getBodyColor(id),
                        materializr::uiSz(220, 0).x);
                    auto mark = [&](int b) { markBodyDirty(b); };
                    if (pick.materialPicked) {
                        materializr::setBodyMaterialAndMark(*m_document, id,
                                                            pick.materialId, mark);
                        markDirty();
                    }
                    if (pick.colorPicked) {
                        materializr::setBodyColorAndMark(*m_document, id, pick.color, mark);
                        markDirty();
                    }
                    ImGui::EndPopup();
                }
                if (act.clicked) {
                    SelectionEntry e;
                    e.type = SelectionType::Body;
                    e.bodyId = id;
                    // Parity with ItemsPanel::makeEntry - downstream code
                    // (highlight outline, ops) expects the shape on the entry.
                    try { e.shape = m_document->getBody(id); } catch (...) {}
                    pick(e, /*multiOk=*/true);
                }
                if (act.rightClicked) ImGui::OpenPopup("bodyCtx");
                if (ImGui::BeginPopup("bodyCtx")) {
                    if (ImGui::MenuItem(materializr::tr("Rename")))
                        startRename(id, m_document->getBodyName(id));
                    if (ImGui::MenuItem(materializr::tr("Delete"))) {
                        if (m_history) {
                            auto op = std::make_unique<DeleteOp>();
                            op->setBodyId(id);
                            m_history->pushOperation(std::move(op), *m_document);
                        } else {
                            m_document->removeBody(id);
                        }
                        if (m_selection) m_selection->clear();
                        gone = true;
                    }
                    if (!gone && ImGui::BeginMenu(materializr::tr("Move to folder"))) {
                        if (m_document->getBodyFolder(id) >= 0 &&
                            ImGui::MenuItem(materializr::tr("(root - no folder)"))) {
                            m_document->setBodyFolder(id, -1);
                            markDirty();
                        }
                        for (int fid : m_document->getAllFolderIds())
                            if (ImGui::MenuItem(m_document->getFolderName(fid).c_str())) {
                                m_document->setBodyFolder(id, fid);
                                markDirty();
                            }
                        ImGui::Separator();
                        if (ImGui::MenuItem(materializr::tr("New folder…"))) {
                            m_imTouchNewFolderBodies = { id };
                            m_imTouchNewFolderName[0] = '\0';
                            m_imTouchNewFolderOpen = true;
                        }
                        ImGui::EndMenu();
                    }
                    ImGui::EndPopup();
                }
                ImGui::PopID();
                return !gone;
            };

            bool any = false;
            const auto bodyIds   = m_document->getAllBodyIds();
            const auto folderIds = m_document->getAllFolderIds();
            if (!bodyIds.empty() || !folderIds.empty()) {
                any = true;
                // Visible "+ Folder" pill on the header (its own hit area) -
                // the obvious way to make an empty folder; bodies join via a
                // row's Move-to-folder menu.
                bool addFolderClick = false;
                if (touchui::treeGroup("grpBodies", "Bodies",
                                       static_cast<int>(bodyIds.size()),
                                       m_imTouchTreeOpenBodies, nullptr,
                                       "+ Folder", &addFolderClick))
                    m_imTouchTreeOpenBodies = !m_imTouchTreeOpenBodies;
                if (addFolderClick) {
                    m_imTouchNewFolderBodies.clear();
                    m_imTouchNewFolderName[0] = '\0';
                    m_imTouchNewFolderOpen = true;
                }
                if (m_imTouchTreeOpenBodies) {
                    // 1) Folders, each with their member bodies indented.
                    bool bodiesStale = false;
                    for (int fid : folderIds) {
                        ImGui::PushID(2000000 + fid); // namespace off body ids
                        bool fvis = m_document->isFolderVisible(fid);
                        bool fexp = m_document->isFolderExpanded(fid);
                        glm::vec3 fcol = m_document->getFolderColor(fid);
                        auto fact = touchui::treeLeaf(
                            "folder", MZ_ICON_OPEN,
                            m_document->getFolderName(fid).c_str(), &fvis,
                            /*selected=*/false, &fcol.x);
                        if (fact.eyeToggled) {
                            m_document->setFolderVisible(fid, fvis);
                            markFolderBodiesDirty(fid);  // cascades to members (#37)
                            markDirty();
                        }
                        if (fact.clicked)
                            m_document->setFolderExpanded(fid, !fexp);
                        if (fact.swatchClicked) ImGui::OpenPopup("folderColor");
                        glm::vec3 fNewCol;
                        if (colorPopup("folderColor", "Folder colour",
                                       m_document->getFolderColor(fid), fNewCol)) {
                            m_document->setFolderColor(fid, fNewCol); // cascades
                            markFolderBodiesDirty(fid);
                            markDirty();
                        }
                        if (fact.rightClicked) ImGui::OpenPopup("folderCtx");
                        bool folderGone = false;
                        if (ImGui::BeginPopup("folderCtx")) {
                            if (ImGui::MenuItem(materializr::tr("Rename")))
                                startRename(2000000 + fid,
                                            m_document->getFolderName(fid));
                            if (ImGui::MenuItem(materializr::tr("Delete folder (keeps bodies)"))) {
                                m_document->removeFolder(fid);
                                markDirty();
                                folderGone = true;
                            }
                            ImGui::EndPopup();
                        }
                        if (!folderGone && fexp) {
                            ImGui::Indent();
                            for (int bid : m_document->getBodiesInFolder(fid))
                                if (!renderBody(bid)) { bodiesStale = true; break; }
                            ImGui::Unindent();
                        }
                        ImGui::PopID();
                        if (folderGone || bodiesStale) break;
                    }
                    // 2) Root-level bodies (folderId == -1).
                    if (!bodiesStale)
                        for (int id : m_document->getBodiesInFolder(-1))
                            if (!renderBody(id)) break;
                }
            }
            const auto sketchIds = m_document->getAllSketchIds();
            if (!sketchIds.empty()) {
                any = true;
                if (touchui::treeGroup("grpSketches", "Sketches",
                                       static_cast<int>(sketchIds.size()),
                                       m_imTouchTreeOpenSketches))
                    m_imTouchTreeOpenSketches = !m_imTouchTreeOpenSketches;
                if (m_imTouchTreeOpenSketches)
                    for (int id : sketchIds) {
                        ImGui::PushID(id);
                        bool visible = m_document->isSketchVisible(id);
                        auto act = touchui::treeLeaf(
                            "sketch", MZ_ICON_SKETCH,
                            m_document->getSketchName(id).c_str(), &visible,
                            selS.count(id) > 0);
                        if (act.eyeToggled) m_document->setSketchVisible(id, visible);
                        if (act.clicked) {
                            SelectionEntry e;
                            e.type = SelectionType::Sketch;
                            e.sketchId = id;
                            pick(e, /*multiOk=*/false);
                        }
                        bool sgone = false;
                        if (act.rightClicked) ImGui::OpenPopup("sketchCtx");
                        if (ImGui::BeginPopup("sketchCtx")) {
                            // The sketch currently being drawn: renaming or
                            // deleting it out from under the tool that's still
                            // writing to it leaves m_activeSketchId stale and
                            // silently re-loses the geometry the mid-edit save
                            // just protected.
                            bool isBeingDrawn = m_inSketchMode && id == m_activeSketchId;
                            if (ImGui::MenuItem(materializr::tr("Rename"), nullptr, false, !isBeingDrawn))
                                startRename(1000000 + id,
                                            m_document->getSketchName(id));
                            if (ImGui::MenuItem(materializr::tr("Delete"), nullptr, false, !isBeingDrawn)) {
                                deleteSketch(id);
                                sgone = true;
                            }
                            ImGui::EndPopup();
                        }
                        ImGui::PopID();
                        if (sgone) break;   // sketchIds now stale
                    }
            }
            const auto planeIds = m_document->getAllPlaneIds();
            const auto axisIds  = m_document->getAllAxisIds();
            if (!planeIds.empty() || !axisIds.empty()) {
                any = true;
                if (touchui::treeGroup(
                        "grpConstruction", "Construction",
                        static_cast<int>(planeIds.size() + axisIds.size()),
                        m_imTouchTreeOpenConstruction))
                    m_imTouchTreeOpenConstruction = !m_imTouchTreeOpenConstruction;
                if (m_imTouchTreeOpenConstruction) {
                    bool cgone = false;
                    for (int id : planeIds) {
                        ImGui::PushID(id + 100000); // avoid plane/axis id collisions
                        const auto* p = m_document->getPlane(id);
                        std::string label = p ? p->name
                                              : std::string("Plane ") + std::to_string(id);
                        bool visible = m_document->isPlaneVisible(id);
                        auto act = touchui::treeLeaf("plane", MZ_ICON_PLANE,
                                                     label.c_str(), &visible,
                                                     selP.count(id) > 0);
                        if (act.eyeToggled) m_document->setPlaneVisible(id, visible);
                        if (act.clicked) {
                            SelectionEntry e;
                            e.type = SelectionType::Plane;
                            e.planeId = id;
                            pick(e, /*multiOk=*/false);
                        }
                        if (act.rightClicked) ImGui::OpenPopup("planeCtx");
                        if (ImGui::BeginPopup("planeCtx")) {
                            if (ImGui::MenuItem(materializr::tr("Rename")))
                                startRename(4000000 + id, label);
                            if (ImGui::MenuItem(materializr::tr("Delete"))) {
                                m_document->removePlane(id);
                                if (m_selection) m_selection->clear();
                                cgone = true;
                            }
                            ImGui::EndPopup();
                        }
                        ImGui::PopID();
                        if (cgone) break;   // planeIds now stale
                    }
                    for (int id : axisIds) {
                        if (cgone) break;
                        ImGui::PushID(id + 200000);
                        const auto* a = m_document->getAxis(id);
                        std::string label = a ? a->name
                                              : std::string("Axis ") + std::to_string(id);
                        bool visible = m_document->isAxisVisible(id);
                        auto act = touchui::treeLeaf("axis", MZ_ICON_AXES,
                                                     label.c_str(), &visible,
                                                     selA.count(id) > 0);
                        if (act.eyeToggled) m_document->setAxisVisible(id, visible);
                        if (act.clicked) {
                            SelectionEntry e;
                            e.type = SelectionType::Axis;
                            e.axisId = id;
                            pick(e, /*multiOk=*/false);
                        }
                        if (act.rightClicked) ImGui::OpenPopup("axisCtx");
                        if (ImGui::BeginPopup("axisCtx")) {
                            if (ImGui::MenuItem(materializr::tr("Rename")))
                                startRename(5000000 + id, label);
                            if (ImGui::MenuItem(materializr::tr("Delete"))) {
                                m_document->removeAxis(id);
                                if (m_selection) m_selection->clear();
                                cgone = true;
                            }
                            ImGui::EndPopup();
                        }
                        ImGui::PopID();
                        if (cgone) break;   // axisIds now stale
                    }
                }
            }
            if (!any)
                ImGui::TextColored(touchui::textDim(), "%s", materializr::tr("Nothing here yet"));
        }
        ImGui::End();

        // ── Rename modal (native keyboard) - raised from any context "Rename".
        //    Decodes the namespaced key to route the committed name back.
        if (m_imTouchRenameOpen) {
            ImGui::OpenPopup("Rename##imtouch");
            m_imTouchRenameOpen = false;
            m_imTouchRenameFocus = true;
        }
        if (ImGui::BeginPopupModal("Rename##imtouch", nullptr,
                                   ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::TextUnformatted(materializr::tr("Name:"));
            if (m_imTouchRenameFocus) {
                ImGui::SetKeyboardFocusHere();
                m_imTouchRenameFocus = false;
            }
            ImGui::SetNextItemWidth(uiSz(240, 0).x);
            bool committed = ImGui::InputText(
                "##rnbuf", m_imTouchRenameBuf, sizeof(m_imTouchRenameBuf),
                ImGuiInputTextFlags_EnterReturnsTrue |
                ImGuiInputTextFlags_AutoSelectAll);
            bool ok     = ImGui::Button(materializr::tr("Rename"), uiSz(110, 44));
            ImGui::SameLine();
            bool cancel = ImGui::Button(materializr::tr("Cancel"), uiSz(110, 44));
            if (committed || ok) {
                const int key = m_imTouchRenameKey;
                if (m_imTouchRenameBuf[0] != '\0' && key >= 0) {
                    // Key ranges mirror Application.h / ItemsPanel.
                    if (key >= 5000000)      m_document->setAxisName(key - 5000000, m_imTouchRenameBuf);
                    else if (key >= 4000000) m_document->setPlaneName(key - 4000000, m_imTouchRenameBuf);
                    else if (key >= 2000000) m_document->setFolderName(key - 2000000, m_imTouchRenameBuf);
                    else if (key >= 1000000) m_document->setSketchName(key - 1000000, m_imTouchRenameBuf);
                    else                     m_document->setBodyName(key, m_imTouchRenameBuf);
                    markDirty();
                }
                m_imTouchRenameKey = -1;
                ImGui::CloseCurrentPopup();
            } else if (cancel || ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
                m_imTouchRenameKey = -1;
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

        // ── New-folder modal - raised from a body's Move-to-folder menu or the
        //    Bodies header. Creates the folder and drops the pending bodies in.
        if (m_imTouchNewFolderOpen) {
            ImGui::OpenPopup("New Folder##imtouch");
            m_imTouchNewFolderOpen = false;
            m_imTouchNewFolderFocus = true;
        }
        if (ImGui::BeginPopupModal("New Folder##imtouch", nullptr,
                                   ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::TextUnformatted(materializr::tr("Folder name:"));
            if (m_imTouchNewFolderFocus) {
                ImGui::SetKeyboardFocusHere();
                m_imTouchNewFolderFocus = false;
            }
            ImGui::SetNextItemWidth(uiSz(240, 0).x);
            bool committed = ImGui::InputText(
                "##nfname", m_imTouchNewFolderName,
                sizeof(m_imTouchNewFolderName),
                ImGuiInputTextFlags_EnterReturnsTrue);
            bool create = ImGui::Button(materializr::tr("Create"), uiSz(110, 44));
            ImGui::SameLine();
            bool cancel = ImGui::Button(materializr::tr("Cancel"), uiSz(110, 44));
            if (committed || create) {
                if (m_imTouchNewFolderName[0] != '\0') {
                    int newId = m_document->addFolder(m_imTouchNewFolderName);
                    for (int bid : m_imTouchNewFolderBodies)
                        m_document->setBodyFolder(bid, newId);
                    markDirty();
                }
                m_imTouchNewFolderBodies.clear();
                ImGui::CloseCurrentPopup();
            } else if (cancel || ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
                m_imTouchNewFolderBodies.clear();
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }
    }

    // ── Contextual tool bar - the same catalogue the modern layout's rail
    //    uses, floating on the LEFT edge, vertically centered. Sketch mode
    //    appends Finish/Exit pills below the tools. Tall catalogues (sketch
    //    mode on a landscape tablet) can exceed the work rect, so cap the
    //    height and let the bar scroll rather than run off-screen.
    ImGui::SetNextWindowPos(ImVec2(wp.x + m, wp.y + ws.y * 0.5f),
                            ImGuiCond_Always, ImVec2(0.0f, 0.5f));
    // The dock is vertically CENTRED, so it grows symmetrically from the
    // middle. Reserve the top-left (menu chip) and bottom-left (History
    // button) zones so a tall catalogue can't expand over them - cap the max
    // height to twice the smaller half-gap. Beyond that it scrolls.
    const float leftReserve = 96.0f * s; // History / chip button + margin
    const float dockMaxH = std::max(120.0f * s,
                                    ws.y - 2.0f * m - 2.0f * leftReserve);
    ImGui::SetNextWindowSizeConstraints(ImVec2(0, 0),
                                        ImVec2(FLT_MAX, dockMaxH));
    ImGui::SetNextWindowBgAlpha(0.92f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, touchui::panelBg());
    if (ImGui::Begin("##LiteToolBar", nullptr,
                     kFloat & ~ImGuiWindowFlags_NoScrollbar)) {
        if (m_toolbar) {
            const auto tools = m_toolbar->railTools();
            // Fire one catalogue entry; `inPopup` closes the hosting group
            // popup after one-shot actions (the inference cycle stays open so
            // repeated taps can walk the levels, and Polygon opens its own
            // nested sides popup which closes the whole stack on pick).
            auto fire = [&](const Toolbar::RailTool& tool, bool clicked,
                            bool inPopup) {
                if (tool.pluginIndex >= 0) {
                    if (clicked) {
                        m_toolbar->fireRailPlugin(tool.pluginIndex);
                        if (inPopup) ImGui::CloseCurrentPopup();
                    }
                } else if (tool.action == ToolAction::Polygon) {
                    renderRailPolygonSidesPopup(clicked);
                } else if (clicked) {
                    handleToolAction(static_cast<int>(tool.action));
                    if (inPopup &&
                        tool.action != ToolAction::SketchCycleInference)
                        ImGui::CloseCurrentPopup();
                }
            };
            auto flatButton = [&](const Toolbar::RailTool& tool, int id) {
                ImGui::PushID(id); // labels can repeat across groups
                const bool clicked = touchui::railButton(
                    tool.label, tool.icon, tr(tool.label), tool.active, 64.0f * s);
                tip(materializr::tr(tool.tip));
                fire(tool, clicked, /*inPopup=*/false);
                ImGui::PopID();
            };
            // Group button + flyout: the button wears the group's ACTIVE
            // tool (icon, label, accent fill) so the current mode stays
            // readable while collapsed; tapping opens a grid of the members.
            auto group = [&](const char* id, const char* popupId,
                             const char* groupIcon, const char* groupLabel,
                             const char* groupTip,
                             const std::vector<const Toolbar::RailTool*>& members) {
                if (members.empty()) return;
                const Toolbar::RailTool* activeTool = nullptr;
                for (const auto* t : members)
                    if (t->active) { activeTool = t; break; }
                if (touchui::railButton(id,
                                        activeTool ? activeTool->icon : groupIcon,
                                        materializr::tr(activeTool ? activeTool->label
                                                                   : groupLabel),
                                        activeTool != nullptr, 64.0f * s))
                    ImGui::OpenPopup(popupId);
                tip(materializr::tr(groupTip));
                if (ImGui::BeginPopup(popupId)) {
                    int idx = 0;
                    for (const auto* t : members) {
                        if (idx % 4 != 0) ImGui::SameLine(0.0f, 6.0f * s);
                        ImGui::PushID(idx++);
                        const bool clicked = touchui::railButton(
                            t->label, t->icon, tr(t->label), t->active, 64.0f * s);
                        tip(materializr::tr(t->tip));
                        fire(*t, clicked, /*inPopup=*/true);
                        ImGui::PopID();
                    }
                    ImGui::EndPopup();
                }
            };

            if (!m_inSketchMode) {
                // Fork: command-first workspace rail (Fusion-style). The
                // workspace's commands are always on the rail; tapping one
                // either runs it on a matching selection or waits for the
                // user to pick its input (startWorkspaceCommand). The
                // selection-specific tools the catalogue offers on top
                // (Edit Sketch, Edit Fillet, Diameter, ...) sit in "More".
                const bool surfaceWs = (m_imTouchWorkspace == 1);
                const std::vector<WsGroup>& groups =
                    surfaceWs ? surfaceWorkspace() : solidWorkspace();
                int cmdIdx = 0;
                auto cmdButton = [&](const WsCmd& c, bool inPopup) {
                    ImGui::PushID(cmdIdx++);
                    const bool pending = m_pendingCmd.active &&
                        std::strcmp(m_pendingCmd.label, c.label) == 0;
                    if (touchui::railButton(c.label, c.icon, tr(c.label),
                                            pending, 64.0f * s)) {
                        PendingCommand pc;
                        pc.label = c.label;
                        pc.action = static_cast<int>(c.action);
                        pc.plugin = c.plugin;
                        pc.typeMask = c.mask;
                        pc.minCount = c.minCount;
                        pc.single = c.single;
                        startWorkspaceCommand(pc);
                        if (inPopup) ImGui::CloseCurrentPopup();
                    }
                    ImGui::PopID();
                };
                for (const WsGroup& g : groups) {
                    if (!g.label) {                 // flat commands
                        for (const WsCmd& c : g.cmds) cmdButton(c, false);
                        continue;
                    }
                    const char* shownIcon = g.icon;
                    const char* shownLabel = g.label;
                    bool anyPending = false;
                    for (const WsCmd& c : g.cmds)
                        if (m_pendingCmd.active &&
                            std::strcmp(m_pendingCmd.label, c.label) == 0) {
                            shownIcon = c.icon; shownLabel = c.label; anyPending = true;
                        }
                    ImGui::PushID(g.label);
                    if (touchui::railButton("grp", shownIcon, tr(shownLabel),
                                            anyPending, 64.0f * s))
                        ImGui::OpenPopup("##wsGroup");
                    if (ImGui::BeginPopup("##wsGroup")) {
                        int idx = 0;
                        for (const WsCmd& c : g.cmds) {
                            if (idx++ % 4 != 0) ImGui::SameLine(0.0f, 6.0f * s);
                            cmdButton(c, true);
                        }
                        ImGui::EndPopup();
                    }
                    ImGui::PopID();
                }

                // Selection-specific extras from the contextual catalogue:
                // whatever it offers that the workspace rail doesn't already
                // have, minus the other workspace's tools.
                if (m_selection && m_selection->hasSelection() && !m_pendingCmd.active) {
                    std::vector<const Toolbar::RailTool*> extras;
                    for (const auto& t : tools) {
                        if (wsCovers(groups, t)) continue;
                        if (surfaceWs ? solidOnlyTool(t) : surfaceOnlyTool(t)) continue;
                        extras.push_back(&t);
                    }
                    group("ctxMore", "##wsMore", MZ_ICON_MORE, "More",
                          "Tools for the current selection", extras);
                }
            } else {
                // Sketch mode: the flat catalogue is ~19 buttons - a screen
                // and a half of scrolling. The DRAWING tools stay flat (they're
                // the constantly-switched core of sketching, and Steve wants
                // them one tap away); the occasional tools collapse into two
                // Fusion-style groups - Modify (trim/copy/mirror/patterns +
                // sketch plugins) and More (guides/measure/look-at). Select,
                // the draw tools and the active tool's origin toggle render
                // in catalogue order, so anything unrecognised (future tools)
                // also lands flat and can't silently vanish from the rail.
                std::vector<const Toolbar::RailTool*> modify, aids, importers;
                int railIdx = 0;
                for (const auto& t : tools) {
                    if (t.pluginIndex >= 0) { modify.push_back(&t); continue; }
                    switch (t.action) {
                    // Bringing outside artwork in is its own act, distinct from
                    // drawing or modifying: SVG outlines and aerofoil sections
                    // share a group so neither is buried in the flat rail.
                    case ToolAction::SketchSvg:
                    case ToolAction::SketchAirfoil:
                        importers.push_back(&t); break;
                    case ToolAction::Trim:
                    case ToolAction::SketchOffset:
                    case ToolAction::SketchFill:
                    case ToolAction::SketchCopy:
                    case ToolAction::SketchMirror:
                    case ToolAction::SketchLinearPattern:
                    case ToolAction::SketchRadialPattern:
                        modify.push_back(&t); break;
                    case ToolAction::SketchCycleInference:
                    case ToolAction::Measure:
                    case ToolAction::LookAtSketch:
                        aids.push_back(&t); break;
                    default:
                        flatButton(t, railIdx++); break;
                    }
                }
                group("modifyGroup", "##sketchModify", MZ_ICON_TRIM, "Modify",
                      "Modify tools: trim, offset, fill, copy, mirror, linear "
                      "and circular patterns",
                      modify);
                group("importGroup", "##sketchImport", MZ_ICON_SVG, "Import",
                      "Bring outside artwork into the sketch: an SVG outline or "
                      "an aerofoil section",
                      importers);
                group("aidsGroup", "##sketchAids", MZ_ICON_MORE, "More",
                      "Drawing guides level, measure, look at the sketch plane",
                      aids);
            }
        }
    }
    ImGui::End();
    ImGui::PopStyleColor();

    renderPendingCommandBar();   // fork: command-first rail

    // ── Bottom-right corner: in a sketch, the commit actions - an accent
    //    ✓ Finish FAB with a smaller ✗ Discard beside it (gap so a Finish tap
    //    can't stray onto Discard). During a live ACTION (push/pull, extrude,
    //    fillet, shell, pattern, …) the same pair reads Apply/Cancel and
    //    drives the action - the op panels hide their own Confirm/Cancel
    //    rows while this corner hosts them (imTouchActionCorner()).
    //    Everywhere else, the "+" create FAB. Commit actions used to live at
    //    the bottom of the left tool bar, which coupled "done sketching" to
    //    transient tool picking and put the throw-it-away button 8 px under
    //    the tools; the corner FAB spot is the tablet thumb zone, and the +
    //    menu is dead weight mid-sketch anyway.
    ImGui::SetNextWindowPos(ImVec2(wp.x + ws.x - m, wp.y + ws.y - m),
                            ImGuiCond_Always, ImVec2(1.0f, 1.0f));
    ImGui::SetNextWindowBgAlpha(0.0f);
    if (m_inSketchMode) {
        if (ImGui::Begin("##LiteFab", nullptr, kFloat)) {
            const bool toolRunning = m_sketchTool && m_sketchTool->isPlacing();
            const float fabD = 56.0f * s;
            const float side = 44.0f * s;
            // ✗ first (left), vertically centred on the FAB.
            const float topY = ImGui::GetCursorPosY();
            ImGui::SetCursorPosY(topY + (fabD - side) * 0.5f);
            // ↶ Back (Backspace twin) while a line chain / spline is in progress.
            if (toolRunning &&
                ((m_sketchTool->getMode() == SketchToolMode::Line &&
                  m_sketchTool->lineSegmentCount() >= 1) ||
                 (m_sketchTool->getMode() == SketchToolMode::Spline &&
                  !m_sketchTool->splinePointsInProgress().empty()))) {
                if (touchui::iconButton("backSk", MZ_ICON_UNDO, side))
                    sketchChainBack();
                tip(materializr::tr("Remove the last segment and keep drawing"));
                ImGui::SameLine(0.0f, 18.0f * s);
                ImGui::SetCursorPosY(topY + (fabD - side) * 0.5f);
            }
            if (touchui::iconButton("exitSk", MZ_ICON_DISCARD, side)) {
                if (toolRunning)
                    m_sketchTool->onCancel();
                else
                    ImGui::OpenPopup("Discard sketch?"); // confirm - destructive
            }
            tip(materializr::tr(toolRunning
                    ? "Cancel the in-progress shape"
                    : "Throw the sketch away and leave (asks to confirm)"));
            ImGui::SameLine(0.0f, 18.0f * s); // deliberate gap: no stray discards
            ImGui::SetCursorPosY(topY);
            if (touchui::fab("finishSk", MZ_ICON_FINISH, fabD)) {
                if (m_sketchShapeConfirmPending && toolRunning) {
                    // A circle held for its ✗/✓ bubble: commit it as-released.
                    // (onConfirm would RESET the in-flight placement instead.)
                    recordSketchMutation([&]{
                        m_sketchTool->onMouseDown(m_sketchShapePendingPos,
                                                  false); });
                    m_sketchShapeConfirmPending = false;
                } else if (toolRunning)
                    recordSketchMutation([&]{ m_sketchTool->onConfirm(); });
                else
                    handleToolAction(static_cast<int>(ToolAction::FinishSketch));
            }
            tip(materializr::tr(toolRunning
                    ? "Finish the current shape, keeping the points placed"
                    : "Finish the sketch and leave sketch mode"));
            if (ImGui::BeginPopupModal("Discard sketch?", nullptr,
                                       ImGuiWindowFlags_AlwaysAutoResize)) {
                ImGui::TextUnformatted(
                    materializr::tr("Leave the sketch and throw away its changes?"));
                ImGui::Spacing();
                const float bw = 150.0f * s;
                if (ImGui::Button(materializr::tr("Discard Sketch"), ImVec2(bw, 44.0f * s))) {
                    ImGui::CloseCurrentPopup();
                    handleToolAction(static_cast<int>(ToolAction::ExitSketchDiscard));
                }
                ImGui::SameLine();
                if (ImGui::Button(materializr::tr("Keep Editing"), ImVec2(bw, 44.0f * s)))
                    ImGui::CloseCurrentPopup();
                ImGui::EndPopup();
            }
        }
        ImGui::End();
    } else if (imTouchActionCorner()) {
        if (ImGui::Begin("##LiteFab", nullptr, kFloat)) {
            const float fabD = 56.0f * s;
            const float side = 44.0f * s;
            // ✗ first (left), vertically centred on the ✓ FAB - mirrors the
            // sketch pair so the corner always means confirm/discard.
            const float topY = ImGui::GetCursorPosY();
            ImGui::SetCursorPosY(topY + (fabD - side) * 0.5f);
            if (touchui::iconButton("cancelAct", MZ_ICON_DISCARD, side))
                cancelActiveAction();
            tip(materializr::tr("Cancel the action and discard its preview"));
            ImGui::SameLine(0.0f, 18.0f * s); // deliberate gap: no stray cancels
            ImGui::SetCursorPosY(topY);
            if (touchui::fab("applyAct", MZ_ICON_FINISH, fabD))
                confirmActiveAction();
            tip(materializr::tr("Apply the action"));
        }
        ImGui::End();
    } else {
        if (ImGui::Begin("##LiteFab", nullptr, kFloat)) {
            if (touchui::fab("create", MZ_ICON_ADD))
                ImGui::OpenPopup("##LiteCreate");
            tip(materializr::tr("Create: a sketch or a primitive solid"));
            if (ImGui::BeginPopup("##LiteCreate")) {
                // Mirror the modern rail's create logic instead of dumping every
                // option flat: sketch is contextual (on a picked face/plane if there
                // is one, else a "New Sketch" submenu of world planes), the five
                // primitives live under ONE "Primitive" submenu, and construction
                // geometry derives from the selection - so only the relevant, grouped
                // create tools show, matching the classic + modern layouts.
                const bool faceOrPlaneSel = m_selection &&
                    (m_selection->hasSelectedFaces() ||
                     m_selection->primaryType() == SelectionType::Plane);
                if (faceOrPlaneSel) {
                    if (ImGui::MenuItem((std::string(MZ_ICON_SKETCH "  ") +
                                      materializr::tr("Sketch on selection") +
                                      "###SketchOnSelection").c_str()))
                        handleToolAction(static_cast<int>(ToolAction::SketchOnFace));
                } else if (ImGui::BeginMenu((std::string(MZ_ICON_SKETCH "  ") +
                                           materializr::tr("New Sketch") +
                                           "###NewSketchMenu").c_str())) {
                    if (ImGui::MenuItem(materializr::tr("XY plane")))
                        handleToolAction(static_cast<int>(ToolAction::StartSketchXY));
                    if (ImGui::MenuItem(materializr::tr("XZ plane")))
                        handleToolAction(static_cast<int>(ToolAction::StartSketchXZ));
                    if (ImGui::MenuItem(materializr::tr("YZ plane")))
                        handleToolAction(static_cast<int>(ToolAction::StartSketchYZ));
                    ImGui::EndMenu();
                }
                if (m_pluginContext &&
                    ImGui::BeginMenu((std::string(MZ_ICON_PRIMITIVE "  ") +
                                    materializr::tr("Primitive") +
                                    "###PrimitiveMenu").c_str())) {
                    if (ImGui::MenuItem(materializr::tr("Box")))
                        m_pluginContext->requestInteractiveOp(InteractiveOp::PrimitiveBox);
                    if (ImGui::MenuItem(materializr::tr("Cylinder")))
                        m_pluginContext->requestInteractiveOp(InteractiveOp::PrimitiveCylinder);
                    if (ImGui::MenuItem(materializr::tr("Sphere")))
                        m_pluginContext->requestInteractiveOp(InteractiveOp::PrimitiveSphere);
                    if (ImGui::MenuItem(materializr::tr("Cone")))
                        m_pluginContext->requestInteractiveOp(InteractiveOp::PrimitiveCone);
                    if (ImGui::MenuItem(materializr::tr("Torus")))
                        m_pluginContext->requestInteractiveOp(InteractiveOp::PrimitiveTorus);
                    ImGui::EndMenu();
                }
                if (ImGui::BeginMenu((std::string(MZ_ICON_FOCUS "  ") +
                                      materializr::tr("Construction") +
                                      "###ConstructionMenu").c_str())) {
                    renderConstructionMenuItems();
                    ImGui::EndMenu();
                }
                ImGui::EndPopup();
            }
        }
        ImGui::End();
    }

    // ── History: a bottom toggle whose REOPEN button sits exactly where its
    //    minimize chevron is (not up on the Items rail). The toggle is a fixed
    //    left-anchored button - a chevron to hide while the strip is open, the
    //    History clock to reopen while collapsed - and the Fusion-360-style
    //    step strip (tap a box for its properties popup: edit params, roll
    //    to it, toggle/delete) is a separate scrolling window to its right.
    //    Hidden in sketch mode: rolling the host body back under a live sketch
    //    is forbidden (see History::setUndoFloor).
    const int steps = m_history ? m_history->stepCount() : 0;
    const bool histAvail = !m_inSketchMode && m_document && m_history;
    const float histX   = wp.x + m;   // bottom-left corner (fps moved to top)
    const float histGap = 8.0f * s;
    // Last frame's measured strip height, so the toggle can be centred on the
    // strip's vertical middle - the strip window carries padding the
    // borderless button doesn't, so plain bottom-alignment left it sitting low.
    // The height persists once the strip has rendered, so the button holds that
    // same centred position when collapsed instead of snapping back down.
    static float s_histStripH = 0.0f;
    if (histAvail) {
        if (s_histStripH > 0.0f)
            ImGui::SetNextWindowPos(
                ImVec2(histX, (wp.y + ws.y - m) - s_histStripH * 0.5f),
                ImGuiCond_Always, ImVec2(0.0f, 0.5f));
        else
            ImGui::SetNextWindowPos(ImVec2(histX, wp.y + ws.y - m),
                                    ImGuiCond_Always, ImVec2(0.0f, 1.0f));
        ImGui::SetNextWindowBgAlpha(0.0f);   // the button draws its own solid fill
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
        if (ImGui::Begin("##LiteHistoryToggle", nullptr, kFloat)) {
            // Clock icon + "History" label - mirrors the Items button, and
            // accent-fills while the timeline is open. The button is the same
            // open or closed, so the reopen state sits exactly where the
            // collapse state was.
            if (touchui::railButton("histToggle", MZ_ICON_HISTORY, materializr::tr("History"),
                                    m_imTouchTimeline, railBtnW, /*solid=*/true)) {
                m_imTouchTimeline = !m_imTouchTimeline;
                saveAppSettings();
            }
            tip(materializr::tr(m_imTouchTimeline ? "Hide the history timeline"
                                                  : "Show the history timeline"));
        }
        ImGui::End();
        ImGui::PopStyleVar();
    }
    // Fork: close the step side panel; `restore` puts the op's parameters
    // back to their values when the panel opened (Cancel / switching steps).
    auto closeStepEdit = [&](bool restore) {
        if (m_imTouchHistoryEdit >= 0 && restore && !m_imTouchStepBackup.empty() &&
            m_history) {
            if (const Operation* op = m_history->getStep(m_imTouchHistoryEdit))
                const_cast<Operation*>(op)->deserializeParams(m_imTouchStepBackup);
        }
        m_imTouchHistoryEdit = -1;
        m_imTouchStepBackup.clear();
        if (m_historyPanel) m_historyPanel->setEditingStep(-1);
    };
    if (m_imTouchTimeline && histAvail) {
        const float stripX = histX + railBtnW + histGap;
        ImGui::SetNextWindowPos(ImVec2(stripX, wp.y + ws.y - m),
                                ImGuiCond_Always, ImVec2(0.0f, 1.0f));
        // Always reserve room for the WIDEST thing the bottom-right corner can
        // host - the ✗+gap+✓ Cancel/Apply pair (44+18+56 = 118dp) - not just
        // the single "+" create FAB (56dp). Sizing to the FAB let the expanded
        // strip run under the Cancel ✗ during a live action (drawn OVER it on
        // Android, UNDER on Linux, purely by z-order - #20). Reserving the pair
        // width unconditionally keeps the strip's right edge fixed instead of
        // popping in/out as the corner button changes. +20 = the FAB window's
        // own padding plus a small visual gap.
        const float fabClear = m + (44.0f + 18.0f + 56.0f + 20.0f) * s;
        ImGui::SetNextWindowSizeConstraints(
            ImVec2(0, 0), ImVec2((wp.x + ws.x - fabClear) - stripX, FLT_MAX));
        ImGui::SetNextWindowBgAlpha(0.92f);
        ImGui::PushStyleColor(ImGuiCol_WindowBg, touchui::panelBg());
        if (ImGui::Begin("##LiteTimeline", nullptr,
                         (kFloat & ~ImGuiWindowFlags_NoScrollbar) |
                             ImGuiWindowFlags_HorizontalScrollbar)) {
            s_histStripH = ImGui::GetWindowSize().y;  // for centring the toggle
            // Empty history still shows a hint so toggling History in a fresh
            // project doesn't look like it did nothing.
            if (steps == 0)
                ImGui::TextColored(touchui::textDim(), "%s", materializr::tr("History: no steps yet"));
            const int curr = m_history->currentStep();
            const int failedAt = m_history->lastReplayFailure();
            const bool histLocked = anyInteractivePreviewActive();
            const ImU32 amber = ImGui::GetColorU32(ImVec4(0.95f, 0.75f, 0.3f, 1.0f));
            const ImU32 red   = ImGui::GetColorU32(ImVec4(1.0f, 0.45f, 0.35f, 1.0f));

            // Auto-scroll the current step into view whenever history mutates
            // (new op, undo/redo, edit) - not on user scrolls.
            static unsigned s_seenRev = ~0u;
            const bool historyMoved = (s_seenRev != m_history->revision());

            // Fork: Fusion-style timeline. A tap opens the step the way it
            // was built - a sketch step enters that sketch, a fillet/chamfer
            // re-opens its interactive tool - and any other step gets a small
            // side panel with only its parameters (OK / Cancel). Go Here,
            // Enable/Disable and Delete moved to a long press on the step.
            static int s_longPressStep = -1;  // step whose long press opened the menu
            static int s_menuStep = -1;
            if (!ImGui::IsMouseDown(ImGuiMouseButton_Left) &&
                !ImGui::IsMouseReleased(ImGuiMouseButton_Left))
                s_longPressStep = -1;
            int tapStep = -1;
            bool openMenu = false;
            for (int i = 0; i < steps; ++i) {
                const Operation* op = m_history->getStep(i);
                if (!op) continue;
                if (i > 0) ImGui::SameLine(0.0f, 6.0f * s);
                ImGui::PushID(i);
                ImU32 tint = 0;
                if (i == failedAt)          tint = red;
                else if (op->isReloaded())  tint = amber;
                const bool dim = (i > curr) || !op->isEnabled();
                std::string stepName = op->name();
                if (stepName.empty()) stepName = op->typeId();
                const bool tapped = touchui::timelineBox(
                    "step", stepIcon(op->typeId()), i == curr,
                    i == m_imTouchHistoryEdit, dim, tint, 0.0f, stepName.c_str());
                // Long press: finger held still on the step for half a second
                // (or a right-click, should the window's synthetic one land here).
                if (ImGui::IsItemActive() && s_longPressStep != i &&
                    ImGui::GetIO().MouseDownDuration[ImGuiMouseButton_Left] > 0.5f &&
                    !ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
                    s_longPressStep = i;
                    s_menuStep = i;
                    openMenu = true;
                } else if (ImGui::IsItemClicked(ImGuiMouseButton_Right)) {
                    s_menuStep = i;
                    openMenu = true;
                }
                if (tapped) {
                    if (s_longPressStep == i) s_longPressStep = -1; // its release
                    else tapStep = i;
                }
                if (i == curr && historyMoved) ImGui::SetScrollHereX(0.5f);
                ImGui::PopID();
            }
            s_seenRev = m_history->revision();

            if (tapStep >= 0 && !histLocked) {
                const int i = tapStep;
                const Operation* op = m_history->getStep(i);
                const bool same = (i == m_imTouchHistoryEdit);
                closeStepEdit(/*restore=*/true);
                bool handled = same || !op;   // a second tap closes the panel
                if (!handled && op->isReloaded()) {
                    showToast(materializr::tr("Restored from an older save - no editable parameters. Undo/redo still work."));
                    handled = true;
                }
                if (!handled) {
                    if (auto* se = dynamic_cast<const SketchEditOp*>(op)) {
                        auto tgt = se->getTarget();
                        const int sid =
                            tgt ? m_document->findSketchId(tgt.get()) : -1;
                        if (sid >= 0) {
                            editSketch(sid);
                            handled = true;
                        }
                    }
                }
                if (!handled && op->isEnabled() && i <= curr &&
                    (op->kind() == Operation::Kind::Fillet ||
                     op->kind() == Operation::Kind::Chamfer)) {
                    m_edgeOpPickedBodyId = -1;
                    beginInteractiveEdgeOpEdit(i);
                    handled = m_edgeCtl.active();
                }
                if (!handled && op->isEnabled() && i <= curr &&
                    (op->typeId() == "extrude" || op->typeId() == "surface_extrude"))
                    handled = beginInteractiveExtrudeEdit(i);
                if (!handled) {
                    m_imTouchHistoryEdit = i;
                    m_imTouchStepBackup = op->serializeParams();
                    if (m_historyPanel) m_historyPanel->setEditingStep(i);
                }
            }

            if (openMenu) ImGui::OpenPopup("##LiteStepMenu");
            if (ImGui::BeginPopup("##LiteStepMenu")) {
                const Operation* op = (s_menuStep >= 0 && s_menuStep < steps)
                                          ? m_history->getStep(s_menuStep)
                                          : nullptr;
                if (!op) {
                    ImGui::CloseCurrentPopup();
                } else {
                    const int i = s_menuStep;
                    const ImVec2 bsz(180.0f * s, 44.0f * s);
                    ImGui::BeginDisabled(histLocked || i == curr);
                    if (ImGui::Button(materializr::tr("Go Here"), bsz)) {
                        closeStepEdit(/*restore=*/true);
                        // Roll the model to this step (Fusion's marker drag).
                        // Progress guard: a failed replay mid-walk must not
                        // spin forever.
                        while (m_history->currentStep() > i) {
                            const int before = m_history->currentStep();
                            undoWithCascade();
                            if (m_history->currentStep() == before) break;
                        }
                        // redoWithCascade() opens its own per-step scope.
                        while (m_history->currentStep() < i) {
                            const int before = m_history->currentStep();
                            redoWithCascade();
                            if (m_history->currentStep() == before) break;
                        }
                        ImGui::CloseCurrentPopup();
                    }
                    ImGui::EndDisabled();
                    ImGui::BeginDisabled(histLocked);
                    if (ImGui::Button(materializr::tr(op->isEnabled() ? "Disable" : "Enable"), bsz)) {
                        closeStepEdit(/*restore=*/true);
                        // In-place toggle - preserves base bodies the op
                        // modifies (replayAll's doc.clear() would drop them).
                        auto trackBodies = trackBodyChanges();
                        m_history->setStepEnabled(i, !op->isEnabled(),
                                                  *m_document);
                        ImGui::CloseCurrentPopup();
                    }
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.68f, 0.24f, 0.24f, 0.97f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.82f, 0.34f, 0.34f, 1.0f));
                    const bool del = ImGui::Button(materializr::tr("Delete"), bsz);
                    ImGui::PopStyleColor(2);
                    if (del) {
                        closeStepEdit(/*restore=*/true);
                        auto trackBodies = trackBodyChanges();
                        if (!m_history->removeStep(i, *m_document))
                            showToast(materializr::tr(
                                "Can't delete: a later operation depends on it."));
                        ImGui::CloseCurrentPopup();
                    }
                    ImGui::EndDisabled();
                }
                ImGui::EndPopup();
            }
        }
        ImGui::End();
        ImGui::PopStyleColor();
    }

    // Fork: side panel for the step being edited - only the op's parameter
    // fields and OK / Cancel. OK replays history from the step; Cancel puts
    // the parameters back as they were when the panel opened.
    if (histAvail && m_imTouchHistoryEdit >= 0) {
        const int i = m_imTouchHistoryEdit;
        const Operation* op = (i < steps) ? m_history->getStep(i) : nullptr;
        if (!op) {
            closeStepEdit(/*restore=*/false);
        } else {
            const bool histLocked = anyInteractivePreviewActive();
            const float panelW = 300.0f * s;
            const float top = materializr::viewportTopChromeBottom() + 120.0f * s;
            const float maxH = std::max(200.0f * s, (wp.y + ws.y) - top - 140.0f * s);
            ImGui::SetNextWindowPos(ImVec2(wp.x + ws.x - m, top), ImGuiCond_Always,
                                    ImVec2(1.0f, 0.0f));
            ImGui::SetNextWindowSizeConstraints(ImVec2(panelW, 0.0f), ImVec2(panelW, maxH));
            ImGui::SetNextWindowBgAlpha(0.95f);
            ImGui::PushStyleColor(ImGuiCol_WindowBg, touchui::panelBg());
            bool ok = false, cancel = false;
            if (ImGui::Begin("##LiteStepEdit", nullptr,
                             kFloat & ~ImGuiWindowFlags_NoScrollbar)) {
                if (i == m_history->lastReplayFailure())
                    ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.35f, 1.0f), "%s",
                                       materializr::tr("Failed to recompute"));
                ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x * 0.5f);
                const_cast<Operation*>(op)->renderProperties();
                ImGui::PopItemWidth();
                ImGui::Spacing();
                const float gap = ImGui::GetStyle().ItemSpacing.x;
                const ImVec2 bsz((ImGui::GetContentRegionAvail().x - gap) * 0.5f, 44.0f * s);
                cancel = ImGui::Button(materializr::tr("Cancel"), bsz);
                ImGui::SameLine();
                ImGui::BeginDisabled(histLocked);
                ImGui::PushStyleColor(ImGuiCol_Button, touchui::accentFill());
                ok = ImGui::Button(materializr::tr("OK"), bsz);
                ImGui::PopStyleColor();
                ImGui::EndDisabled();
            }
            ImGui::End();
            ImGui::PopStyleColor();
            if (ok) {
                // Same sequence as HistoryPanel: carry inline sketch-dimension
                // edits into later snapshots FIRST, then a transactional
                // replay, then cascade so bodies built from the sketch follow.
                auto trackBodies = trackBodyChanges();
                m_history->propagateSketchValueEdits(i, *m_document);
                const std::string paramsBefore = m_imTouchStepBackup;
                const bool applied =
                    m_history->editStep(i, *m_document, /*transactional=*/true);
                if (applied) {
                    if (!paramsBefore.empty()) recordParamEdit({{i, paramsBefore}});
                    if (auto* se = dynamic_cast<const SketchEditOp*>(op)) {
                        auto tgt = se->getTarget();
                        int sid = tgt ? m_document->findSketchId(tgt.get()) : -1;
                        if (sid >= 0) cascadeFromSketchEdit(sid);
                    } else if (auto* st = dynamic_cast<const SketchTransformOp*>(op)) {
                        if (st->getSketchId() >= 0)
                            cascadeFromSketchEdit(st->getSketchId());
                    }
                    closeStepEdit(/*restore=*/false);
                } else {
                    // A rejected edit was rolled back by History; keep the
                    // panel so the values can be corrected.
                    showToast(materializr::tr("Couldn't recompute after an upstream change. Edit its parameters, fix the step before it, or delete it."));
                }
            } else if (cancel) {
                closeStepEdit(/*restore=*/true);
            }
        }
    }

    // ── fps readout - a small solid chip at the top-centre. Hidden entirely
    //    via Settings → Appearance → "Show FPS counter".
    if (m_showFps) {
        ImGui::SetNextWindowPos(ImVec2(wp.x + ws.x * 0.5f, wp.y + m),
                                ImGuiCond_Always, ImVec2(0.5f, 0.0f));
        ImGui::SetNextWindowBgAlpha(0.92f);
        ImGui::PushStyleColor(ImGuiCol_WindowBg, touchui::panelBg());
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(9.0f * s, 4.0f * s));
        if (ImGui::Begin("##LiteFps", nullptr, kFloat)) {
            ImGui::SetWindowFontScale(0.82f);   // smaller than the shell text
            ImGui::TextColored(touchui::textDim(), materializr::tr("%.0f fps"),
                               ImGui::GetIO().Framerate);
            ImGui::SetWindowFontScale(1.0f);
        }
        ImGui::End();
        ImGui::PopStyleVar();
        ImGui::PopStyleColor();
    }

    ImGui::PopStyleVar(2); // WindowRounding + WindowBorderSize
}

// ── Fork: command-first workspace rail ──────────────────────────────────────

bool Application::pendingAllows(SelectionType t) const {
    return !m_pendingCmd.active ||
           (m_pendingCmd.typeMask & (1u << static_cast<unsigned>(t))) != 0;
}

int Application::pendingMatchCount() const {
    if (!m_selection) return 0;
    int n = 0;
    for (const auto& e : m_selection->getSelection())
        if (m_pendingCmd.typeMask & (1u << static_cast<unsigned>(e.type))) ++n;
    return n;
}

void Application::startWorkspaceCommand(const PendingCommand& cmd) {
    cancelPendingCommand();
    m_pendingCmd = cmd;
    m_pendingCmd.active = true;
    if (cmd.minCount <= 0) { firePendingCommand(); return; }
    // Keep only what this command takes; a matching preselection runs it at
    // once (Fusion accepts a preselection too).
    if (m_selection) {
        std::vector<SelectionEntry> drop;
        for (const auto& e : m_selection->getSelection())
            if (!(cmd.typeMask & (1u << static_cast<unsigned>(e.type))))
                drop.push_back(e);
        for (const auto& e : drop) m_selection->removeFromSelection(e);
        m_selection->setNavigationOnly(false);
    }
    if (pendingMatchCount() >= cmd.minCount) firePendingCommand();
}

void Application::firePendingCommand() {
    const PendingCommand c = m_pendingCmd;
    cancelPendingCommand();
    if (c.plugin) {
        if (!m_toolbar) return;
        const auto& contribs = PluginRegistry::instance().toolbarContributions();
        for (size_t i = 0; i < contribs.size(); ++i)
            if (contribs[i].name == c.plugin) {
                m_toolbar->fireRailPlugin(static_cast<int>(i));
                return;
            }
        return;
    }
    handleToolAction(c.action);
}

// The waiting command's bar, top-centre under the chrome: its name, the pick
// count, Cancel and OK. Sketch also offers the three origin planes.
void Application::renderPendingCommandBar() {
    if (!m_pendingCmd.active) return;
    if (m_inSketchMode || anyInteractivePreviewActive()) {
        cancelPendingCommand();
        return;
    }
    const int count = pendingMatchCount();
    if (m_pendingCmd.single && count >= 1) {
        firePendingCommand();
        return;
    }
    const float s = uiScale();
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    const float m = 12.0f * s;
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + vp->WorkSize.x * 0.5f,
                                   vp->WorkPos.y + m + 52.0f * s),
                            ImGuiCond_Always, ImVec2(0.5f, 0.0f));
    ImGui::SetNextWindowBgAlpha(0.95f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, touchui::panelBg());
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f * s, 8.0f * s));
    const ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings |
        ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNavFocus;
    bool ok = false, cancel = false;
    int originPlane = -1;
    if (ImGui::Begin("##PendingCmd", nullptr, flags)) {
        const float side = 44.0f * s;
        const float textY = ImGui::GetCursorPosY() + (side - ImGui::GetFontSize()) * 0.5f;
        ImGui::SetCursorPosY(textY);
        ImGui::TextColored(touchui::textPrimary(), "%s", materializr::tr(m_pendingCmd.label));
        ImGui::SameLine(0.0f, 12.0f * s);
        ImGui::SetCursorPosY(textY);
        ImGui::TextColored(touchui::textDim(), materializr::tr("%d selected"), count);
        ImGui::SameLine(0.0f, 16.0f * s);
        ImGui::SetCursorPosY(textY - (side - ImGui::GetFontSize()) * 0.5f);
        if (m_pendingCmd.action == static_cast<int>(ToolAction::SketchOnFace)) {
            static const char* const kPlanes[3] = { "XY", "XZ", "YZ" };
            for (int i = 0; i < 3; ++i) {
                if (ImGui::Button(kPlanes[i], ImVec2(side * 1.2f, side))) originPlane = i;
                ImGui::SameLine(0.0f, 6.0f * s);
            }
            ImGui::SameLine(0.0f, 10.0f * s);
        }
        cancel = touchui::iconButton("pcCancel", MZ_ICON_DISCARD, side);
        if (!m_pendingCmd.single) {
            ImGui::SameLine(0.0f, 10.0f * s);
            ImGui::BeginDisabled(count < m_pendingCmd.minCount);
            ok = touchui::iconButton("pcOk", MZ_ICON_FINISH, side);
            ImGui::EndDisabled();
        }
    }
    ImGui::End();
    ImGui::PopStyleVar();
    ImGui::PopStyleColor();
    if (originPlane >= 0) {
        cancelPendingCommand();
        const ToolAction a = originPlane == 0 ? ToolAction::StartSketchXY
                           : originPlane == 1 ? ToolAction::StartSketchXZ
                                              : ToolAction::StartSketchYZ;
        handleToolAction(static_cast<int>(a));
    } else if (cancel) {
        cancelPendingCommand();
    } else if (ok) {
        firePendingCommand();
    }
}

} // namespace materializr
