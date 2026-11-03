#include "ui/LengthField.h"
#include "ExtrudeController.h"
#include "TouchArrow.h"
#include "core/BodyChanges.h"
#include "../core/Document.h"
#include "../core/History.h"
#include "../core/Operation.h"
#include "../modeling/SurfaceExtrudeOp.h"
#include "../modeling/Sketch.h"
#include "../modeling/SurfaceOps.h"
#include "../ui_scale.h"
#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include "../core/SelectionManager.h"
#include "../modeling/CutTargetPick.h"
#include "../core/NumParse.h"
#include "../ui/UiTheme.h"       // viewportBanner
#include "../ui/NumField.h"      // btnConfirm / btnCancel
#include "../ui/StepperRow.h"
#include "../ui/TouchWidgets.h"  // im-touch number-pad amount field
#include "../ui/TouchIcons.h"    // MZ_ICON_BODY - the all-bodies pill
#include "../ui/OpDialogGrip.h"
#include "../touch_mode.h"
#include <imgui.h>
#include <BRep_Tool.hxx>
#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp_Face.hxx>
#include <Bnd_Box.hxx>
#include <Geom_Plane.hxx>
#include <Geom_Surface.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <cmath>
#include <cstdio>
#include <utility>
#include <vector>
#include "../i18n.h"
#include "../i18n.h"
#include "../i18n.h"
#include "../i18n.h"

namespace materializr {

double ExtrudeController::opDistance() const {
    return m_sweepSign * static_cast<double>(m_distance);
}

// Every visible, non-mesh body except the preview's own tool volume. Imported
// meshes decline modelling ops elsewhere and must not become a cut target here
// either.
static std::vector<std::pair<int, TopoDS_Shape>> cutCandidates(
        const IopContext& ctx, int excludeBody) {
    std::vector<std::pair<int, TopoDS_Shape>> out;
    for (int id : ctx.doc.getAllBodyIds()) {
        if (id == excludeBody) continue;
        if (!ctx.doc.isBodyVisible(id) || ctx.doc.isBodyMesh(id)) continue;
        try {
            const TopoDS_Shape& s = ctx.doc.getBody(id);
            if (!s.IsNull()) out.push_back({id, s});
        } catch (...) {}
    }
    return out;
}

int ExtrudeController::resolveCutTarget(const IopContext& ctx) const {
    // The live preview IS the tool volume - the exact solid the user is
    // watching - so ask which bodies it overlaps rather than re-deriving it.
    const int previewId = previewBodyId();
    if (previewId < 0) {
        std::fprintf(stderr, "[Subtract] no tool volume to cut with "
                             "(preview not applied)\n");
        return -1;
    }
    TopoDS_Shape tool;
    try { tool = ctx.doc.getBody(previewId); } catch (...) { return -1; }
    if (tool.IsNull()) return -1;
    const auto cands = cutCandidates(ctx, previewId);
    const int hit = cutpick::pickCutTarget(cands, tool, m_targetBody);
    if (hit < 0) {
        // Say what was actually measured - "nothing to cut" is a claim about
        // geometry, and a wrong one is invisible without the numbers (this
        // dump is what turned "but it CLEARLY overlaps" into "the tool spans
        // x -18..-6 and every body starts at 0"). Re-running the booleans to
        // report them is fine: only the refusal path gets here, and the user
        // is already stopped.
        std::fprintf(stderr, "[Subtract] tool body %d (dist %.3f) reaches no "
                     "body; preferred=%d, checked %zu:\n",
                     previewId, opDistance(), m_targetBody, cands.size());
        auto bbox = [](const TopoDS_Shape& s, double* v) {
            Bnd_Box b; BRepBndLib::Add(s, b);
            if (b.IsVoid()) { for (int i = 0; i < 6; ++i) v[i] = 0; return; }
            b.Get(v[0], v[1], v[2], v[3], v[4], v[5]);
        };
        double t[6]; bbox(tool, t);
        std::fprintf(stderr, "  tool bbox [%.2f %.2f %.2f]..[%.2f %.2f %.2f]\n",
                     t[0], t[1], t[2], t[3], t[4], t[5]);
        for (const auto& [id, shape] : cands) {
            double b[6]; bbox(shape, b);
            std::fprintf(stderr, "  body %d overlap %.6g  bbox [%.2f %.2f %.2f]"
                         "..[%.2f %.2f %.2f]\n", id,
                         cutpick::removedVolume(shape, tool),
                         b[0], b[1], b[2], b[3], b[4], b[5]);
        }
    }
    return hit;
}

bool ExtrudeController::beginExtrude(const IopContext& ctx,
                                     const TopoDS_Shape& profile,
                                     ExtrudeMode mode, int targetBody,
                                     int sourceSketchId) {
    // Extrude sweeps a profile along its normal - only meaningful for a FLAT
    // profile. A single curved body face (cylinder / sphere / fillet) has no
    // single normal, so extruding it produced garbage geometry; refuse with
    // guidance instead (mirrors Sketch-on-Face). Checked BEFORE anything is
    // disturbed so a bad attempt leaves an in-progress op alone. Sketch
    // profiles are planar by construction; wire / compound profiles aren't a
    // single face and skip this check.
    if (profile.ShapeType() == TopAbs_FACE) {
        Handle(Geom_Surface) s = BRep_Tool::Surface(TopoDS::Face(profile));
        if (s.IsNull() || !s->IsKind(STANDARD_TYPE(Geom_Plane))) {
            if (ctx.toast)
                ctx.toast(materializr::tr("Can't extrude a curved face - extrude "
                                          "works on flat faces only."));
            return false;
        }
    }
    m_profile = profile;
    m_mode = mode;
    m_targetBody = targetBody;
    m_sketchId = sourceSketchId;
    if (!begin(ctx)) return false;
    // The Profiles field starts armed (Fusion): taps add / remove profiles.
    if (!m_restarting) {
        if (m_picks.empty()) seedPicksFromProfile(ctx);
        m_picking = true;
        syncSelection(ctx);
    }
    return true;
}

bool ExtrudeController::beginSurfaceExtrude(const IopContext& ctx, int sketchId) {
    auto sk = ctx.doc.getSketch(sketchId);
    if (!sk) return false;
    const std::vector<TopoDS_Wire> wires = sk->buildCurveWires();
    if (wires.empty()) {
        if (ctx.toast) ctx.toast(materializr::tr("This sketch has no curves to extrude."));
        return false;
    }
    BRep_Builder b;
    TopoDS_Compound c;
    b.MakeCompound(c);
    for (const TopoDS_Wire& w : wires) b.Add(c, w);
    m_profile = c;
    m_mode = ExtrudeMode::NewBody;
    m_targetBody = -1;
    m_sketchId = sketchId;
    m_surface = true;
    if (!begin(ctx)) { m_surface = false; return false; }
    return true;
}

bool ExtrudeController::beginExtrudeEdit(const IopContext& ctx, int historyIndex) {
    const auto* op = dynamic_cast<const ExtrudeOp*>(ctx.history.getStep(historyIndex));
    if (!op || op->getProfile().IsNull()) return false;
    m_profile = op->getProfile();
    m_mode = op->getMode();
    m_targetBody = op->getTargetBody();
    m_sketchId = op->getSketchId();
    m_origOpDistance = op->getDistance();
    m_origProfile = op->getProfile();
    m_origSketchId = op->getSketchId();
    m_profileEdited = false;
    m_editingIndex = historyIndex;
    m_surface = dynamic_cast<const SurfaceExtrudeOp*>(op) != nullptr;
    if (!begin(ctx)) {
        m_editingIndex = -1;
        return false;
    }
    // Profiles field armed from the start, holding the step's profile; the
    // step's sketch is usually hidden after the extrude - show it meanwhile.
    if (m_surface) return true;   // curves, no Profiles field
    seedPicksFromProfile(ctx);
    m_picking = !m_picks.empty();
    if (m_origSketchId >= 0 && !ctx.doc.isSketchVisible(m_origSketchId)) {
        ctx.doc.setSketchVisible(m_origSketchId, true);
        m_shownSketch = m_origSketchId;
    }
    syncSelection(ctx);
    return true;
}

void ExtrudeController::writeEditDistance(const IopContext& ctx, double d) const {
    auto* op = dynamic_cast<ExtrudeOp*>(
        const_cast<Operation*>(ctx.history.getStep(m_editingIndex)));
    if (op) op->setDistance(d);
}

void ExtrudeController::computeFrame(const IopContext& ctx) {
    // Face normal and centre. A compound profile (multi-region extrude -
    // several letters at once) uses its first face: all regions of one sketch
    // are coplanar, so any face gives the right normal.
    TopoDS_Shape normShape = m_profile;
    if (m_profile.ShapeType() != TopAbs_FACE) {
        TopExp_Explorer fx(m_profile, TopAbs_FACE);
        if (fx.More()) normShape = fx.Current();
    }
    if (normShape.ShapeType() == TopAbs_FACE) {
        BRepGProp_Face prop(TopoDS::Face(normShape));
        gp_Pnt center;
        gp_Vec norm;
        double u1, u2, v1, v2;
        prop.Bounds(u1, u2, v1, v2);
        prop.Normal((u1 + u2) * 0.5, (v1 + v2) * 0.5, center, norm);
        if (norm.Magnitude() > 1e-10)
            m_normal = glm::normalize(glm::vec3(norm.X(), norm.Y(), norm.Z()));
        m_origin = glm::vec3(center.X(), center.Y(), center.Z());
    } else if (m_sketchId >= 0) {
        // Curves only (Surface Extrude): the sketch plane gives the normal,
        // the curves' bounding-box centre the arrow's foot.
        if (auto sk = ctx.doc.getSketch(m_sketchId)) {
            const gp_Dir n = sk->getPlane().Position().Direction();
            m_normal = glm::vec3(n.X(), n.Y(), n.Z());
        }
        Bnd_Box bb;
        BRepBndLib::Add(m_profile, bb);
        if (!bb.IsVoid()) {
            double x0, y0, z0, x1, y1, z1;
            bb.Get(x0, y0, z0, x1, y1, z1);
            m_origin = glm::vec3((x0 + x1) * 0.5, (y0 + y1) * 0.5, (z0 + z1) * 0.5);
        }
    }
}

int ExtrudeController::onBegin(const IopContext& ctx) {
    m_distance = m_dragRaw = m_dragAppliedDist = 5.0f;
    materializr::formatLengthDigits(m_inputBuf, sizeof(m_inputBuf), m_distance);
    m_inputFocus = true;

    computeFrame(ctx);
    // Point the on-screen arrow INTO the material for a Subtract, so dragging
    // toward it deepens the cut. A face sketch gets that for free - its normal
    // points OUT of the host body, so the cut runs the other way. A sketch on a
    // construction or origin plane has no host and no such convention: its
    // normal points wherever the plane faces, which half the time is away from
    // every body, so aim at the nearest one instead.
    m_sweepSign = 1.0;
    if (m_editingIndex >= 0) {
        // Edit: the arrow starts at the op's own distance. A negative one
        // (a cut runs against the profile normal) flips the arrow so the
        // well shows a positive value, as when it was created.
        m_sweepSign = (m_origOpDistance < 0.0) ? -1.0 : 1.0;
        m_normal *= static_cast<float>(m_sweepSign);
        m_distance = m_dragRaw = m_dragAppliedDist =
            static_cast<float>(std::abs(m_origOpDistance));
        materializr::formatLengthDigits(m_inputBuf, sizeof(m_inputBuf), m_distance);
        // Whole document + every op's edit state, BEFORE the first replay.
        m_editPreview.begin(ctx.doc, ctx.history);
        ctx.selection.clear();
        return kNoTargetBody;
    }
    if (m_mode == ExtrudeMode::Subtract) {
        m_sweepSign = -1.0;
        if (m_targetBody < 0) {
            std::vector<TopoDS_Shape> bodies;
            for (const auto& [id, shape] : cutCandidates(ctx, -1))
                bodies.push_back(shape);
            m_sweepSign = cutpick::cutSweepSign(
                gp_Pnt(m_origin.x, m_origin.y, m_origin.z),
                gp_Dir(m_normal.x, m_normal.y, m_normal.z), bodies);
        }
        m_normal *= static_cast<float>(m_sweepSign);
    }

    // Threaded target bodies are fine: the preview is always a NewBody tool
    // volume (never a per-frame boolean against the target), and the real
    // Subtract runs once at commit through History::pushOperation, which
    // reflows the cut beneath the Thread step and re-cuts in background.
    return kNoTargetBody;   // the preview body doesn't exist yet
}

std::unique_ptr<Operation> ExtrudeController::buildOp(const IopContext& ctx) {
    (void)ctx;
    // The live instance. Always NewBody - the user watches the tool volume
    // being swept; Subtract's real boolean is buildCommitOp's job.
    if (m_surface) {
        auto sop = std::make_unique<SurfaceExtrudeOp>();
        sop->setDistance(opDistance());
        sop->setSketchSource(m_sketchId);
        return sop;
    }
    auto op = std::make_unique<ExtrudeOp>();
    op->setProfile(m_profile);
    op->setDistance(opDistance());
    op->setMode(ExtrudeMode::NewBody);
    op->setSketchSource(m_sketchId);
    return op;
}

// Face count above which the boolean is worth a progress window. Measured on
// hole-grid plates a cut costs ~155 ms at 102 faces and grows fast from there;
// below this it finishes before a window could be drawn, so it stays inline.
// A threaded target also stays inline: History reflows the cut beneath the
// Thread step and re-cuts the thread around it, which a deferred push breaks.
bool ExtrudeController::wantsDeferredCommit(const IopContext& ctx) const {
    if (m_mode != ExtrudeMode::Subtract && m_mode != ExtrudeMode::Union) return false;
    if (m_targetBody < 0 || ctx.history.isBodyThreaded(m_targetBody)) return false;
    TopoDS_Shape target;
    try { target = ctx.doc.getBody(m_targetBody); } catch (...) { return false; }
    if (target.IsNull()) return false;
    TopTools_IndexedMapOfShape faces;
    TopExp::MapShapes(target, TopAbs_FACE, faces);
    return faces.Extent() >= 40;
}

bool ExtrudeController::syncLiveOp(Operation& op) {
    static_cast<ExtrudeOp&>(op).setDistance(opDistance());
    return true;
}

// Resolve the cut target from the swept volume before the base records
// anything. Without this the two ways a Subtract can quietly do nothing both
// end in a History step: no target at all (the base would record the preview,
// leaving the tool volume behind as a stray body), or a target the sweep never
// reaches (BRepAlgoAPI_Cut hands the body straight back, valid and unchanged).
// Refusing leaves the op OPEN so the distance can be pushed further or reversed.
void ExtrudeController::update(const IopContext& ctx) {
    if (!active()) return;
    if (m_editingIndex < 0) {
        if (profileCleared()) return;   // nothing picked: no preview
        InteractiveOpController::update(ctx);
        return;
    }
    if (std::abs(m_distance) < 0.01f) return;   // no zero-length preview mid-drag
    materializr::BodyChangeScope trackBodies(ctx.doc, ctx.markBodyDirty, ctx.markMeshesDirty);
    writeEditDistance(ctx, opDistance());
    setPreviewOk(m_editPreview.replay(m_editingIndex, ctx.doc, ctx.history));
}

void ExtrudeController::finishEdit(const IopContext& ctx) {
    if (m_shownSketch >= 0) {
        ctx.doc.setSketchVisible(m_shownSketch, false);
        m_shownSketch = -1;
    }
    m_editPreview.clear();
    ctx.selection.clear();
    if (ctx.markMeshesDirty) ctx.markMeshesDirty();
    teardown();
}

void ExtrudeController::cancel(const IopContext& ctx) {
    if (m_editingIndex < 0) { InteractiveOpController::cancel(ctx); return; }
    if (active()) {
        materializr::BodyChangeScope trackBodies(ctx.doc, ctx.markBodyDirty, ctx.markMeshesDirty);
        // The preview mutated the real op: put its distance back and replay
        // (replay falls back to the pre-edit snapshot if even that fails).
        writeEditDistance(ctx, m_origOpDistance);
        if (m_profileEdited)
            if (auto* op = dynamic_cast<ExtrudeOp*>(
                    const_cast<Operation*>(ctx.history.getStep(m_editingIndex))))
                op->replaceProfile(m_origProfile, m_origSketchId);
        m_editPreview.replay(m_editingIndex, ctx.doc, ctx.history);
    }
    finishEdit(ctx);
}

void ExtrudeController::commit(const IopContext& ctx) {
    if (profileCleared() && active()) {
        // The Profiles field was cleared and nothing re-picked yet.
        if (ctx.toast) ctx.toast(materializr::tr("Pick a profile to extrude."));
        return;
    }
    if (m_editingIndex >= 0) {
        if (!active()) { finishEdit(ctx); return; }
        materializr::BodyChangeScope trackBodies(ctx.doc, ctx.markBodyDirty, ctx.markMeshesDirty);
        // Zero length would delete the feature - treat it as Cancel.
        const bool zero = std::abs(m_distance) < 0.01f;
        writeEditDistance(ctx, zero ? m_origOpDistance : opDistance());
        if (!m_editPreview.replay(m_editingIndex, ctx.doc, ctx.history) && !zero) {
            // replay() restored the pre-edit model; the op gets its old value.
            writeEditDistance(ctx, m_origOpDistance);
            m_editPreview.replay(m_editingIndex, ctx.doc, ctx.history);
            if (ctx.toast)
                ctx.toast(materializr::tr("Couldn't recompute after an upstream change. Edit its parameters, fix the step before it, or delete it."));
        }
        finishEdit(ctx);
        return;
    }
    materializr::BodyChangeScope trackBodies(ctx.doc, ctx.markBodyDirty, ctx.markMeshesDirty);
    if (active() && m_mode == ExtrudeMode::Subtract) {
        if (m_cutAllBodies) {
            const std::vector<int> targets = resolveAllCutTargets(ctx);
            if (targets.empty()) {
                if (ctx.toast)
                    ctx.toast(materializr::tr("Subtract: this profile doesn't reach any body - "
                                              "nothing to cut. Extrude it further, or drag the other way."));
                return;
            }
            commitCutAll(ctx, targets);
            return;
        }
        const int target = resolveCutTarget(ctx);
        if (target < 0) {
            if (ctx.toast)
                ctx.toast(materializr::tr("Subtract: this profile doesn't reach any body - "
                                          "nothing to cut. Extrude it further, or drag the other way."));
            return;
        }
        m_targetBody = target;
    }
    InteractiveOpController::commit(ctx);
}

std::vector<int> ExtrudeController::resolveAllCutTargets(
        const IopContext& ctx) const {
    const int previewId = previewBodyId();
    if (previewId < 0) return {};
    TopoDS_Shape tool;
    try { tool = ctx.doc.getBody(previewId); } catch (...) { return {}; }
    if (tool.IsNull()) return {};
    return cutpick::pickAllCutTargets(cutCandidates(ctx, previewId), tool);
}

// One body, one step. The base's commit() records at most a single alternative
// op, so this takes over the whole tail of the gesture: drop the preview volume
// (it is a NewBody tool that must not survive), then push a real Subtract per
// target. Each carries the same profile and distance, so each cuts its own body
// exactly as a single-target Subtract would - including its face lineage, which
// is per-body and would have to be reinvented to pack them into one op.
void ExtrudeController::commitCutAll(const IopContext& ctx,
                                     const std::vector<int>& targets) {
    if (livePreviewApplied() && liveOp()) {
        try { liveOp()->undo(ctx.doc); } catch (...) {}
    }
    int done = 0;
    for (int id : targets) {
        auto op = std::make_unique<ExtrudeOp>();
        op->setProfile(m_profile);
        op->setDistance(opDistance());
        op->setMode(ExtrudeMode::Subtract);
        op->setTargetBody(id);
        op->setSketchSource(m_sketchId);
        if (ctx.history.pushOperation(std::move(op), ctx.doc)) ++done;
    }
    std::fprintf(stdout, "Subtracted %.1f mm from %d of %zu bodies\n",
                 std::abs(m_distance), done, targets.size());
    // A body whose cut FAILED is not a silent loss - pushOperation refuses an
    // op that can't produce a valid solid, so that body is simply unchanged,
    // and the ones that worked still landed.
    if (done < static_cast<int>(targets.size()) && ctx.toast) {
        ctx.toast(materializr::trFormat("Cut %d of %d bodies - the rest couldn't make a "
                                        "valid solid and were left alone.",
                                        done, static_cast<int>(targets.size())).c_str());
    }
    ctx.selection.clear();
    teardown();
}

std::unique_ptr<Operation> ExtrudeController::buildCommitOp(const IopContext& ctx) {
    (void)ctx;
    // NewBody: the previewed instance IS the result - let the base record it
    // as-is. Subtract: the preview was only a tool volume, so hand back the
    // real boolean cut against the body the sketch was drawn on.
    if (m_mode != ExtrudeMode::Subtract || m_targetBody < 0) {
        std::fprintf(stdout, "Extruded %.1f mm\n", m_distance);
        return nullptr;
    }
    auto op = std::make_unique<ExtrudeOp>();
    op->setProfile(m_profile);
    op->setDistance(opDistance());
    op->setMode(ExtrudeMode::Subtract);
    op->setTargetBody(m_targetBody);
    op->setSketchSource(m_sketchId);
    std::fprintf(stdout, "Subtracted %.1f mm from body %d\n",
                 std::abs(m_distance), m_targetBody);
    return op;
}

// Feed a drag delta into the distance. The raw accumulator is re-seeded from
// m_distance whenever something else (typing, steppers, begin) changed it.
void ExtrudeController::dragBy(float delta) {
    if (m_distance != m_dragAppliedDist) m_dragRaw = m_distance;
    m_dragRaw += delta;
    m_distance = m_dragRaw;   // snapped (if enabled) in updateExtrude
}

void ExtrudeController::updateExtrude(const IopContext& ctx, bool applySnap) {
    if (!active()) return;
    if (!std::isfinite(m_distance)) { m_distance = 0.0f; return; }
    // Snap the live distance to the corner-widget grid step before applying
    // (issue #24). Drag/commit paths snap; live typing and the steppers pass
    // applySnap=false so a typed value stays exact.
    if (applySnap && ctx.snapToGrid && ctx.gridStep > 0.0f) {
        m_distance = std::round(m_distance / ctx.gridStep) * ctx.gridStep;
        materializr::formatLengthDigits(m_inputBuf, sizeof(m_inputBuf), m_distance);
    }
    update(ctx);
}

int ExtrudeController::previewBodyId() const {
    const auto* op = static_cast<const ExtrudeOp*>(liveOp());
    return (op && livePreviewApplied()) ? op->createdBodyId() : -1;
}

// Left-drag in the viewport moves the distance along the arrow's normal.
// No sub-region to hit-test on DESKTOP - the whole viewport is the drag
// surface there - so the handle claims EVERY press while the op is active.
// Without this, trackpad mode (orbit + pan both on Left) let the camera
// claim the drag-threshold frame before vp.dragging ever turned true, so a
// press-and-hold drag orbited instead of extruding (Steve: "it is not
// letting me drag ... during extrude"). TOUCH instead hit-tests the visible
// arrow (toucharrow::arrowScreen - the same geometry the overlay draws) and
// claims only a press on it, so a swipe anywhere else orbits the model.
// Release on button-up, same as EdgeOpController. NOT gated on
// !vp.uiCaptured like the sticky toggle below: this whole block only runs
// inside `if (viewportHovered)` (ImGui::IsItemHovered on the viewport
// image), which is already false whenever a floating panel occludes the
// cursor - io.WantCaptureMouse itself is true across the ENTIRE viewport
// regardless of overlays (see ViewCube.cpp's note on the same gotcha), so
// gating this claim on it made it dead code and the fix a no-op.
void ExtrudeController::onViewportInput(const IopViewport& vp,
                                        const IopContext& ctx) {
    if (!active()) return;

    // Profiles field armed: a tap (press + lift without moving, not on the
    // arrow) toggles the region / face under the finger. Drags still orbit
    // or pull the arrow as usual.
    if (m_picking) {
        if (vp.clicked) { m_tapArmed = true; m_tapAt = vp.mouse; }
        if (m_tapArmed && glm::length(vp.mouse - m_tapAt) > 12.0f * uiScale())
            m_tapArmed = false;
        if (m_tapArmed && vp.released) {
            m_tapArmed = false;
            // Not gated on vp.uiCaptured: WantCaptureMouse is true over the
            // whole viewport image (see the note above), which swallowed
            // every tap. This runs only while the viewport is hovered.
            if (!draggingHandle()) {
                Pick p;
                if (pickAt(vp, ctx, p)) {
                    togglePick(ctx, p);
                    return;   // the controller may have restarted
                }
            }
        }
    }

    if (vp.clicked) {
        if (materializr::touchMode()) {
            // Hit-test the visible arrow (same geometry the viewport draws).
            glm::vec2 a, b;
            const bool hit =
                toucharrow::arrowScreen(vp.toScreen, m_origin, m_normal,
                                        m_distance, a, b) &&
                toucharrow::distToSegment(vp.mouse, a, b) <
                    toucharrow::pickRadiusPx(ctx.panel.imTouch,
                                             ctx.panel.uiScale);
            setDraggingHandle(hit);
        } else {
            setDraggingHandle(true);   // desktop: unchanged
        }
    } else if (draggingHandle() && !vp.down) setDraggingHandle(false);

    if (vp.dragging && draggingHandle()) {
        dragBy(vp.dragAlongAxis(m_origin, m_normal, vp.mouseDelta));
        updateExtrude(ctx);
        m_dragAppliedDist = m_distance;
        materializr::formatLengthDigits(m_inputBuf, sizeof(m_inputBuf), m_distance);
    }
    // Trackpad-mode click-move-click, same model as Push/Pull: with Left
    // now free to orbit during the op, a whole-viewport drag surface needs
    // a buttonless way to drive the value. Click (without dragging) arms
    // sticky, cursor motion feeds the distance, click again to release.
    if (vp.trackpadInput && !vp.uiCaptured) {
        if (vp.clicked) m_stickyPressWasDrag = false;
        if (vp.dragging) m_stickyPressWasDrag = true;
        if (vp.released && !m_stickyPressWasDrag) m_sticky = !m_sticky;
    }
    if (m_sticky && (vp.mouseDelta.x != 0.0f || vp.mouseDelta.y != 0.0f)) {
        dragBy(vp.dragAlongAxis(m_origin, m_normal, vp.mouseDelta));
        updateExtrude(ctx);
        m_dragAppliedDist = m_distance;
        materializr::formatLengthDigits(m_inputBuf, sizeof(m_inputBuf), m_distance);
    }
}

// ── Profiles field ──────────────────────────────────────────────────────────

int ExtrudeController::profileCount() const {
    if (m_picking || !m_picks.empty()) return static_cast<int>(m_picks.size());
    int n = 0;
    for (TopExp_Explorer ex(m_profile, TopAbs_FACE); ex.More(); ex.Next()) ++n;
    return n;
}

bool ExtrudeController::pickAt(const IopViewport& vp, const IopContext& ctx, Pick& out) const {
    const glm::vec3 d = glm::normalize(vp.rayDir);
    const gp_Pnt o(vp.rayOrigin.x, vp.rayOrigin.y, vp.rayOrigin.z);
    const gp_Dir dir(d.x, d.y, d.z);
    double best = 1e300;
    bool found = false;
    // Sketch regions of visible sketches (cached builds only - a tap never
    // starts a region build).
    for (int sid : ctx.doc.getAllSketchIds()) {
        if (!ctx.doc.isSketchVisible(sid)) continue;
        auto sk = ctx.doc.getSketch(sid);
        if (!sk || (!sk->regionsCached() && sid != m_shownSketch)) continue;
        const auto regions = sk->buildRegions();
        for (size_t i = 0; i < regions.size(); ++i) {
            gp_Pnt hit;
            if (regions[i].face.IsNull() || !surf::pickPoint(regions[i].face, o, dir, hit)) continue;
            // Regions win a tie with the body face they lie on.
            const double t = o.Distance(hit) - 1e-3;
            if (t < best) {
                best = t;
                out = Pick{sid, static_cast<int>(i), TopoDS_Face()};
                found = true;
            }
        }
    }
    // Planar faces of visible bodies, never the preview's own tool volume.
    // Edit mode: not the edited step's own result either (its created body,
    // or the body it cuts / joins) - that would feed the step its own output.
    int preview = previewBodyId(), ownTarget = -1;
    if (m_editingIndex >= 0) {
        if (const auto* op = dynamic_cast<const ExtrudeOp*>(ctx.history.getStep(m_editingIndex))) {
            preview = op->createdBodyId();
            ownTarget = op->getTargetBody();
        }
    }
    for (int id : ctx.doc.getAllBodyIds()) {
        if (id == preview || id == ownTarget || !ctx.doc.isBodyVisible(id) ||
            ctx.doc.isBodyMesh(id)) continue;
        TopoDS_Shape body;
        try { body = ctx.doc.getBody(id); } catch (...) { continue; }
        gp_Pnt hit;
        if (!surf::pickPoint(body, o, dir, hit)) continue;
        const double t = o.Distance(hit);
        if (t >= best) continue;
        const TopoDS_Face f = surf::nearestFace(body, hit);
        if (f.IsNull()) continue;
        Handle(Geom_Surface) gs = BRep_Tool::Surface(f);
        if (gs.IsNull() || !gs->IsKind(STANDARD_TYPE(Geom_Plane))) continue;
        best = t;
        out = Pick{-1, -1, f, id};
        found = true;
    }
    return found;
}

void ExtrudeController::togglePick(const IopContext& ctx, const Pick& p) {
    bool removed = false;
    for (auto it = m_picks.begin(); it != m_picks.end(); ++it) {
        const bool same = p.face.IsNull()
            ? (it->face.IsNull() && it->sketchId == p.sketchId && it->region == p.region)
            : (!it->face.IsNull() && it->face.IsSame(p.face));
        if (same) { m_picks.erase(it); removed = true; break; }
    }
    if (!removed) {
        // Regions extrude together only from one sketch (one plane): a region
        // of another sketch starts a fresh set.
        if (p.sketchId >= 0)
            for (const Pick& q : m_picks)
                if (q.sketchId >= 0 && q.sketchId != p.sketchId) { m_picks.clear(); break; }
        m_picks.push_back(p);
    }
    syncSelection(ctx);
    if (m_editingIndex >= 0) { applyEditProfile(ctx); return; }
    if (m_picks.empty()) {
        // The last pick went: same as clearing the field.
        m_picking = true;
        retractLivePreview(ctx);
        return;
    }
    restartFromPicks(ctx);
}

void ExtrudeController::syncSelection(const IopContext& ctx) const {
    ctx.selection.clear();
    for (const Pick& p : m_picks) {
        SelectionEntry e;
        if (p.face.IsNull()) {
            e.type = SelectionType::SketchRegion;
            e.sketchId = p.sketchId;
            e.subShapeIndex = p.region;
        } else if (p.bodyId >= 0) {
            e.type = SelectionType::Face;
            e.bodyId = p.bodyId;
            e.shape = p.face;
        } else {
            continue;
        }
        ctx.selection.addToSelection(e);
    }
}

void ExtrudeController::seedPicksFromProfile(const IopContext& ctx) {
    m_picks.clear();
    auto props = [](const TopoDS_Shape& f, double& area, gp_Pnt& c) {
        GProp_GProps g;
        BRepGProp::SurfaceProperties(f, g);
        area = g.Mass();
        c = area > 1e-12 ? g.CentreOfMass() : gp_Pnt();
    };
    std::vector<Sketch::Region> regions;
    std::shared_ptr<Sketch> sk = m_sketchId >= 0 ? ctx.doc.getSketch(m_sketchId) : nullptr;
    if (sk) regions = sk->buildRegions();
    for (TopExp_Explorer ex(m_profile, TopAbs_FACE); ex.More(); ex.Next()) {
        const TopoDS_Face f = TopoDS::Face(ex.Current());
        double a = 0.0;
        gp_Pnt c;
        props(f, a, c);
        int match = -1;
        for (size_t i = 0; i < regions.size() && match < 0; ++i) {
            if (regions[i].face.IsNull()) continue;
            double ra = 0.0;
            gp_Pnt rc;
            props(regions[i].face, ra, rc);
            const double tol = 1e-3 * std::max(1.0, std::sqrt(std::max(a, 0.0)));
            if (std::abs(ra - a) <= 1e-3 * std::max(1.0, a) && rc.Distance(c) <= tol)
                match = static_cast<int>(i);
        }
        if (match >= 0) m_picks.push_back(Pick{m_sketchId, match, TopoDS_Face()});
        else            m_picks.push_back(Pick{-1, -1, f});
    }
}

bool ExtrudeController::buildPickedProfile(const IopContext& ctx, TopoDS_Shape& out,
                                           int& sketchId) const {
    std::vector<TopoDS_Face> faces;
    sketchId = -1;
    for (const Pick& p : m_picks) {
        if (!p.face.IsNull()) { faces.push_back(p.face); continue; }
        auto sk = ctx.doc.getSketch(p.sketchId);
        if (!sk) continue;
        const auto regions = sk->buildRegions();
        if (p.region < 0 || p.region >= static_cast<int>(regions.size())) continue;
        if (!regions[static_cast<size_t>(p.region)].face.IsNull()) {
            faces.push_back(regions[static_cast<size_t>(p.region)].face);
            sketchId = p.sketchId;
        }
    }
    if (faces.empty()) return false;
    if (faces.size() == 1) {
        out = faces.front();
    } else {
        BRep_Builder b;
        TopoDS_Compound comp;
        b.MakeCompound(comp);
        for (const TopoDS_Face& f : faces) b.Add(comp, f);
        out = comp;
    }
    return true;
}

void ExtrudeController::applyEditProfile(const IopContext& ctx) {
    if (m_picks.empty()) return;   // cleared: keep the last preview, commit refuses
    TopoDS_Shape profile;
    int sketchId = -1;
    if (!buildPickedProfile(ctx, profile, sketchId)) return;
    auto* op = dynamic_cast<ExtrudeOp*>(const_cast<Operation*>(ctx.history.getStep(m_editingIndex)));
    if (!op) return;
    op->replaceProfile(profile, sketchId);
    m_profile = profile;
    m_sketchId = sketchId;
    m_profileEdited = true;
    computeFrame(ctx);
    m_normal *= static_cast<float>(m_sweepSign);
    update(ctx);
}

bool ExtrudeController::restartFromPicks(const IopContext& ctx) {
    TopoDS_Shape profile;
    int sketchId = -1;
    if (!buildPickedProfile(ctx, profile, sketchId)) return false;
    // Keep everything the user set: distance, mode, target, the picks.
    const float dist = m_distance;
    const ExtrudeMode mode = m_mode;
    const int target = m_targetBody;
    const bool cutAll = m_cutAllBodies;
    const bool picking = m_picking;
    m_restarting = true;
    InteractiveOpController::cancel(ctx);   // drop the old preview
    const bool ok = beginExtrude(ctx, profile, mode, target, sketchId);
    m_restarting = false;
    if (!ok) return false;
    m_picking = picking;
    m_cutAllBodies = cutAll;
    m_distance = m_dragRaw = m_dragAppliedDist = dist;
    materializr::formatLengthDigits(m_inputBuf, sizeof(m_inputBuf), m_distance);
    updateExtrude(ctx, /*applySnap=*/false);
    syncSelection(ctx);   // the restart's cancel cleared it
    return true;
}

void ExtrudeController::renderExtrudePanel(const IopContext& ctx) {
    if (!active()) return;
    const float s = ctx.panel.uiScale;
    const bool imTouch = ctx.panel.imTouch;

    materializr::viewportBanner(
        ImVec4(1.0f, 0.8f, 0.2f, 1.0f),
        materializr::touchMode()
            ? "EXTRUDE - Drag in viewport or type distance, then Confirm / Cancel."
            : "EXTRUDE - Drag in viewport or type distance. Enter to confirm, Escape to cancel.");

    // Floating distance input panel. im-touch: anchored just off the extrude
    // arrow's tip, like the sketch bubbles; other layouts (or a tip behind
    // the camera) keep the fixed top-right spot.
    bool extAnchored = false;
    if (imTouch && ctx.panel.anchorValid) {
        const ImVec2 vwp = ImGui::GetWindowPos();
        const float vww = ImGui::GetWindowWidth();
        float ax = std::min(std::max(ctx.panel.anchorX + 24.0f * s, vwp.x + 8.0f),
                            vwp.x + vww - 250.0f * s);
        float ay = std::max(ctx.panel.anchorY + 12.0f * s, vwp.y + 8.0f);
        ImGui::SetNextWindowPos(ImVec2(ax, ay), ImGuiCond_Appearing);
        extAnchored = true;
    }
    if (!extAnchored)
        ImGui::SetNextWindowPos(ImVec2(
            std::max(ImGui::GetWindowPos().x + 6.0f,
                     ImGui::GetWindowPos().x + ImGui::GetWindowWidth() - 250.0f * s),
            ImGui::GetWindowPos().y + 50), ImGuiCond_Appearing);
    // Pin the width (min == max) so moving the panel can't feed back into
    // the value field's content-avail width and ratchet the window wider.
    ImGui::SetNextWindowSizeConstraints(ImVec2(240.0f * s, 0.0f),
                                        ImVec2(240.0f * s, 100000.0f));
    const ImVec2 hostPos = ImGui::GetWindowPos(), hostSize = ImGui::GetWindowSize();
    ImGui::Begin("##ExtrudeInput", nullptr,
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoDocking |
        ImGuiWindowFlags_AlwaysAutoResize);
    opDialogDragGrip(s);

    if (!imTouch) {   // im-touch: just the value well below
        ImGui::Text("%s", materializr::trFormat("Extrude Distance (%s)", materializr::unitSuffix()).c_str());
        ImGui::Separator();
    }

    if (m_inputFocus) {
        if (!materializr::touchMode())
            ImGui::SetKeyboardFocusHere();  // touch: drag to set distance, or tap the field to type
        m_inputFocus = false;
    }

    bool doCommit = false, doCancel = false;
    // Profiles selection field (Fusion's "Profiles: 1 selected  x").
    if (!m_surface) {
        const float fh = 40.0f * s;
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted(materializr::tr("Profiles"));
        ImGui::SameLine();
        char lbl[48];
        std::snprintf(lbl, sizeof(lbl), materializr::tr("%d selected"), profileCount());
        // The colour push and pop key off one value: the button below flips
        // m_picking, and popping on the new value unbalanced ImGui's stack.
        const bool armed = m_picking;
        if (armed) ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        const float clearW = fh;
        const float fieldW = std::max(80.0f * s, ImGui::GetContentRegionAvail().x - clearW -
                                                     ImGui::GetStyle().ItemSpacing.x);
        bool toggleField = ImGui::Button(lbl, ImVec2(fieldW, fh));
        if (armed) ImGui::PopStyleColor();
        ImGui::SameLine();
        bool clearField = ImGui::Button("x##clearProfiles", ImVec2(clearW, fh));
        if (toggleField) {
            if (!m_picking && m_picks.empty()) seedPicksFromProfile(ctx);
            // Edit mode: the step's sketch is usually hidden after the
            // extrude; show it while its regions are being picked.
            if (m_editingIndex >= 0 && m_origSketchId >= 0 && m_shownSketch < 0 &&
                !ctx.doc.isSketchVisible(m_origSketchId)) {
                ctx.doc.setSketchVisible(m_origSketchId, true);
                m_shownSketch = m_origSketchId;
            }
            m_picking = !m_picking || m_picks.empty();
        }
        if (clearField) {
            m_picks.clear();
            m_picking = true;
            ctx.selection.clear();
            // Create mode: the preview goes with the profile. Edit mode keeps
            // showing the step until something is picked.
            if (m_editingIndex < 0) retractLivePreview(ctx);
        }
        ImGui::Spacing();
    }
    if (imTouch) {
        // im-touch: the WHOLE panel is this one tappable value well - no
        // header, hint or steppers (Steve: the full "distance dialog" kept
        // showing up; drag for coarse, pad for exact).
        if (materializr::amountLengthField("extAmt", materializr::tr("Distance"), &m_distance, /*allowSign=*/true)) {
            materializr::formatLengthDigits(m_inputBuf, sizeof(m_inputBuf), m_distance);
            updateExtrude(ctx, /*applySnap=*/false);  // typed = exact
        }
    } else {
        // The member is the truth; the buffer follows it unless being typed in.
        materializr::reseedLengthBufferIfIdle("##dist", m_inputBuf, sizeof(m_inputBuf), m_distance);
        if (materializr::inputNumberText("##dist", m_inputBuf, sizeof(m_inputBuf),
                             ImGuiInputTextFlags_EnterReturnsTrue)) {
            // Enter pressed - commit (parseFinite: keep last on garbage)
            (void)materializr::parseLength(m_inputBuf, m_distance);
            // Typed = exact. Snapping here rounded a typed 0.2 to the grid
            // step (usually 0), so Enter committed nothing while the Confirm
            // button, which never re-snaps, worked.
            updateExtrude(ctx, /*applySnap=*/false);
            doCommit = true;
        } else if (materializr::lengthBufferIsActive("##dist")) {
            // Only while typing - an idle re-parse truncated the member to the
            // buffer's decimals and reinterpreted stale text after a unit switch.
            float parsed = m_distance;
            if (materializr::parseLength(m_inputBuf, parsed) &&
                std::abs(parsed - m_distance) > 0.01f && std::abs(parsed) > 0.01f) {
                m_distance = parsed;
                updateExtrude(ctx, /*applySnap=*/false);  // live typing = exact
            }
        }
        ImGui::SameLine();
        ImGui::Text("%s", materializr::unitSuffix());
    }

    // Quick-nudge stepper (replaces the slider): ±10/1/0.1, and 0 to clear
    // the extrusion mid-preview. Desktop only - im-touch stays a single well.
    if (!imTouch &&
        materializr::lengthStepperRow("extrudeStep", &m_distance,
                                /*allowNegative=*/true, -50.0f, 50.0f)) {
        materializr::formatLengthDigits(m_inputBuf, sizeof(m_inputBuf), m_distance);
        updateExtrude(ctx, /*applySnap=*/false);  // steppers override the grid
    }

    // Subtract only: cut everything the sweep passes through, not just the one
    // body. Off by default - a sketch on a face means that face's body, and
    // carving a neighbour it merely overlaps would be a surprise. The checkbox
    // is here (not a setting) because it is a property of THIS cut: a profile
    // meant to pass through a stack and one meant to pocket a single plate are
    // the same gesture until you say which.
    if (m_mode == ExtrudeMode::Subtract && m_editingIndex < 0) {
        ImGui::Spacing();
        // Nothing to re-preview either way: the preview IS the tool volume, and
        // which bodies it cuts is decided at commit. The panel is pinned to
        // 240*s, so the label has to stay short or it clips - the tooltip
        // carries the detail. im-touch gets a pill instead of a checkbox: a
        // checkbox tickbox is a fingertip-hostile tap target.
        if (imTouch) {
            if (touchui::pillButton("cutall", MZ_ICON_BODY, materializr::tr("All bodies"),
                                    m_cutAllBodies))
                m_cutAllBodies = !m_cutAllBodies;
        } else {
            ImGui::Checkbox(materializr::tr("Cut every body it reaches"), &m_cutAllBodies);
        }
        ImGui::SetItemTooltip("%s", materializr::tr("Off: cut ONE body - the one the sketch sits on when it has a host, otherwise whichever the sweep reaches most of.\nOn: cut every body the swept profile reaches, each as its own undoable step."));
    }

    if (!ctx.cornerCommitUi) {   // im-touch: corner ✓/✗ FABs instead
        ImGui::Spacing();
        if (ImGui::Button(materializr::btnConfirm(), ImVec2(110, 0)))
            doCommit = true;
        ImGui::SameLine();
        if (ImGui::Button(materializr::btnCancel(), ImVec2(110, 0)))
            doCancel = true;
    }
    opDialogKeepOnScreen(hostPos, hostSize);
    ImGui::End();
    // Commit/cancel AFTER End() - they tear the controller's state down, and
    // the window has to be closed first.
    if (doCommit) commit(ctx);
    else if (doCancel) cancel(ctx);
}

void ExtrudeController::confirmFromKey(const IopContext& ctx) {
    if (!active()) return;
    (void)materializr::parseLength(m_inputBuf, m_distance);
    updateExtrude(ctx, /*applySnap=*/false);   // typed = exact, see the panel's Enter
    commit(ctx);
}

void ExtrudeController::panelBody(const IopContext&, bool&) {
    // Unused: renderExtrudePanel draws the whole thing (renderPanel is
    // overridden silent) because the value well anchors to the viewport.
}

void ExtrudeController::onCleanup() {
    m_sticky = false;
    m_stickyPressWasDrag = false;
    m_profile.Nullify();
    m_mode = ExtrudeMode::NewBody;
    m_targetBody = -1;
    m_sweepSign = 1.0;
    m_cutAllBodies = false;
    m_sketchId = -1;
    m_editingIndex = -1;
    m_origOpDistance = 0.0;
    m_editPreview.clear();
    m_surface = false;
    m_origProfile.Nullify();
    m_origSketchId = -1;
    m_profileEdited = false;
    if (!m_restarting) {
        m_picks.clear();
        m_picking = false;
    }
    m_tapArmed = false;
}

} // namespace materializr
