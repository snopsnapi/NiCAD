#pragma once
#include "../core/Operation.h"
#include "../core/Document.h"
#include "TransformOp.h"
#include "FaceTweakOp.h"
#include <gp_Trsf.hxx>
#include <memory>
#include <string>
#include <vector>

// Fork: Fusion's Move/Copy as ONE parametric history step.
//
// What moves: whole bodies, or one face (its neighbours rebuilt - FaceTweak).
// How: a translation of the pivot (tx, ty, tz) plus rotations about the world
// X, Y and Z axes through the pivot (rx, then ry, then rz, degrees). Every drag
// and typed value of a Move session composes into these six numbers, so the
// history shows one "Move" with readable values, and editing the step re-runs
// it with new ones.
//
// The geometry is done by the existing ops, held as children: a TransformOp per
// body (raw transform - it keeps the sketch follow / de-link, face lineage and
// mate guard) or one FaceTweakOp for a face.
class MoveOp : public Operation {
public:
    enum class Target { Bodies, Face };

    MoveOp() = default;
    ~MoveOp() override = default;

    // Bodies: one child per body. detach = sketches this body-only move de-links.
    void addBody(int bodyId, const std::vector<int>& detachSketches = {});
    // Face: the face as it is on the body BEFORE this step.
    void setFace(int bodyId, const TopoDS_Face& face);

    void setPivot(double x, double y, double z) { m_pivot[0] = x; m_pivot[1] = y; m_pivot[2] = z; }
    void setParams(const double t[3], const double rDeg[3]);
    // Set the params from a full rigid transform (about the stored pivot).
    void setFromTrsf(const gp_Trsf& m);
    gp_Trsf trsf() const;

    Target target() const { return m_target; }
    const double* translation() const { return m_t; }
    const double* rotationDeg() const { return m_r; }
    const double* pivot() const { return m_pivot; }
    std::vector<int> bodyIds() const;
    int faceBodyId() const { return m_faceBody; }

    bool execute(Document& doc) override;
    bool undo(Document& doc) override;
    std::string name() const override { return "Move"; }
    std::string description() const override;
    void renderProperties() override;
    std::string typeId() const override { return "move"; }
    OperationDiff captureDiff() const override;
    std::vector<int> plannedBodyIds() const override { return bodyIds(); }
    std::string serializeParams() const override;
    bool deserializeParams(const std::string& blob) override;
    bool rehydrateFromReload(const ReloadState& state, Document& doc) override;
    void snapshotEditState() override;
    void restoreEditState() override;

private:
    void pushTrsfToChildren();

    Target m_target = Target::Bodies;
    double m_t[3] = {0, 0, 0};
    double m_r[3] = {0, 0, 0};
    double m_pivot[3] = {0, 0, 0};

    struct BodyChild {
        int bodyId = -1;
        std::vector<int> detach;
        std::unique_ptr<TransformOp> op;
    };
    std::vector<BodyChild> m_bodies;
    int m_faceBody = -1;
    std::unique_ptr<FaceTweakOp> m_faceOp;
    std::string m_faceBlob;   // reload: FaceTweakOp params until rehydrated
};
