#pragma once
#include "ExtrudeOp.h"

// Surface Extrude (fork, Surface workspace): every curve of a sketch - open
// or closed - swept straight along the sketch normal into a SURFACE body
// (never a solid). An ExtrudeOp subclass so the interactive extrude tool
// (arrow, distance well, timeline edit) drives it unchanged: it reuses the
// distance and the created-body id, and re-derives its curves from the sketch
// on every execute, so a sketch edit flows through on replay.
class SurfaceExtrudeOp : public ExtrudeOp {
public:
    SurfaceExtrudeOp() = default;

    bool execute(Document& doc) override;
    bool undo(Document& doc) override;
    std::string name() const override { return "Surface Extrude"; }
    std::string description() const override;
    void renderProperties() override;
    std::string typeId() const override { return "surface_extrude"; }
    OperationDiff captureDiff() const override;
    std::vector<int> plannedBodyIds() const override { return {}; }
    std::string serializeParams() const override;
    bool deserializeParams(const std::string& blob) override;
    bool rehydrateFromReload(const ReloadState& state, Document& doc) override;
};
