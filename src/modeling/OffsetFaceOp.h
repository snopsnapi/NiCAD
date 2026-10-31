#pragma once
#include "../core/Operation.h"
#include "../core/Document.h"
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <TopTools_ListOfShape.hxx>
#include <vector>
#include <string>

// Offset the selected face(s) along their outward normals while the rest of
// the body follows (OffsetFace.h). Adjacent fillets/chamfers can be removed
// first, which is how a face with a baked round on it is moved and the round
// comes off sharp. Faces persist as ordinal indices into the INPUT shape's
// canonical face map (see SubShapeIndex.h) so the step reloads as a real
// editable op - same scheme as DefeatureOp.
class OffsetFaceOp : public Operation {
public:
    OffsetFaceOp();
    ~OffsetFaceOp() override = default;

    void setBody(int id);
    void addFace(const TopoDS_Face& face);
    void clearFaces();
    void setDistance(double distance);
    void setRemoveBlends(bool remove);

    int getBodyId() const { return m_bodyId; }
    double getDistance() const { return m_distance; }
    bool getRemoveBlends() const { return m_removeBlends; }

    // The off-thread preview (OffsetFaceController::previewOffThread) points
    // these at a private copy of the snapshot before execute() runs there.
    std::vector<TopoDS_Shape*> shapeParams() override {
        std::vector<TopoDS_Shape*> out;
        for (TopTools_ListIteratorOfListOfShape it(m_faces); it.More(); it.Next())
            out.push_back(&it.ChangeValue());
        return out;
    }

    bool execute(Document& doc) override;
    bool undo(Document& doc) override;
    std::string name() const override { return "Offset Face"; }
    std::string description() const override;
    void renderProperties() override;
    std::string typeId() const override { return "offset_face"; }
    OperationDiff captureDiff() const override;
    std::vector<int> plannedBodyIds() const override { return {m_bodyId}; }
    std::string serializeParams() const override;
    bool deserializeParams(const std::string& blob) override;
    bool rehydrateFromReload(const ReloadState& state, Document& doc) override;

private:
    int m_bodyId = -1;
    TopTools_ListOfShape m_faces;
    double m_distance = 5.0;
    bool m_removeBlends = true;
    TopoDS_Shape m_previousShape;
    // Offset faces persist as ordinal indices into the INPUT shape's canonical
    // face map (see SubShapeIndex.h) so the step reloads as a real editable op.
    std::vector<int> m_faceIndices;
};
