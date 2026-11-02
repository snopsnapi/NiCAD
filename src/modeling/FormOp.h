#pragma once
#include "../core/Operation.h"
#include "../core/Document.h"
#include <string>

// Form (fork, Form workspace): one T-spline-style cage as a history step. The
// cage text (form::Cage::serialize) is the step's parameter; execute writes
// it into the Document's form table under a stable id, so the Edit Form tool
// edits this step in place and everything after it (a NURBS conversion)
// replays.
class FormOp : public Operation {
public:
    void setCage(const std::string& cage) { m_cage = cage; }
    const std::string& cage() const { return m_cage; }
    int formId() const { return m_formId; }

    bool execute(Document& doc) override;
    bool undo(Document& doc) override;
    std::string name() const override { return "Form"; }
    std::string description() const override;
    void renderProperties() override {}
    std::string typeId() const override { return "form"; }
    std::vector<int> plannedBodyIds() const override { return {}; }
    std::string serializeParams() const override;
    bool deserializeParams(const std::string& blob) override;
    bool rehydrateFromReload(const ReloadState&, Document&) override { return m_formId >= 0; }

private:
    std::string m_cage;
    int m_formId = -1;
};

// Convert to NURBS: the form's smooth surface as bicubic B-spline patches,
// sewn (a closed form becomes a solid), as a new body; the form is hidden.
// Re-reads the cage on every execute, so editing the form updates the body.
class ConvertFormOp : public Operation {
public:
    void setForm(int id) { m_formId = id; }
    int createdBodyId() const { return m_createdBodyId; }

    bool execute(Document& doc) override;
    bool undo(Document& doc) override;
    std::string name() const override { return "Convert Form"; }
    std::string description() const override { return "Form to NURBS"; }
    void renderProperties() override {}
    std::string typeId() const override { return "convert_form"; }
    OperationDiff captureDiff() const override;
    std::vector<int> plannedBodyIds() const override { return {}; }
    std::string serializeParams() const override;
    bool deserializeParams(const std::string& blob) override;
    bool rehydrateFromReload(const ReloadState& state, Document& doc) override;

private:
    int m_formId = -1;
    int m_createdBodyId = -1;
};
