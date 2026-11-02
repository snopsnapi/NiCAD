#include "FormOp.h"
#include "Form.h"
#include <cstdio>
#include <cstdlib>

bool FormOp::execute(Document& doc) {
    materializr::form::Cage c;
    if (!c.deserialize(m_cage)) return false;
    if (m_formId < 0) m_formId = doc.allocateFormId();
    doc.setForm(m_formId, m_cage);
    doc.setFormVisible(m_formId, true);
    return true;
}

bool FormOp::undo(Document& doc) {
    if (m_formId >= 0) doc.removeForm(m_formId);   // id kept for redo
    return true;
}

std::string FormOp::description() const {
    materializr::form::Cage c;
    if (!c.deserialize(m_cage)) return "Form";
    return "Form, " + std::to_string(c.faces.size()) + " faces";
}

// The cage text holds no ';' or '=', so it rides as the last field as-is.
std::string FormOp::serializeParams() const {
    return "id=" + std::to_string(m_formId) + ";cage=" + m_cage;
}

bool FormOp::deserializeParams(const std::string& blob) {
    const size_t c = blob.find(";cage=");
    if (blob.rfind("id=", 0) != 0 || c == std::string::npos) return false;
    m_formId = std::atoi(blob.c_str() + 3);
    m_cage = blob.substr(c + 6);
    return true;
}

bool ConvertFormOp::execute(Document& doc) {
    const std::string* text = doc.getForm(m_formId);
    if (!text) return false;
    materializr::form::Cage c;
    if (!c.deserialize(*text)) return false;
    const TopoDS_Shape s = materializr::form::toNurbs(c, 3);
    if (s.IsNull()) return false;
    try {
        doc.addOrPutBody(m_createdBodyId, s, "Form body");
    } catch (...) {
        return false;
    }
    doc.setFormVisible(m_formId, false);
    return true;
}

bool ConvertFormOp::undo(Document& doc) {
    try {
        if (m_createdBodyId >= 0) doc.removeBody(m_createdBodyId);
    } catch (...) {}
    doc.setFormVisible(m_formId, true);
    return true;
}

OperationDiff ConvertFormOp::captureDiff() const {
    OperationDiff d;
    if (m_createdBodyId >= 0) d.created.push_back(m_createdBodyId);
    return d;
}

std::string ConvertFormOp::serializeParams() const {
    return "form=" + std::to_string(m_formId);
}

bool ConvertFormOp::deserializeParams(const std::string& blob) {
    if (blob.rfind("form=", 0) != 0) return false;
    m_formId = std::atoi(blob.c_str() + 5);
    return true;
}

bool ConvertFormOp::rehydrateFromReload(const ReloadState& state, Document&) {
    if (m_formId < 0 || state.created.empty()) return false;
    m_createdBodyId = state.created.front();
    return true;
}
