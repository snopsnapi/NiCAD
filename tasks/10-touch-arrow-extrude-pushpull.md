# ТЗ-10. Толстая стрелка-ручка для Extrude и Push/Pull (тач)

## Цель (BACKLOG п.1)
Сейчас на планшете во время Extrude и Push/Pull любое касание вьюпорта тянет значение — модель нельзя
вращать. Причина: `ExtrudeController::onViewportInput` (src/app/ExtrudeController.cpp ~351) и
`PushPullController::onViewportInput` (src/app/PushPullController.cpp ~684) делают `setDraggingHandle(true)` на
КАЖДОЕ нажатие. Стрелка при этом тонкая (общий `drawDim` в Application_Viewport.cpp ~1401, `DimStyle::Bold`:
линия 3 px, наконечник 13 px — без учёта плотности экрана).
Сделать как в Offset Face (уже работает на планшете): толстая стрелка, тянуть ТОЛЬКО за неё, мимо — вращение.

ТОЛЬКО в режиме касаний (`materializr::touchMode()`). Десктоп (мышь/трекпад, sticky-режим) не менять.

## 1. Общий хелпер `src/app/TouchArrow.h` (новый, header-only)
Вынеси из `src/app/FaceOpControllers.cpp` функцию `offsetArrowScreen` (namespace без имени, рядом с
`OffsetFaceController::onBegin`) в `namespace materializr::toucharrow`:
```cpp
constexpr float kMinArrowDp = 70.0f;
template <class ToScreen>
bool arrowScreen(const ToScreen& toScreen, const glm::vec3& origin, const glm::vec3& dir,
                 float distance, glm::vec2& a, glm::vec2& b);   // как offsetArrowScreen сейчас
inline float distToSegment(glm::vec2 p, glm::vec2 a, glm::vec2 b);  // из лямбды distToSeg
inline float pickRadiusPx(bool imTouch, float uiScale) { return (imTouch ? 48.0f : 20.0f) * uiScale; }
```
`OffsetFaceController` переведи на этот хелпер (поведение не меняется). Размеры отрисовки тоже вынеси константами:
толщина 5 dp (7 при захвате), наконечник 22 dp (26 при захвате) — как сейчас в `OffsetFaceController::drawOverlay`.

## 2. Захват только за стрелку (touch)
В обоих контроллерах в `onViewportInput`:
```cpp
if (vp.clicked) {
    if (materializr::touchMode()) {
        // hit-test the visible arrow (same geometry the viewport draws)
        glm::vec2 a, b;
        bool hit = toucharrow::arrowScreen(vp.toScreen, origin, dir, distance, a, b) &&
                   toucharrow::distToSegment(vp.mouse, a, b) < toucharrow::pickRadiusPx(ctx.panel.imTouch, ctx.panel.uiScale);
        setDraggingHandle(hit);
    } else {
        setDraggingHandle(true);            // desktop: unchanged
    }
} else if (draggingHandle() && !vp.down) setDraggingHandle(false);
if (vp.dragging && draggingHandle()) { ...существующий код изменения значения... }
```
Важно: сейчас изменение значения идёт по `if (vp.dragging)` без проверки захвата — добавь `&& draggingHandle()`
(только в touch-ветке поведения; на десктопе draggingHandle всегда true после клика, так что условие безопасно).
Для Push/Pull со значением ~0 (там рисуется двусторонняя «стартовая» стрелка через центр грани, ±40 dp) — hit-test
по отрезку от `origin - dir*L` до `origin + dir*L`, где L подбери так, чтобы совпадало с отрисовкой (в dp → px).
Sticky-логику трекпада не трогай.
`origin()`, `normal()`, `distance()` у контроллеров уже есть (ExtrudeController.h:38, PushPullController.h:61).

## 3. Отрисовка толстой стрелки (touch)
В `src/app/Application_Viewport.cpp`, блок `if (m_extrudeCtl.active()) { ... drawDim(...) }` и ветка Push/Pull (~1796–1850):
если `materializr::touchMode()` — вместо `drawDim` рисуй толстую стрелку через `toucharrow::arrowScreen` (`toImg`
как toScreen — оберни в лямбду с нужной сигнатурой) на `dl` (ImDrawList): контур-ореол тёмный + янтарная линия +
залитый наконечник, размеры из констант п.1 × `uiScale()`; подпись значения рядом с концом стрелки (как сейчас в
drawDim — плашка с текстом). Стартовую двустороннюю стрелку Push/Pull тоже утолщи по тем же константам.
Десктоп — `drawDim` как раньше.

## Ограничения
- Логику изменения значения (`dragBy`, `applyDrag`, `updateExtrude`, `updatePushPull`) не менять.
- `Application_Viewport.cpp`: правило подавления камеры для touch уже handle-aware (строка ~3891, `suppressCamDrag =
  m_inSketchMode ? true : gizmoOwnsDrag`) — не трогай, оно заработает само, когда контроллеры перестанут
  захватывать каждое нажатие.
- Строка в `FORK-CHANGES.md`.

## Самопроверка
Перечитай; в отчёте — как совмещены геометрия hit-test и отрисовки (одна функция), что изменилось на десктопе (ничего).

## Критерии приёмки (куратор, планшет)
1. Extrude по скетчу: толстая стрелка; тянуть за неё — меняется значение; свайп мимо — модель вращается.
2. Push/Pull по грани: стартовая двусторонняя стрелка толстая; тянуть за неё — работает; мимо — вращение.
3. Offset Face работает как раньше.
