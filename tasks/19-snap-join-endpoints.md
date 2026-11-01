# ТЗ-19. Привязки v2, часть 1: соединение концов (сплайн замыкается наведением конца на начало)

## Цель (пользователь, «как во Fusion»)
Тянешь точку скетча (конец сплайна/линии) пальцем к другой точке → когда она в радиусе захвата, появляется
привязка (маркер Endpoint, точка «прилипает») → отпускаешь → точки сливаются в одну.
Частный случай: конец открытого сплайна наведён на НАЧАЛО того же сплайна → сплайн становится ЗАМКНУТЫМ.

## Сейчас
- Перенос точки: `SketchTool::onMouseMove` ветка `if (m_isDragging && m_sketch)` (~строка 255–330). Для одной
  точки в touch я отключил все магниты: `glm::vec2 inferred = touchMode() ? target : snap(target);` (~300) —
  пользователь жаловался, что точка липнет к осям/линиям. Магнит к ДРУГИМ ТОЧКАМ нужно вернуть (только он).
- Сплайн хранит `controlPointIds`; замкнутый = `front() == back()` (см. `Sketch::sampleSpline2D`).
- Слияния точек в Sketch нет (`grep -n merge src/modeling/Sketch.h`). `findCoincidentPoint(pos, excludeId)` —
  поиск ближайшей точки в радиусе «сварки».
- Undo переноса: снимок `m_sketchDragBefore` в `Application_Viewport.cpp` и коммит `SketchEditOp` на отпускании
  (~7250–7275, сравнивает позиции и КОЛИЧЕСТВО точек — слияние меняет количество, значит закоммитится).

## Сделать
### 1. `Sketch::mergePoints(int fromId, int intoId)` — `src/modeling/Sketch.{h,cpp}`
Все ссылки на `fromId` в линиях (`startPointId/endPointId`), окружностях (`centerPointId`), дугах
(`centerPointId/startPointId/endPointId`), сплайнах (`controlPointIds`), полигонах (`centerPointId`,
`vertexPointIds`) и ограничениях (посмотри, как `SketchConstraints.h`/Sketch хранит ссылки на точки) заменить на
`intoId`; точку `fromId` удалить. Затем почистить вырожденное:
- линия, у которой оба конца стали одной точкой — удалить;
- сплайн: подряд идущие одинаковые id схлопнуть; если после этого `front()==back()` и уникальных ≥3 — это
  ЗАМКНУТЫЙ сплайн (оставить так); если уникальных < 2 — удалить сплайн.
Вернуть `bool` (было ли слияние). Найди, как Sketch инвалидирует кэши при изменениях (`geometryHash` /
`m_regionCacheValid`) и делай так же.

### 2. Магнит к точкам при переносе (touch)
В ветке одиночного переноса (`pts.size() == 1`): вместо `target` без привязок — искать ближайшую точку, кроме
переносимой, в радиусе 24 dp (`24 * uiScale() * m_mmPerPixel`); нашлась — `inferred = её позиция`, запомнить
`m_dragSnapTargetId`, показать гайд `InferenceGuide::Endpoint` (как это делает `snap()` — найди, как заполняется
`m_activeInferences`, и добавь туда такой же Endpoint-гайд на эту точку). Не нашлась — `inferred = target`,
`m_dragSnapTargetId = -1`. Никаких других магнитов (оси, линии) — как сейчас.
### 3. Слияние на отпускании
В `SketchTool::onMouseUp`: если был перенос одной точки и `m_dragSnapTargetId >= 0` →
`m_sketch->mergePoints(m_dragPointId, m_dragSnapTargetId)`; сбросить. Undo пойдёт через существующий
`SketchEditOp` (проверь, что сравнение «до/после» в Application ловит изменение количества точек — да, есть
`before.size() != after.size()`).
Если переносится незаконченный сплайн (`m_activeSplineId`, `m_splinePoints`) — после слияния синхронизируй
`m_splinePoints` (замени id) или, если сплайн замкнулся, заверши его размещение (`m_splinePoints.clear();
m_activeSplineId = -1; m_isPlacing = false;`).
### 4. Тесты — `tests/test_sketch_merge.cpp` (+ регистрация как `test_offset_face`)
- открытый сплайн из 5 точек: `mergePoints(last, first)` → сплайн замкнут (`front()==back()`, 5 уникальных),
  точек в скетче 5;
- линия A–B и линия C–D: `mergePoints(C, B)` → у второй линии `startPointId == B`, точка C удалена;
- линия A–B: `mergePoints(B, A)` → линия удалена;
- сплайн из 2 точек: `mergePoints(второй, первый)` → сплайн удалён.

## Ограничения
Только `Sketch.*`, `SketchTool.*`, новый тест + CMake тестов. Строка в `FORK-CHANGES.md`.

## Самопроверка
Перечитай; в отчёте — какие поля с id точек ты нашёл в Sketch (полный список) и все ли обработаны.

## Критерии приёмки
- CI-тест `test_sketch_merge` зелёный (куратор добавит в TEST_TARGETS и запустит).
- Планшет: Select → выделить конец сплайна → тянуть к началу → маркер привязки → отпустить → сплайн замкнут
  (появляется заливка контура); «Назад» откатывает.
