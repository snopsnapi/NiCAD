# ТЗ-07. Offset Face — операция истории + инструмент со стрелкой

## Цель
Ядро уже есть: `src/modeling/OffsetFace.{h,cpp}` (`offsetface::offsetFaces`, `findAdjacentBlends`) — прочитай.
Сделать из него полноценную команду: выбрал грань(и) тела → «Offset Face» → стрелка по нормали, тянешь
пальцем ЗА СТРЕЛКУ (мимо стрелки — обычное вращение камеры), либо вводишь число → живой предпросмотр →
Confirm. Результат — шаг истории, переживает сохранение/загрузку, Undo работает.

## 1. Операция `src/modeling/OffsetFaceOp.{h,cpp}`
Копия устройства `DefeatureOp` (`src/modeling/DefeatureOp.{h,cpp}`) — тот же набор методов и тот же способ
хранить грани (`SubShapeIndex` по входной форме), но:
- поля: `m_bodyId`, `TopTools_ListOfShape m_faces`, `double m_distance = 5.0`, `bool m_removeBlends = true`,
  `m_previousShape`, `m_faceIndices`;
- сеттеры `setBody`, `addFace`, `clearFaces`, `setDistance`, `setRemoveBlends`;
- `execute`: `offsetface::offsetFaces(prev, faces, m_distance, opt{removeAdjacentBlends=m_removeBlends}, r)`;
  при неудаче — `fprintf(stderr, "[OffsetFace] %s\n", r.error)`, false; при успехе `doc.updateBody`;
- `name()` = "Offset Face", `typeId()` = "offset_face", `description()` = "Offset N face(s) by X mm";
- params: `body=..;dist=..;blends=0|1;faces=..` (парсер как в `DefeatureOp::deserializeParams`);
- `renderProperties`: заголовок, число граней, расстояние, флаг.
- Регистрация: `src/modeling/OperationFactory.cpp` (рядом с `"defeature"`), корневой `CMakeLists.txt`
  (рядом с `OffsetFace.cpp`), `tests/CMakeLists.txt` `materializr_core` (рядом с `OffsetFace.cpp`).

## 2. Контроллер `OffsetFaceController` в `src/app/FaceOpControllers.{h,cpp}`
Образцы: `DefeatureController` (сбор граней одного тела в `onBegin`, `buildOp`, `panelBody`, `onCleanup`) и
`ScaleFaceController` (`wantsViewportInput`, `onViewportInput`, `previewOffThread`). Стрелку и её захват
бери из `EdgeOpController` (`src/app/EdgeOpController.cpp`: `onViewportInput` ~строка 613 — `distToSeg`,
`pick` радиус в физ. пикселях через `ctx.panel.uiScale`, `setDraggingHandle(...)`, расчёт значения по лучу
на плоскости, обращённой к камере; отрисовка стрелки — найди, как `EdgeOpController` рисует её
(`drawGizmos3D` / `IopGizmo3D::arrow` или overlay) и сделай так же).
- `onBegin`: грани одного тела (как Defeature). Начало стрелки — центр первой грани (`BRepGProp::SurfaceProperties`
  → центр масс), направление — внешняя нормаль в этой точке (`BRepGProp_Face::Normal`, учитывает ориентацию).
- Захват: ТОЛЬКО если касание рядом со стрелкой: радиус `(imTouch ? 48 : 20) * uiScale` пикселей
  (для пальца шире, чем у EdgeOp). Иначе `setDraggingHandle(false)` и ничего не делаем → камера вращается.
- Значение: со знаком (можно тянуть внутрь — отрицательное), квантуется `materializr::quantiseDragMm` как в EdgeOp;
  ноль не допускается (|d| < 0.01 → не обновлять).
- Панель: поле расстояния (как у EdgeOp/Extrude — `formatLengthDigits`), чекбокс «Remove adjacent fillets/chamfers»
  (по умолчанию вкл.; подсказка «only when moving outwards»), строка статуса превью: зелёная «Previewing» /
  оранжевая с текстом ошибки.
- `previewOffThread() = true` (офсет может быть медленным).
- Тексты — через `materializr::tr("...")` (английские; переводов не добавлять).

## 3. Подключение
По образцу `RemoveFace` / `m_defeatureCtl` — найди все места `grep -rn "RemoveFace\|m_defeatureCtl" src`:
- `ToolAction::OffsetFace` (enum рядом с `RemoveFace`);
- член `OffsetFaceController m_offsetFaceCtl;` в `Application.h` и в списке всех контроллеров (`Application.h` ~1711);
- `case ToolAction::OffsetFace: beginIop(m_offsetFaceCtl); break;` в `Application.cpp` (~2964);
- кнопка в классическом тулбаре `src/ui/Toolbar.cpp` рядом с «Remove Feature» (иконку возьми существующую,
  подходящую по смыслу, например ту же, что у Push/Pull; новых иконок не рисовать);
- пункты в Modern и im-touch раскладках рядом с Remove Feature (`ModernLayout.cpp` ~510, `ImTouchLayout.cpp` ~842 и
  маппинг иконки ~66 для `"offset_face"`).
Если где-то ещё перечисляются все ToolAction (switch без default, таблицы подсказок, KeyBindings) — добавь и туда,
иначе предупреждения/ошибки компилятора.

## Ограничения
- Ядро `OffsetFace.cpp` не меняй. Push/Pull, Extrude и EdgeOp не трогай.
- Строка в `FORK-CHANGES.md`.

## Самопроверка
- Перечитай всё; в отчёте — список всех мест подключения и где взят образец каждого.
- `grep -rn "OffsetFace" src` — все объявления/определения на месте.

## Критерии приёмки (куратор, на планшете)
1. Куб → выбрать верхнюю грань → Offset Face → стрелка вверх; тянуть за стрелку — грань едет, превью есть.
2. Касание мимо стрелки во время операции — модель вращается.
3. Куб с фаской сверху → тянуть вверх — фаска исчезает, рёбра острые.
4. Confirm → шаг в истории; Undo откатывает; сохранить/открыть проект — шаг на месте.
