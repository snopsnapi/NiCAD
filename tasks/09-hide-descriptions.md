# ТЗ-09. Убрать громоздкие описания из панелей

## Цель
Пользователя раздражают абзацы-пояснения в панелях операций и диалогах — например в Offset Face
«Moves the selected face(s) in or out along their normal. Drag the arrow...» или в Import STL «Lower = coarser,
faster, ...». Они занимают пол-экрана планшета. Убрать их ВЕЗДЕ, оставив поля, кнопки, короткие подписи полей,
статусы и ошибки.

## Как
1. В `src/ui/UiTheme.h` (или рядом, где лежат общие UI-хелперы — посмотри, что подключают панели) добавь:
   ```cpp
   // Fork: descriptive help paragraphs in op panels/dialogs are hidden.
   inline bool showPanelHints() { return false; }
   ```
2. Найди все места, где выводится ПОЯСНИТЕЛЬНЫЙ текст (не подпись поля и не статус):
   `grep -rn "TextDisabled\|TextWrapped\|PushTextWrapPos" src --include=*.cpp`, плюс `TextColored` с
   поясняющими (не ошибочными) фразами. Ориентиры: `src/app/FaceOpControllers.cpp`, `EdgeOpController.cpp`,
   `ExtrudeController.cpp`, `PushPullController.cpp`, `SplitController.cpp`, `Application_Dialogs.cpp`
   (импорт STL/SVG/DXF, примитивы, экспорт), `src/ui/PropertiesPanel.cpp`, `src/modeling/*Op.cpp` (`renderProperties`).
3. Каждый такой абзац (вместе с его `PushTextWrapPos/PopTextWrapPos` и `Separator`, если он отделял только этот
   абзац) оберни в `if (materializr::showPanelHints()) { ... }`. Не удаляй текст из кода — только оборачивай,
   чтобы при слиянии с оригиналом не было конфликтов в строках переводов.

## НЕ трогать
- Сообщения об ошибках и предупреждения о результате («Can't remove: ...», «offset failed», оранжевые/красные).
- Статусы превью («Previewing ...»), значения, единицы, подписи полей и чекбоксов, заголовки окон.
- `HelpPanel.cpp`, тур «Getting Started», тексты онбординга, тултипы (`SetTooltip` — они не занимают место).
- Подсказку-баннер в верхней части вьюпорта (`viewportBanner`) — это отдельный вопрос, не трогай.

## Отчёт
Таблица: файл:строка → первые слова скрытого текста. Отдельно — спорные места, которые ты НЕ стал скрывать, и почему.
Строка в `FORK-CHANGES.md`.

## Критерии приёмки (куратор на планшете)
Панели Offset Face, Remove Feature, Shell, Draft, Scale Face, Import STL, New Box — без абзацев-пояснений;
поля и кнопки на месте; ошибки по-прежнему видны.
