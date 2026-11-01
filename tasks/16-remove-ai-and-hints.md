# ТЗ-16. Убрать AI-ассистента и оставшиеся описания

## 1. AI-ассистент — выпилить (пользователь: «точно не нужен»)
Android-сборка его уже не компилирует (`android/app/jni/src/CMakeLists.txt:67-68` исключает `/src/ai/` и
`AiAssistantPlugin.cpp`), но в интерфейсе могут остаться пункты/настройки. Сделать:
- найди все упоминания в UI: `grep -rn -i "AI Assistant\|AiAssistant\|ai assistant\|aiSettings\|m_aiSettings" src --include=*.cpp`
  (Application_Dialogs.cpp — настройки AI, меню, кнопки, тур) и убери пункты меню/кнопки/вкладку настроек
  AI из интерфейса (оберни в `#if 0` или удали вызовы отрисовки; сами файлы `src/ai/*` не удаляй — сборка
  тестов на Linux может на них ссылаться, проверь `tests/CMakeLists.txt`);
- ничего, что связано с AI, не должно быть видно на планшете.

## 2. Оставшиеся описания (продолжение ТЗ-09)
Пользователь нашёл: панель ввода длины линии в скетче показывает «Type a value and press Enter. The shape
extends from your first click toward the cursor.» (`src/app/Application_Viewport.cpp` ~7982). Оберни в
`materializr::showPanelHints()` (функция уже есть в `src/ui/UiTheme.h`). Пройди этот же файл и
`Application_Dialogs.cpp` поиском похожих поясняющих фраз (длинные `TextWrapped`/`TextDisabled` с объяснениями,
которые ТЗ-09 пропустил) и оберни так же. Ошибки и статусы не трогать.

## Ограничения
Строка в `FORK-CHANGES.md`.
## Отчёт
Список скрытых мест (файл:строка → первые слова).
