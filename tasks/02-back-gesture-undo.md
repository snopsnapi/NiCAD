# ТЗ-02. Свайп «Назад» от правого края = Undo (пошаговый, в сплайне — по одной точке)

## Цель
Сейчас системный жест «Назад» Android приложение не обрабатывает: он уходит в `SDLActivity.onBackPressed()`
и сворачивает/закрывает приложение. Нужно:
- свайп от **правого** края экрана → тот же Undo, что у кнопки Undo в тач-интерфейсе;
- в инструменте Spline с поставленными точками → удалить **одну последнюю точку** (как Backspace);
- свайп от **левого** края → ничего не делать (и приложение НЕ сворачивать);
- аппаратная/экранная кнопка «Назад» и `adb shell input keyevent KEYCODE_BACK` (край неизвестен) → как правый край.

Только Android. Десктоп и iOS не меняются.

## 1. Java: `android/app/src/main/java/com/materializr/app/MaterializrActivity.java`
Поля:
```java
// Back gestures waiting for native (Undo requests). Read+cleared by pollBackGesture().
private static final java.util.concurrent.atomic.AtomicInteger sPendingBack =
        new java.util.concurrent.atomic.AtomicInteger(0);
private int mBackEdge = -1;   // BackEvent.EDGE_LEFT / EDGE_RIGHT of the current gesture, -1 = unknown
```
Статический метод для native (вызывается каждый кадр):
```java
public static int pollBackGesture() { return sPendingBack.getAndSet(0); }
```
Регистрация в `onCreate` (после `super.onCreate(...)`):
- `Build.VERSION.SDK_INT >= 34`: `getOnBackInvokedDispatcher().registerOnBackInvokedCallback(
  OnBackInvokedDispatcher.PRIORITY_DEFAULT, new OnBackAnimationCallback() {...})`:
  - `onBackStarted(BackEvent e)` → `mBackEdge = e.getSwipeEdge();`
  - `onBackInvoked()` → если `mBackEdge != BackEvent.EDGE_LEFT` — `sPendingBack.incrementAndGet()`; затем `mBackEdge = -1;`
  - `onBackCancelled()` → `mBackEdge = -1;`
  - `onBackProgressed` — не нужен (пустой или не переопределять).
- `Build.VERSION.SDK_INT == 33`: `registerOnBackInvokedCallback(PRIORITY_DEFAULT, () -> sPendingBack.incrementAndGet())`
  (края нет — считаем правым).
- Ниже 33: переопредели `onBackPressed()` в `MaterializrActivity`: `sPendingBack.incrementAndGet();` и **не** вызывай `super`.
Импорты: `android.window.OnBackInvokedDispatcher`, `android.window.OnBackAnimationCallback`, `android.window.BackEvent`.
Классы API 33/34 используй только внутри веток с проверкой `SDK_INT` (minSdk 24).

## 2. Манифест: `android/app/src/main/AndroidManifest.xml`
В тег `<application ...>` добавь `android:enableOnBackInvokedCallback="true"` (без этого на Android 13+
callback не получает события). Остальное не трогай.

## 3. Native: `src/android_platform.h` / `src/android_platform.cpp`
Объяви в `namespace materializr`:
```cpp
// Number of "Back" gestures (right edge / back key) since the last call; each
// one is an Undo request. Always 0 off Android. Call once per frame.
int androidPollBackGesture();
```
Реализация Android — по образцу `androidTopInsetPx()` в том же файле (`SDL_GetAndroidJNIEnv`,
`SDL_GetAndroidActivity`, `GetObjectClass`, `GetStaticMethodID(cls, "pollBackGesture", "()I")`,
`CallStaticIntMethod`, `ExceptionCheck/ExceptionClear`, `DeleteLocalRef`). Без троттлинга.
В десктопной ветке (`#else` в конце файла) — `int androidPollBackGesture() { return 0; }`.

## 4. Обработка: `src/app/Application.cpp`, функция `Application::handleShortcuts()`
Рядом с обработкой `KeyAction::Undo` (~строка 3165) добавь блок:
```cpp
#if defined(__ANDROID__)
    // Android "Back" (right-edge swipe / back key) = touch Undo, one step per gesture.
    for (int n = materializr::androidPollBackGesture(); n > 0; --n) {
        if (m_edgeCtl.active() || m_extrudeCtl.active() || m_ppCtl.active() ||
            !m_deferredHeavy.empty())
            break;   // same guard as Ctrl+Z: never undo under a live/queued operation
        if (m_inSketchMode && m_sketchTool &&
            m_sketchTool->getMode() == SketchToolMode::Spline &&
            !m_sketchTool->splinePointsInProgress().empty()) {
            recordSketchMutation([&] { m_sketchTool->removeLastSplinePoint(); });
        } else if (touchCanUndo()) {
            touchUndo();
        }
    }
#endif
```
Проверь по коду, что все имена (`m_edgeCtl`, `m_extrudeCtl`, `m_ppCtl`, `m_deferredHeavy`,
`SketchToolMode::Spline`, `splinePointsInProgress`, `removeLastSplinePoint`, `recordSketchMutation`,
`touchCanUndo`, `touchUndo`) существуют и доступны в этой функции; блок Backspace для сплайна
(~строка 3437) — образец. Если `handleShortcuts()` на Android не вызывается каждый кадр (проверь, откуда
он вызывается) — напиши в отчёте и поставь блок туда, где вызывается каждый кадр в главном цикле.
`android_platform.h` в `Application.cpp` уже подключён.

## Ограничения
- Ничего не меняй в `org/libsdl/app/*` (это SDL).
- Не трогай десктопную обработку Ctrl+Z/Backspace.
- Строку в `FORK-CHANGES.md`.

## Самопроверка
- Перечитай изменённые файлы целиком; в отчёте — где нашёл образец каждого JNI-вызова.
- `grep -n "pollBackGesture" -r src android` — Java-имя и строка в `GetStaticMethodID` совпадают, сигнатура `()I`.

## Критерии приёмки (куратор, на планшете)
1. `adb shell input keyevent KEYCODE_BACK` после экструда → экструд откатывается, приложение не сворачивается.
2. Сплайн: 4 точки → Back → осталось 3, сплайн не пропал.
3. Свайп от левого края → ничего не происходит, приложение на экране.
4. Свайп от правого края → Undo.
