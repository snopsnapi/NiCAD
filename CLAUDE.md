# Роль Claude в этом проекте

Форк Materializr (C++17 CAD: OpenCASCADE + Dear ImGui + SDL3, Android через NDK) под планшет
пользователя и реверс сканов. Claude — куратор, Kimi Code CLI — исполнитель. Правила для
исполнителя — в `AGENTS.md`. Что делаем и почему — `BACKLOG.md` (собран с пользователем).

## Цикл
1. Claude пишет ТЗ в `tasks/<NN>-<тема>.md`: цель, файлы и функции, готовые сигнатуры,
   ограничения, критерии приёмки, чем Kimi может проверить себя сам.
2. Запуск исполнителя из корня репозитория (PowerShell, в фоне):
   `C:\Users\SnopS\.kimi-code\bin\kimi.exe -p "Выполни ТЗ из tasks/<файл>.md. Следуй AGENTS.md." *> logs/<файл>.log`
   Доработка той же сессии: `kimi.exe -r <session_id> -p "<замечания>"` — ID из конца лога
   («To resume this session: kimi -r ...»). Не `-c` (берёт чужую сессию). `-p` не совмещать с
   `--auto`; слэш-команды в `-p` не работают. Ошибка авторизации → пользователь сам делает
   `kimi.exe login`. Квота Kimi недельная + 5-часовое окно, общая с Kimi Work.
3. Claude ревьюит diff. Типичные промахи: выдуманные функции ImGui/SDL3/OCCT, сломанная
   десктопная сборка (код под `__ANDROID__`/`MZ_MOBILE` без guard), размеры в пикселях вместо dp,
   правки вне рамок ТЗ.
4. Сборка — GitHub Actions форка (локально компилятора нет): лог ошибок через `gh run view --log-failed`,
   APK — `gh run download`. Ошибки компиляции → п.2 с `-r`.
5. Claude проверяет APK на планшете через adb, Kimi устройство не трогает.
6. Замечания → п.2 с `-r`, пока критерии приёмки не выполнены. Затем коммит.

## Планшет
- adb: `C:\Program Files\SuperDisplay\adb\adb.exe` (из SuperDisplay; своего SDK нет).
- Xiaomi 2410CRP4CG, Android 16, arm64-v8a, Adreno 732, 3200×2136 (ландшафт), 440 dpi.
- Там же стоит Materializr из Google Play (`org.ravenhold.materializr`) с проектами пользователя —
  его не трогать. Наша сборка ставится отдельным applicationId (`MATERIALIZR_APP_ID` в gradle).
- Управление: `adb shell input tap/swipe`, `adb exec-out screencap -p > shots/x.png`, `adb logcat`.
  Интерфейс рисуется ImGui в GL — дерева элементов нет, кнопки ищутся по скриншоту.
  Мультитач (щипок, twist) adb не умеет — это проверяет пользователь.

## Git
- Ветка работы: `nikaishi`. `upstream` = materializr-cad/materializr, `origin` = форк пользователя.
- GPLv3: копирайты автора не трогать; список изменений форка — `FORK-CHANGES.md` (GPLv3 §5a).
- Коммит/push — только когда пользователь согласен; `logs/` и `.claude/settings.local.json` в .gitignore.

## Источники для переноса
Nikaishi (FreeCAD-верстак пользователя): `C:\Users\SnopS\FreeCADAddons\ReverseTools` — математика
сплайнов/поверхностей на Python поверх OCCT, переносится в C++. Его `README.md`/`CLAUDE.md` — читать.
