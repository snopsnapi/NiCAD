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
3. Claude ревьюит diff. Типичные промахи: выдуманные функции ImGui/SDL3/OCCT, размеры в пикселях вместо dp,
   правки вне рамок ТЗ.
4. Цель — только Android. Сборка — GitHub Actions форка (на каждый push только APK; тесты геометрии —
   вручную `gh workflow run tests-linux.yml --ref nikaishi`, когда трогаем геометрию).
   Сборка — GitHub Actions форка (локально компилятора нет): лог ошибок через `gh run view --log-failed`,
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

## Грабли (выучено 31.10)
- Установка на Xiaomi: `adb install` показывает запрос «Установить» на планшете — пользователь должен
  быть рядом; иначе `INSTALL_FAILED_USER_RESTRICTED`. Нужен «Установка через USB» в настройках разработчика.
- CI-сборки подписаны постоянным ключом (секрет `DEBUG_KEYSTORE_B64`), ставятся поверх (`adb install -r`).
- Git Bash коверкает пути устройства (`/sdcard/...`) — `export MSYS_NO_PATHCONV=1` перед adb push/shell.
- ID сессии Kimi содержит дефисы: `grep -o "kimi -r session_[0-9a-f-]*"`.
- После `git merge` с конфликтом НЕ делать `commit -a` в той же команде — сначала проверить маркеры
  (`grep -rln "^<<<<<<<"`). `FORK-CHANGES.md` мержится union (`.gitattributes`).
- Пуш во время сборки отменяет её (android.yml cancel-in-progress); тесты стоят в очереди.
- Квота Kimi кончается посреди задачи — проверять незаконченную работу в worktree и доделывать.
- Мультитач (twist/pinch) adb не проверить: логика жестов — особенно внимательное ревью.

## Git
- Ветка работы: `nikaishi`. `upstream` = materializr-cad/materializr, `origin` = форк пользователя.
- GPLv3: копирайты автора не трогать; список изменений форка — `FORK-CHANGES.md` (GPLv3 §5a).
- Коммит/push — только когда пользователь согласен; `logs/` и `.claude/settings.local.json` в .gitignore.

## Источники для переноса
Nikaishi (FreeCAD-верстак пользователя): `C:\Users\SnopS\FreeCADAddons\ReverseTools` — математика
сплайнов/поверхностей на Python поверх OCCT, переносится в C++. Его `README.md`/`CLAUDE.md` — читать.
