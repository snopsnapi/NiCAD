# ТЗ-04. Тесты геометрии в GitHub Actions (Linux)

## Цель
Компилятора локально нет, а геометрию (Offset Face, сплайны и т.п.) на планшете тапами не проверить.
Нужен workflow, который на каждый push в `nikaishi` собирает на Linux выбранные gtest-тесты из `tests/`
и прогоняет их. Новые тесты потом добавляются одной строкой в список.

Ничего не собирай локально — только пишешь файл workflow.

## Как устроено у автора (изучи сам, прежде чем писать)
- `tests/CMakeLists.txt`: библиотека `materializr_core` (исходники `src/core`, `src/modeling`, …),
  у каждого теста свой `add_executable(test_x test_x.cpp)` + `add_test(NAME test_x ...)`. Тесты подключаются из
  корневого `CMakeLists.txt` при `-DBUILD_TESTS=ON`.
- Корневой `CMakeLists.txt` ищет SDL3 (или тянет FetchContent), OpenGL, OpenCASCADE (`-DMZR_OCCT_PREFIX=<prefix>`
  → чистый `find_package(OpenCASCADE CONFIG ...)`), ZLIB, Threads, GLEW (проверь, при каком условии), CURL.
- `Dockerfile` (строки 1–65): точный список apt-пакетов для Ubuntu 24.04 и сборка OCCT 7.9.3 из исходников с
  нужными флагами (тег и sha256 те же, что в `android/scripts/setup-deps.sh`). Бери оттуда, не выдумывай.

## Workflow `.github/workflows/tests-linux.yml`
- Имя `Geometry Tests (nikaishi)`. Триггеры: `push` в `nikaishi` с `paths-ignore: ['**.md', 'tasks/**', 'docs/**', 'android/**']`;
  `workflow_dispatch`. `concurrency` как в `android.yml`. `permissions: contents: read`.
- `runs-on: ubuntu-24.04`, `timeout-minutes: 120`.
- `env: TEST_TARGETS: "test_blend_cut test_chamfer_at_limit"` — список целей через пробел
  (сначала проверь, что такие `add_executable` реально есть в `tests/CMakeLists.txt`; если нет — возьми
  2 существующих теста про фаски/скругления/push-pull и напиши в отчёте, какие).
- Шаги:
  1. checkout.
  2. apt-пакеты — список из `Dockerfile` (без `file patchelf fuse libfuse2 imagemagick zsync` — они для AppImage),
     плюс `ninja-build ccache` и `libglew-dev`, если GLEW нужен.
  3. OCCT: `actions/cache/restore@v4` для `~/occt-linux` с ключом `occt-linux-V7_9_3-v1`; если промах — скачать,
     проверить sha256, собрать с флагами из `Dockerfile` в `CMAKE_INSTALL_PREFIX=$HOME/occt-linux`
     (`-G Ninja`), и сразу `actions/cache/save@v4` (как в `android.yml`, чтобы кэш не терялся при падении тестов).
  4. ccache: `actions/cache@v4` для `~/.ccache`, ключ `ccache-linux-${{ github.sha }}`, `restore-keys: ccache-linux-`.
  5. Configure:
     ```bash
     cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON \
       -DMZR_OCCT_PREFIX="$HOME/occt-linux" \
       -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
     ```
  6. Build только нужные цели: `cmake --build build --target $TEST_TARGETS -j4`.
  7. Run: `export LD_LIBRARY_PATH="$HOME/occt-linux/lib:$LD_LIBRARY_PATH"`; для каждой цели из `TEST_TARGETS`
     запустить `ctest --test-dir build -R "^${t}$" --output-on-failure` (ошибка любого — job красный).
     Если какой-то тест требует дисплей (GL) — `xvfb-run` не добавляй, а напиши в отчёте.

## Ограничения
- Не трогай `CMakeLists.txt`, `tests/CMakeLists.txt`, исходники и чужие workflow. Если без правки CMake
  не собрать — опиши в отчёте, что именно мешает, правку не делай.
- Строка в `FORK-CHANGES.md`.

## Самопроверка
- `python -c "import yaml; yaml.safe_load(open('.github/workflows/tests-linux.yml', encoding='utf-8')); print('ok')"`.
- В отчёте: откуда взят каждый apt-пакет и флаг OCCT (строки `Dockerfile`), при каком условии нужен GLEW,
  какие цели тестов выбраны.

## Критерии приёмки (куратор)
- Job зелёный, в логе видно, что выбранные тесты запустились и прошли.
- Повторный запуск берёт OCCT из кэша.
