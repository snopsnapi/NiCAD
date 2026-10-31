# Изменения форка

Этот форк основан на [Materializr](https://github.com/materializr-cad/materializr)
(GPLv3 + дополнительные разрешения, см. `LICENSE`, `LICENSE-EXCEPTIONS.md`).
Ниже — что изменено относительно оригинала (GPLv3 §5a). Полная история — в git.

| Дата | Изменение | Файлы |
|---|---|---|
| 2026-10-31 | Схема работы куратор/исполнитель: CLAUDE.md, AGENTS.md, BACKLOG.md, tasks/ | — |
| 2026-10-31 | CI: сборка debug-APK для планшета (workflow android.yml, nikaishi); appLabel через MATERIALIZR_APP_LABEL; setup-deps.sh пропускает пересборку OCCT при наличии libTKernel.so | .github/workflows/android.yml, android/app/build.gradle, android/app/src/main/AndroidManifest.xml, android/scripts/setup-deps.sh |
| 2026-10-31 | Android: свайп «Назад» от правого края / кнопка Back = Undo (в сплайне — удаление последней точки, как Backspace); свайп от левого края игнорируется, приложение не сворачивается | android/app/src/main/java/com/materializr/app/MaterializrActivity.java, android/app/src/main/AndroidManifest.xml, src/android_platform.h, src/android_platform.cpp, src/app/Application.cpp |
| 2026-10-31 | ТЗ-03: бледная заливка всех замкнутых нарисованных профилей видимых скетчей (Fusion-style), флаг m_shadeProfiles; кэш набора граней m_profileShadeCache (ключ — regionKey XOR список заливок, drawnRegions не гоняется per-frame); активный скетч пропускается во время размещения фигуры | src/app/Application.h, src/app/Application_Viewport.cpp |
| 2026-10-31 | CI: прогон gtest-тестов геометрии на Linux (tests-linux.yml, nikaishi): OCCT 7.9.3 из кэша/исходников, цели TEST_TARGETS | .github/workflows/tests-linux.yml |
| 2026-10-31 | ТЗ-05: Offset Face — геометрическое ядро (findAdjacentBlends/offsetFaces) + gtest-тесты, без UI | src/modeling/OffsetFace.h, src/modeling/OffsetFace.cpp, tests/test_offset_face.cpp, CMakeLists.txt, tests/CMakeLists.txt |
