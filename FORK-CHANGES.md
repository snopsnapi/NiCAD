# Изменения форка

Этот форк основан на [Materializr](https://github.com/materializr-cad/materializr)
(GPLv3 + дополнительные разрешения, см. `LICENSE`, `LICENSE-EXCEPTIONS.md`).
Ниже — что изменено относительно оригинала (GPLv3 §5a). Полная история — в git.

| Дата | Изменение | Файлы |
|---|---|---|
| 2026-10-31 | Схема работы куратор/исполнитель: CLAUDE.md, AGENTS.md, BACKLOG.md, tasks/ | — |
| 2026-10-31 | CI: сборка debug-APK для планшета (workflow android.yml, nikaishi); appLabel через MATERIALIZR_APP_LABEL; setup-deps.sh пропускает пересборку OCCT при наличии libTKernel.so | .github/workflows/android.yml, android/app/build.gradle, android/app/src/main/AndroidManifest.xml, android/scripts/setup-deps.sh |
| 2026-10-31 | ТЗ-03: бледная заливка всех замкнутых нарисованных профилей видимых скетчей (Fusion-style), флаг m_shadeProfiles; кэш набора граней m_profileShadeCache (ключ — regionKey XOR список заливок, drawnRegions не гоняется per-frame); активный скетч пропускается во время размещения фигуры | src/app/Application.h, src/app/Application_Viewport.cpp |
