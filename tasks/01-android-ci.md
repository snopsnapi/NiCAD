# ТЗ-01. Сборка Android APK в GitHub Actions + отдельный ID приложения

## Цель
На каждый push в ветку `nikaishi` GitHub Actions собирает debug-APK (arm64-v8a) и выкладывает его
артефактом. APK ставится на планшет РЯДОМ с официальным Materializr из Google Play, не заменяя его.

Локально компилятора нет — ничего не собирай и не запускай Gradle. Только пишешь файлы.

## 1. Отдельный ID и имя приложения
Файлы: `android/app/build.gradle`, `android/app/src/main/AndroidManifest.xml`.

- `applicationId` уже читается из env `MATERIALIZR_APP_ID` (см. `defaultConfig`). Ничего в этой логике
  не меняй — CI передаст `MATERIALIZR_APP_ID=com.nikaishi.materializr`.
- Имя на иконке: добавь в `defaultConfig`
  ```groovy
  manifestPlaceholders = [appLabel: (System.getenv('MATERIALIZR_APP_LABEL') ?: '@string/app_name')]
  ```
  и в `AndroidManifest.xml` замени у `<application ...>` и у активности (строки ~66 и ~77)
  `android:label="@string/app_name"` на `android:label="${appLabel}"`.
  Если в `defaultConfig` уже есть `manifestPlaceholders` — дополни существующий, не перезаписывай.
  Без env всё ведёт себя как раньше (десктоп/F-Droid-сборка автора не меняется).

## 2. Пропуск пересборки OpenCASCADE, если он уже есть
Файл: `android/scripts/setup-deps.sh`.

Сейчас скрипт всегда пересобирает FreeType и OCCT (~40 мин). В CI префикс будет восстанавливаться из кэша.
Оберни блоки сборки FreeType и OCCT (от `fetch ".../freetype..."` до `"$CMAKE_BIN" --build "$BUILD/occt-$ABI" --target install ...`
включительно) в условие:
```bash
if [ -f "$PREFIX/lib/libTKernel.so" ] && [ "${FORCE_NATIVE_REBUILD:-0}" != "1" ]; then
    echo "OCCT already in $PREFIX - skipping FreeType/OCCT build (FORCE_NATIVE_REBUILD=1 to rebuild)"
else
    ... существующие команды без изменений ...
fi
```
Загрузка SDL3 + `ln -sfn` (до этого блока) и `bash "$REPO/android/copy-occt-libs.sh" "$ABI"` (после) — должны
выполняться ВСЕГДА. Больше в скрипте ничего не меняй.

## 3. Workflow `.github/workflows/android.yml`
- Имя: `Android Build (nikaishi)`.
- Триггеры: `push` в ветку `nikaishi` с `paths-ignore: ['**.md', 'tasks/**', 'docs/**']`; `workflow_dispatch`.
- `concurrency: { group: android-${{ github.ref }}, cancel-in-progress: true }`.
- Один job, `runs-on: ubuntu-24.04`, `timeout-minutes: 150`.
- Шаги:
  1. `actions/checkout@v4`.
  2. `actions/setup-java@v4` — `distribution: temurin`, `java-version: '17'`.
  3. Установка пакетов SDK (на раннере уже есть `$ANDROID_HOME` и `sdkmanager` в
     `$ANDROID_HOME/cmdline-tools/latest/bin`):
     `yes | sdkmanager --licenses >/dev/null || true`, затем
     `sdkmanager "platforms;android-35" "ndk;26.3.11579264" "ndk;28.2.13676358" "cmake;3.22.1"`.
  4. `actions/cache@v4` для `~/Android/prefix` и `~/Android/dl` с ключом
     `occt-arm64-${{ hashFiles('android/scripts/setup-deps.sh', 'android/copy-occt-libs.sh') }}`
     (без restore-keys — при смене скрипта собираем заново).
  5. Нативные зависимости:
     ```bash
     export ANDROID_NDK="$ANDROID_HOME/ndk/26.3.11579264"   # OCCT собирается r26, как у автора
     JOBS=4 bash android/scripts/setup-deps.sh
     ```
  6. Стабильный debug-ключ (иначе каждый билд подписан новым ключом и APK не ставится поверх):
     если секрет `DEBUG_KEYSTORE_B64` не пуст — `mkdir -p ~/.android && echo "$DEBUG_KEYSTORE_B64" | base64 -d > ~/.android/debug.keystore`.
     Секрет передавай через `env:` шага, не подставляй `${{ secrets... }}` прямо в текст скрипта.
  7. Сборка:
     ```bash
     cd android
     chmod +x gradlew
     MATERIALIZR_APP_ID=com.nikaishi.materializr MATERIALIZR_APP_LABEL="Materializr N" \
     MATERIALIZR_OCCT_PREFIX="$HOME/Android/prefix/arm64-v8a" ./gradlew assembleDebug --stacktrace
     ```
  8. `actions/upload-artifact@v4`: имя `materializr-n-apk`, путь
     `android/app/build/outputs/apk/debug/*.apk`, `retention-days: 14`, `if-no-files-found: error`.
  9. Если секрета `DEBUG_KEYSTORE_B64` нет — `actions/upload-artifact@v4` с именем `debug-keystore`,
     путь `~/.android/debug.keystore`, `retention-days: 3` (куратор сохранит его в секрет).
- `permissions: contents: read`.

## Ограничения
- Существующие workflow автора (`linux.yml`, `windows.yml`, `macos.yml`, `checks.yml`, `prune-artifacts.yml`) не трогай.
- Не добавляй подпись release, публикацию, релизы.
- Никаких секретов/паролей в файлах.

## Самопроверка
- `python -c "import yaml,sys; yaml.safe_load(open('.github/workflows/android.yml', encoding='utf-8')); print('ok')"`
- `bash -n android/scripts/setup-deps.sh` (Git Bash есть) — синтаксис скрипта.
- Покажи в отчёте `git diff` для `build.gradle`, `AndroidManifest.xml`, `setup-deps.sh`.
- Добавь строку в `FORK-CHANGES.md`.

## Критерии приёмки (проверяет куратор)
- Workflow стартует на push в `nikaishi`, собирает APK, артефакт `materializr-n-apk` скачивается.
- Повторный запуск берёт OCCT из кэша (шаг 5 занимает минуты, а не ~40).
- APK ставится на планшет как отдельное приложение «Materializr N» (`com.nikaishi.materializr`) и запускается.
