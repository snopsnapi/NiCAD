# ТЗ-05. Offset Face — геометрическое ядро + тесты (без UI)

## Цель
Операция как Offset Face во Fusion: выбранные грани тела (плоские И кривые) сдвигаются по нормали на
`distance`, соседние грани продлеваются/подрезаются до них (не «призма сверху», как Push/Pull).
При сдвиге вперёд (`distance > 0`) фаски и скругления, примыкающие к сдвигаемой грани, удаляются.
В этом ТЗ — только чистые функции геометрии и gtest-тесты. Операция истории и UI — следующее ТЗ.

## Новые файлы
`src/modeling/OffsetFace.h`, `src/modeling/OffsetFace.cpp`, `tests/test_offset_face.cpp`.

### API (ровно такие сигнатуры)
```cpp
#pragma once
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <string>
#include <vector>

namespace materializr {
namespace offsetface {

struct Options {
    // Remove fillets/chamfers that touch an offset face before offsetting.
    // Applied only when distance > 0 (moving the face outwards).
    bool removeAdjacentBlends = true;
    // A neighbour counts as a blend only if it is at most this wide (mm).
    double maxBlendWidth = 10.0;
};

struct Result {
    TopoDS_Shape shape;                       // valid solid on success
    std::vector<TopoDS_Face> removedBlends;   // blend faces of the INPUT body that were removed
    std::string error;                        // human-readable reason on failure
};

// Faces of `body` adjacent to `face` (sharing an edge) that look like a fillet or chamfer:
//  * fillet  - non-planar face (cylinder / torus / sphere / B-spline / revolution) tangent (normals within 1 deg)
//              to `face` along the shared edge AND tangent to the face on its opposite side;
//  * chamfer - planar face, NOT tangent to `face` (angle between normals 5..85 deg) and narrow.
// "Narrow": the largest distance from the shared edge to any vertex of the candidate <= maxWidth.
// Never returns `face` itself. Order: TopExp map order (deterministic).
std::vector<TopoDS_Face> findAdjacentBlends(const TopoDS_Shape& body, const TopoDS_Face& face,
                                            double maxWidth);

// Offset `faces` of `body` by `distance` along their outward normals; all other faces offset 0.
// Returns false and fills r.error on failure; on success r.shape is a BRepCheck-valid solid.
bool offsetFaces(const TopoDS_Shape& body, const std::vector<TopoDS_Face>& faces,
                 double distance, const Options& opt, Result& r);

} // namespace offsetface
} // namespace materializr
```

### Алгоритм `offsetFaces`
1. Проверки: тело не пустое, грани не пустые, `|distance| > Precision::Confusion()`. Всё в `try { } catch (...)`
   (+ `Standard_Failure`) → `r.error`.
2. Если `opt.removeAdjacentBlends && distance > 0`: собрать blend-грани для всех `faces` (без дублей, и не входящие
   в `faces`), убрать их `BRepAlgoAPI_Defeaturing` — точно как в `src/modeling/DefeatureOp.cpp`
   (`SetShape`, `AddFacesToRemove`, `SetRunParallel(Standard_False)`, `Build`, `HasErrors`). Выбранные грани в
   новом теле найти через историю: `df.Modified(f)` (если список пуст и `!df.IsDeleted(f)` — грань та же).
   Записать удалённые в `r.removedBlends`.
3. Офсет: `BRepOffset_MakeOffset`:
   `mo.Initialize(shape, 0.0, Precision::Confusion(), BRepOffset_Skin, Standard_False, Standard_False,
   GeomAbs_Intersection, Standard_False, Standard_False);` затем для каждой (перенайденной) грани
   `mo.SetOffsetOnFace(face, distance);` затем `mo.MakeOffsetShape();`
   **Сверь сигнатуры `Initialize` / `SetOffsetOnFace` / `MakeOffsetShape` / `IsDone` / `Error` с заголовком
   `BRepOffset_MakeOffset.hxx` OCCT 7.9** (в Android-сборке он в `~/Android/prefix/arm64-v8a/include/opencascade/`
   — локально его нет; найди использование `BRepOffset`/`BRepOffsetAPI` в `src/modeling/ShellOp.cpp` как образец
   стиля и укажи в отчёте, что сигнатуру сверял по документации OCCT 7.9, а не по коду проекта).
4. Если `!mo.IsDone()` или результат пустой → `r.error = "offset failed (code N)"`, false.
5. Результат привести к одному solid (если пришла shell — `BRepBuilderAPI_MakeSolid`), проверить
   `BRepCheck_Analyzer(...).IsValid()`, иначе false.

### Регистрация в сборке
- Корневой `CMakeLists.txt`: добавь `src/modeling/OffsetFace.cpp` в список исходников рядом с
  `src/modeling/DefeatureOp.cpp` (~строка 351).
- `tests/CMakeLists.txt`: добавь `${CMAKE_SOURCE_DIR}/src/modeling/OffsetFace.cpp` в `materializr_core` рядом с
  `DefeatureOp.cpp` (~строка 89) и тест по образцу `test_chamfer_at_limit` (`add_executable`,
  `target_link_libraries(... PRIVATE materializr_core gtest gtest_main)`, `add_test`).
- Android/iOS подхватят глобом сами. Если для `BRepOffset_MakeOffset` нужен тулкит OCCT, которого нет в списке
  линковки (TKOffset), — проверь `CMakeLists.txt`, `tests/CMakeLists.txt`, `android/app/jni/src/CMakeLists.txt`
  и напиши в отчёте; `ShellOp` уже использует офсет, так что скорее всего есть.

## Тесты `tests/test_offset_face.cpp` (gtest, образец — `tests/test_chamfer_at_limit.cpp`)
Объём — `BRepGProp::VolumeProperties`, допуск 1e-3 относительный. Грани — счёт `TopExp_Explorer(..., TopAbs_FACE)`.
Верхняя грань ищется как плоская с нормалью +Z на максимальной высоте (напиши хелпер).
1. `Box_TopUp`: куб 20×20×20 (`BRepPrimAPI_MakeBox`), верх +5 → объём 10000, 6 граней, валидный, max Z = 25.
2. `Box_TopDown`: верх −5 → объём 6000.
3. `ChamferedBox_TopUp_RemovesChamfers`: куб 20, фаска 2 мм на 4 верхних рёбрах (`BRepFilletAPI_MakeChamfer`),
   `findAdjacentBlends(top)` → 4 грани; `offsetFaces(+3)` → 6 граней, объём 20·20·23 = 9200, `removedBlends.size()==4`.
4. `FilletedBox_TopUp_RemovesFillets`: то же со скруглением r=2 (`BRepFilletAPI_MakeFillet`) → 6 граней, объём 9200.
5. `ChamferKeptWhenDisabled`: как 3, но `removeAdjacentBlends=false` → `removedBlends` пуст, объём больше,
   чем у исходного тела с фаской (результат валидный ИЛИ осмысленная ошибка — тест проверяет, что не падает).
6. `Cylinder_SideOut`: цилиндр r=10, h=20, боковая грань +2 → объём π·12²·20.
7. `PlainBox_NoBlends`: у куба `findAdjacentBlends(top)` пусто.
8. `EmptyInput_Fails`: пустой список граней → false, `error` не пуст.

## Ограничения
- Ничего не меняй в существующих операциях и UI. Только новые файлы + регистрация в двух CMakeLists.
- Комментарии на английском. Строка в `FORK-CHANGES.md`.

## Самопроверка
Компилятора нет. Перечитай всё; для каждого OCCT-класса укажи в отчёте, где в `src/` есть его `#include`
(или что его там нет). Куратор добавит `test_offset_face` в список `TEST_TARGETS` CI и пришлёт ошибки.

## Критерии приёмки
`test_offset_face` собирается и проходит в CI (Linux). Если какой-то тест падает из-за ограничений OCCT —
куратор решает, менять алгоритм или ожидание.
