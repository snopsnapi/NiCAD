# ТЗ-06. Сплайн через точки (перенос из Nikaishi) — математика + эталонные тесты

## Цель
Перенести в C++ математику сплайна «Studio Spline» из FreeCAD-верстака пользователя Nikaishi:
`C:\Users\SnopS\FreeCADAddons\ReverseTools\rt_spline.py`, функция `interpolate(points, constraints, degree)`
(прочитай её целиком, ~60 строк). Это B-сплайн через точки: хордовая параметризация, произвольная степень,
в любой точке можно задать 1-ю (G1) и 2-ю (G2) производную. Код пользовательский, переносить можно.
Потом на нём будут построены и 2D-сплайны скетча, и 3D-сплайны. В этом ТЗ — только математика и тесты,
никакого UI и никаких изменений в `Sketch`.

## 1. Эталонные данные (Python, запускаешь сам)
Создай `tools/golden/spline_golden.py`:
- скопируй в него `interpolate()` из `rt_spline.py` **без FreeCAD**: вместо построения `Part.BSplineCurve` верни
  `dict(degree=p, knots=<полный вектор узлов, с повторами>, poles=<список [x,y,z]>, params=<t каждой точки>)`;
  numpy/scipy установлены (`python -c "import numpy, scipy"` работает);
- набор случаев (имя → points, constraints, degree):
  1. `two_points` — 2 точки, degree 3;
  2. `three_points_deg3` — 3 точки, degree 3 (степень сама снизится);
  3. `five_3d` — 5 точек в пространстве, degree 3;
  4. `seven_deg5` — 7 точек, degree 5;
  5. `planar_six` — 6 точек с z=0, degree 3;
  6. `end_tangents` — 5 точек, d1 в первой и последней (d2=None);
  7. `g2_middle` — 6 точек, d1 и d2 в точке с индексом 2;
  8. `g1_all_ends_deg5` — 6 точек, degree 5, d1 на концах.
  Координаты — «неровные» числа (например, 12.3, -4.71), не на сетке;
- пишет `tests/data/spline_golden.json`: список `{name, degree_in, points, constraints: [{index, d1, d2|null}], out: {...}}`.
Запусти: `python tools/golden/spline_golden.py` → файл создан, приложи в отчёт его размер и число случаев.

## 2. C++: `src/modeling/FitSpline.h`, `src/modeling/FitSpline.cpp`
```cpp
#pragma once
#include <glm/glm.hpp>
#include <optional>
#include <vector>
#include <Geom_BSplineCurve.hxx>

namespace materializr {
namespace fitspline {

struct Constraint {
    int index = 0;                       // which input point
    glm::dvec3 d1{0.0};                  // 1st derivative wrt the [0,1] parameter
    std::optional<glm::dvec3> d2;        // optional 2nd derivative
};

struct Curve {
    int degree = 0;
    std::vector<double> knots;           // FULL knot vector (with repeats), length = poles + degree + 1
    std::vector<glm::dvec3> poles;
    std::vector<double> params;          // parameter of each input point, in [0,1]
};

// Port of rt_spline.interpolate(): chord-length parameters, collocation matrix with averaged
// knots, degree = clamp(degree, needed multiplicity, sites-1); 2 points -> single span.
// Returns false for < 2 points or coincident consecutive points.
bool interpolate(const std::vector<glm::dvec3>& points, const std::vector<Constraint>& constraints,
                 int degree, Curve& out);

// Closed curve through the points (>= 3), periodic, via OCCT GeomAPI_Interpolate (as rt_spline does
// with PeriodicFlag). Null handle on failure.
Handle(Geom_BSplineCurve) interpolateClosed(const std::vector<glm::dvec3>& points);

// Curve -> OCCT curve (knots collapsed into distinct values + multiplicities).
Handle(Geom_BSplineCurve) toOcct(const Curve& c);

// Point / derivative on a Curve (own de Boor evaluation, no OCCT) - used by tests and UI previews.
glm::dvec3 evaluate(const Curve& c, double t, int derivative = 0);

} // namespace fitspline
} // namespace materializr
```
Требования:
- Базисные функции и их производные пиши **сами** (алгоритм A2.3 `DersBasisFuns` из The NURBS Book,
  Piegl & Tiller) — не используй `BSplCLib`, чтобы не гадать с его API. Поиск интервала — A2.1 `FindSpan`,
  при `t == 1` — последний ненулевой интервал (как scipy).
- Линейная система n×n: своя Гауссова элиминация с выбором главного элемента (n обычно < 100). Если матрица
  вырождена — МНК не нужен, верни false (в Python там lstsq — в отчёте отметь это отличие).
- Узлы: как в Python — `inner = [mean(tau[j:j+p]) for j in 1..n-p-1]`, clamped с обоих концов.
- `toOcct`: `Geom_BSplineCurve(TColgp_Array1OfPnt, TColStd_Array1OfReal knots, TColStd_Array1OfInteger mults, degree)`
  — найди этот конструктор в `src/` (`grep -rn "Geom_BSplineCurve(" src`), сверь с использованием там.
- `interpolateClosed`: `GeomAPI_Interpolate` — найди использование в `src/` и сделай так же (periodic = true).
- Комментарии на английском, сослаться на `rt_spline.py` как источник.

## 3. Тест `tests/test_fit_spline.cpp` (gtest)
- Читает `tests/data/spline_golden.json` (путь — через `#define` от `CMAKE_SOURCE_DIR`: в `tests/CMakeLists.txt`
  добавь `target_compile_definitions(test_fit_spline PRIVATE MZR_TEST_DATA_DIR="${CMAKE_SOURCE_DIR}/tests/data")`;
  JSON парсить через `nlohmann/json` — проверь, что он доступен тестам (`grep -rn nlohmann tests/CMakeLists.txt CMakeLists.txt`);
  если нет — напиши в отчёте и сделай простой парсер не нужно: тогда генерируй из Python вместо JSON
  заголовок `tests/data/spline_golden.inc` с C++-инициализаторами и подключай его).
- Для каждого случая: degree совпадает; knots и poles совпадают с эталоном с допуском 1e-9·(1+|x|); params тоже.
- Дополнительно (без эталона): `evaluate(c, params[i])` == points[i] (1e-9); для точек с d1 — `evaluate(c, t, 1)` == d1
  (1e-7); с d2 — 2-я производная (1e-6); `toOcct(c)->Value(t)` == `evaluate(c, t)` в 20 точках (1e-9).
- `Closed`: 6 точек на окружности-эллипсе → `interpolateClosed` не null, `IsPeriodic()`, проходит через все точки (1e-7).
- Регистрация: `FitSpline.cpp` в `materializr_core` (`tests/CMakeLists.txt`) и в список исходников корневого
  `CMakeLists.txt` (рядом с `src/modeling/Sketch.cpp`), тест — по образцу `test_chamfer_at_limit`.

## Ограничения
- Не трогай `Sketch`, `SketchTool`, рендер и т.д. Только новые файлы + регистрация + генератор + данные.
- Строка в `FORK-CHANGES.md` (укажи, что математика перенесена из Nikaishi `rt_spline.py`).

## Самопроверка
- Python-генератор запущен, JSON/INC создан.
- Если можешь — проверь свою C++-логику, переписав её 1:1 на Python в `tools/golden/check_port.py` (свой
  DersBasisFuns + Гаусс) и сравнив с эталоном: это поймает ошибки в алгоритме до CI. Приложи вывод.

## Критерии приёмки
`test_fit_spline` проходит в CI (куратор добавит его в `TEST_TARGETS`).
