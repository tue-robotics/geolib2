# Remove type-punning unions from math_types Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove the anonymous type-punning unions from the five math types in `include/geolib/math_types.h` so the `cppcoreguidelines-pro-type-union-access` clang-tidy warning clears at every access site (including dependent packages), while preserving the public API exactly.

**Architecture:** Each type stores its data in an anonymous `union { struct { T x, y, ...; }; T m[N]; }`. We make the **named members the single source of truth**, delete the `m[N]` array member and the union, and re-route every indexed access (`operator[]`, `operator()`, and internal `m[k]` uses) through a `switch`-based `operator[]`. This is behaviour-preserving: object size, named-member access, `operator[]`, and `operator()` are all unchanged.

**Tech Stack:** Header-only C++17 templates; GoogleTest; ament/colcon build; clang-tidy 21 / clang-format 21 via `ament_cmake`.

## Global Constraints

- **C++ standard:** C++17 (`CMAKE_CXX_STANDARD 17`). No newer features.
- **No `std::variant`:** it holds one alternative at a time and cannot model simultaneous named + array access. "Convert unions to variants" means *eliminate the unions*.
- **No `reinterpret_cast`** for indexed access: it is UB across distinct non-array members. Use `switch`. (The lint check `cppcoreguidelines-pro-type-reinterpret-cast` happens to be disabled, but the choice is for correctness, not lint.)
- **Public API must not change:** member names (`x, y, z, w, xx … zz`), `operator[]`, and `operator()` must behave identically. Only the `m` array *member* is removed.
- **Code style:** must stay clang-format-21 clean — Allman braces (opening brace on its own line), 4-space indent, matching the existing file.
- **Single file:** all production changes are in `include/geolib/math_types.h`. Do not touch `include/geolib/matrix.h` (separate non-union `Vector3`/`Matrix3x3` class).

---

## File Structure

- **Modify:** `include/geolib/math_types.h` — remove unions from `Vec2T`, `Vec3T`, `Mat2T`, `Mat3T`, `QuaternionT`; reimplement `operator[]`; reroute internal array access.
- **Create:** `test/test_math_types.cpp` — GoogleTest characterization tests pinning the access surface of all five types.
- **Modify:** `CMakeLists.txt` — register the new gtest under `if(BUILD_TESTING)`.

### Note on test style (read before Task 1)

These are **characterization tests** for a behaviour-preserving refactor: they pass on the *current* (union) code and must stay green after the refactor. There is no per-type unit-test "red" — the genuine red→green signal for this work is the clang-tidy warning disappearing, verified in the final task. Do not fabricate artificial unit-test failures.

### Fast test loop

`math_types.h` only includes `<cmath>` and `<iostream>`, so the test compiles standalone without ROS:

```bash
g++ -std=c++17 -I include test/test_math_types.cpp -lgtest -lgtest_main -pthread \
    -o /tmp/test_math_types && /tmp/test_math_types
```

Use this for every per-task test run. The full integration + lint run (final task) uses colcon.

---

### Task 1: Characterization tests for all five types

**Files:**
- Create: `test/test_math_types.cpp`
- Modify: `CMakeLists.txt` (inside `if(BUILD_TESTING)`, after the `test_lrf` block at line 218)

**Interfaces:**
- Consumes: `geo::Vec2`, `geo::Vec3`, `geo::Mat2`, `geo::Mat3`, `geo::Quaternion`-equivalent (`geo::QuaternionT<double>`) from `<geolib/math_types.h>`.
- Produces: a gtest binary `test_math_types` that pins, for every type, that `operator[]` aliases the named members, that `(const T*)` constructors populate correctly, and that `operator()` maps `(i,j)` correctly. Later tasks rely on this staying green.

- [ ] **Step 1: Write the characterization tests**

Create `test/test_math_types.cpp`:

```cpp
#include <gtest/gtest.h>

#include <geolib/math_types.h>

// These are characterization tests for a behaviour-preserving refactor:
// they pin the public access surface (named members, operator[], operator())
// and must pass both before and after the unions are removed.

TEST(MathTypes, Vec2IndexAliasesNamedMembers)
{
    geo::Vec2T<double> v(1.0, 2.0);
    EXPECT_EQ(v[0], v.x);
    EXPECT_EQ(v[1], v.y);
    EXPECT_DOUBLE_EQ(v[0], 1.0);
    EXPECT_DOUBLE_EQ(v[1], 2.0);

    v[0] = 7.0;
    EXPECT_DOUBLE_EQ(v.x, 7.0);
}

TEST(MathTypes, Vec2PointerConstructor)
{
    const double values[2] = {3.0, 4.0};
    geo::Vec2T<double> v(values);
    EXPECT_DOUBLE_EQ(v.x, 3.0);
    EXPECT_DOUBLE_EQ(v.y, 4.0);
}

TEST(MathTypes, Vec3IndexAliasesNamedMembers)
{
    geo::Vec3T<double> v(1.0, 2.0, 3.0);
    EXPECT_EQ(v[0], v.x);
    EXPECT_EQ(v[1], v.y);
    EXPECT_EQ(v[2], v.z);
    EXPECT_DOUBLE_EQ(v[2], 3.0);

    v[2] = 9.0;
    EXPECT_DOUBLE_EQ(v.z, 9.0);
}

TEST(MathTypes, Vec3PointerConstructor)
{
    const double values[3] = {3.0, 4.0, 5.0};
    geo::Vec3T<double> v(values);
    EXPECT_DOUBLE_EQ(v.x, 3.0);
    EXPECT_DOUBLE_EQ(v.y, 4.0);
    EXPECT_DOUBLE_EQ(v.z, 5.0);
}

TEST(MathTypes, Mat2IndexAndCallAliasMembers)
{
    geo::Mat2T<double> m(1.0, 2.0, 3.0, 4.0); // xx, xy, yx, yy
    EXPECT_EQ(m[0], m.xx);
    EXPECT_EQ(m[1], m.xy);
    EXPECT_EQ(m[2], m.yx);
    EXPECT_EQ(m[3], m.yy);
    // operator() row-major: (i,j) -> i*2+j
    EXPECT_EQ(m(0, 0), m.xx);
    EXPECT_EQ(m(0, 1), m.xy);
    EXPECT_EQ(m(1, 0), m.yx);
    EXPECT_EQ(m(1, 1), m.yy);
}

TEST(MathTypes, QuaternionIndexAliasesNamedMembers)
{
    geo::QuaternionT<double> q(1.0, 2.0, 3.0, 4.0); // x, y, z, w
    EXPECT_EQ(q[0], q.x);
    EXPECT_EQ(q[1], q.y);
    EXPECT_EQ(q[2], q.z);
    EXPECT_EQ(q[3], q.w);

    q[3] = 8.0;
    EXPECT_DOUBLE_EQ(q.w, 8.0);
}

TEST(MathTypes, Mat3IndexAndCallAliasMembers)
{
    geo::Mat3T<double> m(1, 2, 3, 4, 5, 6, 7, 8, 9);
    EXPECT_EQ(m[0], m.xx);
    EXPECT_EQ(m[1], m.xy);
    EXPECT_EQ(m[2], m.xz);
    EXPECT_EQ(m[3], m.yx);
    EXPECT_EQ(m[4], m.yy);
    EXPECT_EQ(m[5], m.yz);
    EXPECT_EQ(m[6], m.zx);
    EXPECT_EQ(m[7], m.zy);
    EXPECT_EQ(m[8], m.zz);
    // operator() row-major: (i,j) -> i*3+j
    EXPECT_EQ(m(0, 0), m.xx);
    EXPECT_EQ(m(1, 2), m.yz);
    EXPECT_EQ(m(2, 1), m.zy);
}

TEST(MathTypes, Mat3PointerConstructor)
{
    const double values[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    geo::Mat3T<double> m(values);
    EXPECT_DOUBLE_EQ(m.xx, 1.0);
    EXPECT_DOUBLE_EQ(m.yy, 5.0);
    EXPECT_DOUBLE_EQ(m.zz, 9.0);
}

TEST(MathTypes, Mat3GetRowGetColumn)
{
    geo::Mat3T<double> m(1, 2, 3, 4, 5, 6, 7, 8, 9);
    geo::Vec3T<double> row0 = m.getRow(0);
    EXPECT_DOUBLE_EQ(row0.x, 1.0);
    EXPECT_DOUBLE_EQ(row0.y, 2.0);
    EXPECT_DOUBLE_EQ(row0.z, 3.0);
    geo::Vec3T<double> col0 = m.getColumn(0);
    EXPECT_DOUBLE_EQ(col0.x, 1.0);
    EXPECT_DOUBLE_EQ(col0.y, 4.0);
    EXPECT_DOUBLE_EQ(col0.z, 7.0);
}

TEST(MathTypes, Mat3RotationRoundTripsThroughQuaternion)
{
    // Exercises setRotation + getRotation, which use indexed access internally.
    geo::Mat3T<double> m = geo::Mat3T<double>::identity();
    geo::QuaternionT<double> q;
    m.getRotation(q);
    EXPECT_DOUBLE_EQ(q.w, 1.0);
    EXPECT_DOUBLE_EQ(q.x, 0.0);
    EXPECT_DOUBLE_EQ(q.y, 0.0);
    EXPECT_DOUBLE_EQ(q.z, 0.0);

    geo::Mat3T<double> m2;
    m2.setRotation(q);
    EXPECT_NEAR(m2.xx, 1.0, 1e-12);
    EXPECT_NEAR(m2.yy, 1.0, 1e-12);
    EXPECT_NEAR(m2.zz, 1.0, 1e-12);
    EXPECT_NEAR(m2.xy, 0.0, 1e-12);
}
```

- [ ] **Step 2: Run the tests to confirm they pass on the current (union) code**

Run:
```bash
g++ -std=c++17 -I include test/test_math_types.cpp -lgtest -lgtest_main -pthread \
    -o /tmp/test_math_types && /tmp/test_math_types
```
Expected: PASS — `[  PASSED  ] 11 tests.` (They characterize existing behaviour; they must already pass.)

- [ ] **Step 3: Register the test in CMakeLists.txt**

In `CMakeLists.txt`, immediately after the `test_lrf` block (currently ending at line 218 with `target_link_libraries(test_lrf geolib)`), and before the closing `endif()`, add:

```cmake
  ament_add_gtest(test_math_types test/test_math_types.cpp)
  target_link_libraries(test_math_types geolib)
```

- [ ] **Step 4: Commit**

```bash
git add test/test_math_types.cpp CMakeLists.txt
git commit -m "test: add characterization tests for math_types access surface

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 2: Remove union from Vec2T and Vec3T

**Files:**
- Modify: `include/geolib/math_types.h` — `Vec2T` (lines ~21-154) and `Vec3T` (lines ~158-303)

**Interfaces:**
- Consumes: characterization tests from Task 1 (`MathTypes.Vec2*`, `MathTypes.Vec3*`).
- Produces: `Vec2T`/`Vec3T` with `T x, y[, z];` as plain members, a `switch`-based `operator[]`, and element-wise `(const T*)` constructors. No `m` member, no union.

- [ ] **Step 1: Confirm the relevant tests currently pass**

Run:
```bash
g++ -std=c++17 -I include test/test_math_types.cpp -lgtest -lgtest_main -pthread \
    -o /tmp/test_math_types && /tmp/test_math_types --gtest_filter='MathTypes.Vec*'
```
Expected: PASS (4 tests).

- [ ] **Step 2: Vec2T — replace the `(const T*)` constructor**

In `Vec2T`, replace:
```cpp
    Vec2T(const T* values) { memcpy(m, values, 2 * sizeof(T)); }
```
with:
```cpp
    Vec2T(const T* values) : x(values[0]), y(values[1]) {}
```

- [ ] **Step 3: Vec2T — replace `operator[]` overloads**

Replace:
```cpp
    T& operator[](const uint i) { return m[i]; }

    const T& operator[](const uint i) const { return m[i]; }
```
with:
```cpp
    T& operator[](const uint i)
    {
        switch (i)
        {
        case 0:
            return x;
        default:
            return y;
        }
    }

    const T& operator[](const uint i) const
    {
        switch (i)
        {
        case 0:
            return x;
        default:
            return y;
        }
    }
```

- [ ] **Step 4: Vec2T — replace the union with plain members**

Replace:
```cpp
    union
    {
        struct
        {
            T x, y;
        };
        T m[2];
    };
```
with:
```cpp
    T x, y;
```

- [ ] **Step 5: Vec3T — replace the `(const T*)` constructor**

In `Vec3T`, replace:
```cpp
    Vec3T(const T* values) { memcpy(m, values, 3 * sizeof(T)); }
```
with:
```cpp
    Vec3T(const T* values) : x(values[0]), y(values[1]), z(values[2]) {}
```

- [ ] **Step 6: Vec3T — replace `operator[]` overloads**

Replace:
```cpp
    T& operator[](const uint i) { return m[i]; }

    const T& operator[](const uint i) const { return m[i]; }
```
with:
```cpp
    T& operator[](const uint i)
    {
        switch (i)
        {
        case 0:
            return x;
        case 1:
            return y;
        default:
            return z;
        }
    }

    const T& operator[](const uint i) const
    {
        switch (i)
        {
        case 0:
            return x;
        case 1:
            return y;
        default:
            return z;
        }
    }
```

- [ ] **Step 7: Vec3T — replace the union with plain members**

Replace:
```cpp
    union
    {
        struct
        {
            T x, y, z;
        };
        T m[3];
    };
```
with:
```cpp
    T x, y, z;
```

- [ ] **Step 8: Run the tests to confirm they still pass**

Run:
```bash
g++ -std=c++17 -I include test/test_math_types.cpp -lgtest -lgtest_main -pthread \
    -o /tmp/test_math_types && /tmp/test_math_types --gtest_filter='MathTypes.Vec*'
```
Expected: PASS (4 tests).

- [ ] **Step 9: Commit**

```bash
git add include/geolib/math_types.h
git commit -m "refactor: remove type-punning union from Vec2T and Vec3T

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 3: Remove union from Mat2T

**Files:**
- Modify: `include/geolib/math_types.h` — `Mat2T` (lines ~307-390)

**Interfaces:**
- Consumes: characterization test `MathTypes.Mat2IndexAndCallAliasMembers`.
- Produces: `Mat2T` with `T xx, xy, yx, yy;` as plain members, `switch`-based `operator[]`, and `operator()` routed through `operator[]`. No `m` member, no union.

- [ ] **Step 1: Confirm the relevant test currently passes**

Run:
```bash
g++ -std=c++17 -I include test/test_math_types.cpp -lgtest -lgtest_main -pthread \
    -o /tmp/test_math_types && /tmp/test_math_types --gtest_filter='MathTypes.Mat2*'
```
Expected: PASS (1 test).

- [ ] **Step 2: Replace `operator[]` overloads**

Replace:
```cpp
    T& operator[](const uint i) { return m[i]; }

    const T& operator[](const uint i) const { return m[i]; }
```
with:
```cpp
    T& operator[](const uint i)
    {
        switch (i)
        {
        case 0:
            return xx;
        case 1:
            return xy;
        case 2:
            return yx;
        default:
            return yy;
        }
    }

    const T& operator[](const uint i) const
    {
        switch (i)
        {
        case 0:
            return xx;
        case 1:
            return xy;
        case 2:
            return yx;
        default:
            return yy;
        }
    }
```

- [ ] **Step 3: Route `operator()` through `operator[]`**

Replace:
```cpp
    T& operator()(int i, int j) { return m[i * 2 + j]; }

    const T& operator()(int i, int j) const { return m[i * 2 + j]; }
```
with:
```cpp
    T& operator()(int i, int j) { return (*this)[i * 2 + j]; }

    const T& operator()(int i, int j) const { return (*this)[i * 2 + j]; }
```

- [ ] **Step 4: Replace the union with plain members**

Replace:
```cpp
    union
    {
        struct
        {
            T xx, xy, yx, yy;
        };
        T m[4];
    };
```
with:
```cpp
    T xx, xy, yx, yy;
```

Note: `operator<<` for `Mat2T` uses `m[0]…m[3]`, but `m` there is the **function parameter** (`const Mat2T& m`), so it calls `m.operator[]` — leave it unchanged.

- [ ] **Step 5: Run the tests to confirm they still pass**

Run:
```bash
g++ -std=c++17 -I include test/test_math_types.cpp -lgtest -lgtest_main -pthread \
    -o /tmp/test_math_types && /tmp/test_math_types --gtest_filter='MathTypes.Mat2*'
```
Expected: PASS (1 test).

- [ ] **Step 6: Commit**

```bash
git add include/geolib/math_types.h
git commit -m "refactor: remove type-punning union from Mat2T

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 4: Remove union from Mat3T

**Files:**
- Modify: `include/geolib/math_types.h` — `Mat3T` (lines ~483-710)

**Interfaces:**
- Consumes: characterization tests `MathTypes.Mat3*`.
- Produces: `Mat3T` with `T xx … zz;` as plain members, `switch`-based `operator[]`, `operator()` and all internal indexed access (`getRow`, `getColumn`, `setRPY`, `getRotation`, `setRotation`) routed through `operator[]`/`q[]`, element-wise `(const T*)` constructor. No `m` member, no union. After this task **no code references `q.m`** anywhere in the file (unblocks Task 5).

- [ ] **Step 1: Confirm the relevant tests currently pass**

Run:
```bash
g++ -std=c++17 -I include test/test_math_types.cpp -lgtest -lgtest_main -pthread \
    -o /tmp/test_math_types && /tmp/test_math_types --gtest_filter='MathTypes.Mat3*'
```
Expected: PASS (4 tests).

- [ ] **Step 2: Replace the `(const T*)` constructor**

Replace:
```cpp
    Mat3T(const T* values) { memcpy(m, values, 9 * sizeof(T)); }
```
with:
```cpp
    Mat3T(const T* values) :
        xx(values[0]), xy(values[1]), xz(values[2]), yx(values[3]), yy(values[4]), yz(values[5]), zx(values[6]),
        zy(values[7]), zz(values[8])
    {
    }
```

- [ ] **Step 3: Replace `operator[]` overloads**

Replace:
```cpp
    T& operator[](const uint i) { return m[i]; }

    const T& operator[](const uint i) const { return m[i]; }
```
with:
```cpp
    T& operator[](const uint i)
    {
        switch (i)
        {
        case 0:
            return xx;
        case 1:
            return xy;
        case 2:
            return xz;
        case 3:
            return yx;
        case 4:
            return yy;
        case 5:
            return yz;
        case 6:
            return zx;
        case 7:
            return zy;
        default:
            return zz;
        }
    }

    const T& operator[](const uint i) const
    {
        switch (i)
        {
        case 0:
            return xx;
        case 1:
            return xy;
        case 2:
            return xz;
        case 3:
            return yx;
        case 4:
            return yy;
        case 5:
            return yz;
        case 6:
            return zx;
        case 7:
            return zy;
        default:
            return zz;
        }
    }
```

- [ ] **Step 4: Route `operator()` through `operator[]`**

Replace:
```cpp
    T& operator()(int i, int j) { return m[i * 3 + j]; }

    const T& operator()(int i, int j) const { return m[i * 3 + j]; }
```
with:
```cpp
    T& operator()(int i, int j) { return (*this)[i * 3 + j]; }

    const T& operator()(int i, int j) const { return (*this)[i * 3 + j]; }
```

- [ ] **Step 5: Route `getRow` / `getColumn` through `operator[]`**

Replace:
```cpp
    Vec3T<T> getRow(int i) const { return Vec3T<T>(m[i * 3], m[i * 3 + 1], m[i * 3 + 2]); }

    Vec3T<T> getColumn(int i) const { return Vec3T<T>(m[i], m[3 + i], m[6 + i]); }
```
with:
```cpp
    Vec3T<T> getRow(int i) const { return Vec3T<T>((*this)[i * 3], (*this)[i * 3 + 1], (*this)[i * 3 + 2]); }

    Vec3T<T> getColumn(int i) const { return Vec3T<T>((*this)[i], (*this)[3 + i], (*this)[6 + i]); }
```

- [ ] **Step 6: Route `setRPY` through `operator[]`**

In `setRPY`, replace the assignment block:
```cpp
        m[0] = cj * ch;
        m[1] = sj * sc - cs;
        m[2] = sj * cc + ss;
        m[3] = cj * sh;
        m[4] = sj * ss + cc, m[5] = sj * cs - sc;
        m[6] = -sj;
        m[7] = cj * si;
        m[8] = cj * ci;
```
with:
```cpp
        (*this)[0] = cj * ch;
        (*this)[1] = sj * sc - cs;
        (*this)[2] = sj * cc + ss;
        (*this)[3] = cj * sh;
        (*this)[4] = sj * ss + cc;
        (*this)[5] = sj * cs - sc;
        (*this)[6] = -sj;
        (*this)[7] = cj * si;
        (*this)[8] = cj * ci;
```
(Note: the original line 606 used a comma operator joining `m[4]` and `m[5]`; this is split into two proper statements.)

- [ ] **Step 7: Route `getRotation` through `operator[]` and `q[]`**

In `getRotation`, replace:
```cpp
        if (trace > 0)
        {
            T s = sqrt(trace + 1);
            q.m[3] = (s / 2);
            s = 0.5 / s;

            q.m[0] = ((zy - yz) * s);
            q.m[1] = ((xz - zx) * s);
            q.m[2] = ((yx - xy) * s);
        }
        else
        {
            int i = xx < yy ? (yy < zz ? 2 : 1) : (xx < zz ? 2 : 0);
            int j = (i + 1) % 3;
            int k = (i + 2) % 3;

            T s = sqrt(m[i * 4] - m[j * 4] - m[k * 4] + 1);
            q.m[i] = s / 2;
            s = 0.5 / s;

            q.m[3] = (m[k * 3 + j] - m[j * 3 + k]) * s;
            q.m[j] = (m[j * 3 + i] + m[i * 3 + j]) * s;
            q.m[k] = (m[k * 3 + i] + m[i * 3 + k]) * s;
        }
```
with:
```cpp
        if (trace > 0)
        {
            T s = sqrt(trace + 1);
            q[3] = (s / 2);
            s = 0.5 / s;

            q[0] = ((zy - yz) * s);
            q[1] = ((xz - zx) * s);
            q[2] = ((yx - xy) * s);
        }
        else
        {
            int i = xx < yy ? (yy < zz ? 2 : 1) : (xx < zz ? 2 : 0);
            int j = (i + 1) % 3;
            int k = (i + 2) % 3;

            T s = sqrt((*this)[i * 4] - (*this)[j * 4] - (*this)[k * 4] + 1);
            q[i] = s / 2;
            s = 0.5 / s;

            q[3] = ((*this)[k * 3 + j] - (*this)[j * 3 + k]) * s;
            q[j] = ((*this)[j * 3 + i] + (*this)[i * 3 + j]) * s;
            q[k] = ((*this)[k * 3 + i] + (*this)[i * 3 + k]) * s;
        }
```

- [ ] **Step 8: Route `setRotation` through `operator[]`**

In `setRotation`, replace the assignment block:
```cpp
        m[0] = 1 - (yy + zz);
        m[1] = xy - wz;
        m[2] = xz + wy;
        m[3] = xy + wz;
        m[4] = 1 - (xx + zz);
        m[5] = yz - wx;
        m[6] = xz - wy;
        m[7] = yz + wx;
        m[8] = 1 - (xx + yy);
```
with:
```cpp
        (*this)[0] = 1 - (yy + zz);
        (*this)[1] = xy - wz;
        (*this)[2] = xz + wy;
        (*this)[3] = xy + wz;
        (*this)[4] = 1 - (xx + zz);
        (*this)[5] = yz - wx;
        (*this)[6] = xz - wy;
        (*this)[7] = yz + wx;
        (*this)[8] = 1 - (xx + yy);
```
(Note: inside `setRotation`, `xx`, `xy`, … are **local variables** that shadow nothing—the matrix members are written via `(*this)[...]`. This preserves the original behaviour where the right-hand side used the locals and the left-hand side wrote the array.)

- [ ] **Step 9: Replace the union with plain members**

Replace:
```cpp
    union
    {
        struct
        {
            T xx, xy, xz, yx, yy, yz, zx, zy, zz;
        };
        T m[9];
    };
```
with:
```cpp
    T xx, xy, xz, yx, yy, yz, zx, zy, zz;
```

Note: `operator<<` for `Mat3T` uses `m[0]…m[8]` where `m` is the **function parameter** (`const Mat3T& m`) → `m.operator[]`. Leave it unchanged.

- [ ] **Step 10: Run the tests to confirm they still pass**

Run:
```bash
g++ -std=c++17 -I include test/test_math_types.cpp -lgtest -lgtest_main -pthread \
    -o /tmp/test_math_types && /tmp/test_math_types --gtest_filter='MathTypes.Mat3*'
```
Expected: PASS (4 tests).

- [ ] **Step 11: Verify no `q.m` references remain in the file**

Run:
```bash
grep -n "\.m\[" include/geolib/math_types.h
```
Expected: no output (every `.m[` access is gone; only `operator<<`'s parameter-based `m[...]` remains, which is not `.m[`).

- [ ] **Step 12: Commit**

```bash
git add include/geolib/math_types.h
git commit -m "refactor: remove type-punning union from Mat3T

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 5: Remove union from QuaternionT

**Files:**
- Modify: `include/geolib/math_types.h` — `QuaternionT` (lines ~394-479)

**Interfaces:**
- Consumes: characterization test `MathTypes.QuaternionIndexAliasesNamedMembers`. Requires Task 4 done first (so nothing references `q.m`).
- Produces: `QuaternionT` with `T x, y, z, w;` as plain members and a `switch`-based `operator[]`. No `m` member, no union.

- [ ] **Step 1: Confirm the relevant test currently passes**

Run:
```bash
g++ -std=c++17 -I include test/test_math_types.cpp -lgtest -lgtest_main -pthread \
    -o /tmp/test_math_types && /tmp/test_math_types --gtest_filter='MathTypes.Quaternion*'
```
Expected: PASS (1 test).

- [ ] **Step 2: Replace `operator[]` overloads**

Replace:
```cpp
    T& operator[](const uint i) { return m[i]; }

    const T& operator[](const uint i) const { return m[i]; }
```
with:
```cpp
    T& operator[](const uint i)
    {
        switch (i)
        {
        case 0:
            return x;
        case 1:
            return y;
        case 2:
            return z;
        default:
            return w;
        }
    }

    const T& operator[](const uint i) const
    {
        switch (i)
        {
        case 0:
            return x;
        case 1:
            return y;
        case 2:
            return z;
        default:
            return w;
        }
    }
```

- [ ] **Step 3: Replace the union with plain members**

Replace:
```cpp
    union
    {
        struct
        {
            T x, y, z, w;
        };
        T m[4];
    };
```
with:
```cpp
    T x, y, z, w;
```

- [ ] **Step 4: Run the full test binary to confirm everything passes**

Run:
```bash
g++ -std=c++17 -I include test/test_math_types.cpp -lgtest -lgtest_main -pthread \
    -o /tmp/test_math_types && /tmp/test_math_types
```
Expected: PASS — `[  PASSED  ] 11 tests.`

- [ ] **Step 5: Verify no `union` keyword remains in the file**

Run:
```bash
grep -n "union" include/geolib/math_types.h
```
Expected: no output.

- [ ] **Step 6: Commit**

```bash
git add include/geolib/math_types.h
git commit -m "refactor: remove type-punning union from QuaternionT

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

### Task 6: Full integration build, lint, and clang-format verification

**Files:** none modified (verification only; apply clang-format fixes if needed).

**Interfaces:**
- Consumes: all prior tasks.
- Produces: confirmation that the package builds, all gtests and the legacy executable tests pass, clang-format is clean, and `cppcoreguidelines-pro-type-union-access` no longer fires.

- [ ] **Step 1: Check clang-format compliance**

Run (matches the project's pinned version):
```bash
clang-format-21 --dry-run --Werror \
    --style=file:$(find / -path '*tue_lint_config*/config/.clang-format' 2>/dev/null | head -1) \
    include/geolib/math_types.h test/test_math_types.cpp
```
Expected: no output (clean). If it reports diffs, apply them:
```bash
clang-format-21 -i \
    --style=file:$(find / -path '*tue_lint_config*/config/.clang-format' 2>/dev/null | head -1) \
    include/geolib/math_types.h test/test_math_types.cpp
```
then re-run the dry-run and, if anything changed, `git add` + `git commit -m "style: clang-format math_types changes"`.

- [ ] **Step 2: Build the package with tests enabled (runs clang-tidy)**

From the colcon workspace root (the directory whose `src/` contains these repos):
```bash
colcon build --packages-select geolib2 --cmake-args -DBUILD_TESTING=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```
Expected: build succeeds. Inspect the clang-tidy output for `math_types.h`:
```bash
colcon build --packages-select geolib2 --cmake-args -DBUILD_TESTING=ON 2>&1 | grep -i "union-access" || echo "NO union-access warnings"
```
Expected: `NO union-access warnings`.

> If `colcon` is unavailable in this environment, document that the integration build was not run and report the standalone results from Step 3 + the per-task gtest passes. Do not claim the colcon build passed if it was not executed.

- [ ] **Step 3: Run the full test suite**

```bash
colcon test --packages-select geolib2 && colcon test-result --all --verbose
```
Expected: all tests pass, including `test_math_types`, `test_geolib`, `test_geolib_lrf`, `test_matrix`, `test_box`, `test_composite_shape`, `test_shape`, `test_lrf`.

Fallback if colcon is unavailable — run the standalone unit tests:
```bash
g++ -std=c++17 -I include test/test_math_types.cpp -lgtest -lgtest_main -pthread \
    -o /tmp/test_math_types && /tmp/test_math_types
```
Expected: `[  PASSED  ] 11 tests.`

- [ ] **Step 4: Final review of the diff**

```bash
git diff master --stat
git log --oneline master..HEAD
```
Confirm only `include/geolib/math_types.h`, `test/test_math_types.cpp`, `CMakeLists.txt`, and the docs are touched, and the commit history is clean. No further commit needed unless clang-format changed files in Step 1.
```

## Self-Review

**Spec coverage:**
- Remove unions from all 5 types → Tasks 2 (Vec2/Vec3), 3 (Mat2), 4 (Mat3), 5 (Quaternion). ✓
- `operator[]` switch-based → each task. ✓
- `operator()` rerouted → Tasks 3, 4. ✓
- `memcpy` constructors → element-wise → Tasks 2 (Vec2/Vec3), 4 (Mat3). ✓
- Mat3T internal `m[k]` + `q.m[k]` rerouted → Task 4 (steps 5-8). ✓
- `operator<<` parameter-based access unchanged → noted in Tasks 3, 4. ✓
- Verify warning cleared → Task 6. ✓
- Out-of-scope `matrix.h` untouched → Global Constraints. ✓

**Placeholder scan:** No TBD/TODO; every code step shows full code. ✓

**Type consistency:** `operator[]`/`operator()`/`(*this)[...]`/`q[...]` used consistently; member name lists match the spec table across all tasks. ✓

**Ordering:** Task 4 (Mat3T) eliminates the only `q.m` reference before Task 5 removes Quaternion's `m`, so every commit compiles. ✓
