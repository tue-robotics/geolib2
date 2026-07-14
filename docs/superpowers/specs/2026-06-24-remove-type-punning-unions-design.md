# Remove type-punning unions from `math_types.h`

**Date:** 2026-06-24
**Status:** Approved design

## Problem

Each math type in `include/geolib/math_types.h` (`Vec2T`, `Vec3T`, `Mat2T`,
`QuaternionT`, `Mat3T`) stores its data in an **anonymous union** that
type-puns between named members and a flat array:

```cpp
union {
    struct { T x, y, z; };  // named access:  v.x, v.y, v.z
    T m[3];                 // array access:  v.m[i], v[i], memcpy(m, ...)
};
```

This pattern has two problems:

1. The anonymous `struct` inside a `union` is a non-standard compiler
   extension, and reading a union member other than the one last written is
   technically undefined behaviour.
2. The clang-tidy check `cppcoreguidelines-pro-type-union-access` fires at
   **every access to a union member** — so every `vec.x`, `mat.xx`, and
   `q.m[i]` in *dependent* packages trips the warning, because those members
   live inside a union.

## Goal

Remove the unions so the warning disappears at all access sites — including in
dependent packages — **without changing dependent source code**. The public
member names (`x`, `y`, `z`, `xx` … `zz`, `w`) and the `operator[]` /
`operator()` indexed access must continue to work unchanged.

`std::variant` is explicitly **not** used: it holds only one alternative at a
time and cannot model simultaneous named + array access. The task title
("convert unions to variants") resolves to "eliminate the unions," not a
literal `std::variant` substitution.

## Approach

Make the **named members the single source of truth**. Drop the `m[N]` array
member and the anonymous struct/union entirely. Re-route all indexed access
through the existing `operator[]`, reimplemented as a `switch` — fully
standard, no UB, and no `reinterpret_cast` (which would only trade this lint
check for `cppcoreguidelines-pro-type-reinterpret-cast`).

### Rejected alternative

Keep a `std::array<T,N> m` as storage and turn `x`/`y`/`z` into accessor
methods. Rejected: named access (`.x`, `.xx`) vastly dominates the API, so this
would force large, breaking churn across every dependent package.

## Concrete changes (all in `include/geolib/math_types.h`)

| Type          | New storage members                 | Indexed access                                            |
|---------------|-------------------------------------|-----------------------------------------------------------|
| `Vec2T`       | `T x, y;`                            | `operator[]` → `switch` (2 cases)                         |
| `Vec3T`       | `T x, y, z;`                         | `switch` (3)                                              |
| `Mat2T`       | `T xx, xy, yx, yy;`                  | `switch` (4); `operator()` → `(*this)[i*2+j]`             |
| `QuaternionT` | `T x, y, z, w;`                      | `switch` (4)                                              |
| `Mat3T`       | `T xx, xy, xz, yx, yy, yz, zx, zy, zz;` | `switch` (9); `operator()` → `(*this)[i*3+j]`; reroute internal `m[k]`→`(*this)[k]` and `q.m[k]`→`q[k]` |

### `operator[]` shape

```cpp
T& operator[](const uint i)
{
    switch (i)
    {
    case 0: return x;
    case 1: return y;
    default: return z;   // i == 2 (and, as before, out-of-range maps here)
    }
}
```

Both the mutable and `const` overloads change identically. Out-of-range
indices were already undefined behaviour with the old `m[i]`; the `default`
branch now maps them to the last member, which also satisfies the
"return on all paths" requirement.

### Constructors using `memcpy`

`Vec2T`, `Vec3T`, and `Mat3T` each have a `(const T* values)` constructor that
does `memcpy(m, values, N*sizeof(T))`. These become element-wise assignment:

```cpp
Vec3T(const T* values) : x(values[0]), y(values[1]), z(values[2]) {}
```

### Internal `m[k]` rewrites in `Mat3T`

The following member functions index the array member directly and must route
through `operator[]` (`(*this)[k]`), and the quaternion argument through
`q[k]`:

- `getRow`, `getColumn`
- `setRPY`
- `getRotation` (own `m[...]` and the `q.m[...]` writes)
- `setRotation`

`operator<<` for `Mat2T`/`Mat3T` indexes its **parameter** (`m[0]` =
`m.operator[](0)`, where `m` is the function argument), so it needs no change.

## Out of scope

- `include/geolib/matrix.h` (`Vector3::v_`, `Matrix3x3::m_`) is a separate,
  non-union class. Its `memcpy` uses are unrelated and unchanged.
- The word "union" in `CompositeShape.h` / `HeightMapNode.h` appears only in
  prose comments ("union of geometry"); no code change there.

## Compatibility / impact

- **Object size unchanged** — the union's size equalled the named-member
  struct's size; named members are still contiguous standard-layout, so code
  passing `&vec.x` as a buffer is unaffected.
- **Public API preserved** — `.x`/`.xx`/…, `operator[]`, and `operator()` all
  behave identically. Only the `m` array *member* is removed.
- **Dependent breakage:** direct `obj.m[i]` access would no longer compile.
  Verified `grep` across all sibling repos in the workspace
  (`ed`, `rgbd`, `code_profiler`, `ed_msgs`, `tue_config`, `tue_filesystem`,
  `tue_serialization`, `upower_ros`): **zero** `.m[` or bare `.m` uses. Nothing
  to migrate in-workspace.

## Verification

- Build geolib2 and run the existing test executables (`test_geolib`,
  `test_geolib_lrf`, `test_matrix`, plus the shape/box/lrf/composite tests) —
  they exercise these types indirectly and must continue to pass.
- Add focused unit assertions confirming, for each type, that `operator[]`
  returns the same storage as the named members (e.g. `v[0]` aliases `v.x`)
  and that the `(const T*)` constructors populate correctly.
- Confirm a clang-tidy run no longer reports
  `cppcoreguidelines-pro-type-union-access` for these types.
