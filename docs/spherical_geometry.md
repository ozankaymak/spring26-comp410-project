# Spherical Geometry Notes

The renderer ships with an experimental spherical mode that mirrors the
hyperbolic pipeline. It is implemented as a separate module
(`src/math/spherical.h` / `src/math/spherical.cpp`, namespace
`hyper::math::sphere`) that reuses the plain data types from the hyperbolic
layer but swaps the underlying metric. The original hyperbolic API is left
untouched; the tiling and rendering layers dispatch between the two with a
`hyper::math::GeometryMode` flag.

## Model

Points use the embedded unit 2-sphere with the Euclidean signature `(+, +, +)`:

```text
S^2 = { (t, x, y) in R^3 : t^2 + x^2 + y^2 = 1 }
```

The Euclidean inner product is the spherical counterpart of the Minkowski dot:

```text
<a, b> = a.t * b.t + a.x * b.x + a.y * b.y
```

The "origin" is `(1, 0, 0)`. The first coordinate `t` plays the role the
timelike coordinate plays on the hyperboloid, which keeps the embedding,
projections, and camera frames structurally identical to the hyperbolic path.

The distance between two valid points is the angle between them:

```text
distance(a, b) = arccos(<a, b>)
```

Tangent vectors at `p` are Euclidean-orthogonal to `p`, since on the sphere the
surface normal is the position vector itself:

```text
<p, v> = 0
```

## Motion

A local movement vector `(dx, dy)` is treated as a tangent displacement in the
current frame. Its angular length is `d = sqrt(dx^2 + dy^2)` and its unit
direction is `(nx, ny) = (dx, dy) / d`. The local isometry is a rotation in the
plane spanned by the time axis and the tangent direction. It is the SO(3)
counterpart of the Lorentz boost, obtained by replacing `cosh/sinh` with
`cos/sin` and giving the time/space coupling the antisymmetric sign of a
rotation:

```text
[  cos d       -sin d nx              -sin d ny            ]
[  sin d nx     1 + (cos d - 1) nx^2   (cos d - 1) nx ny   ]
[  sin d ny     (cos d - 1) nx ny      1 + (cos d - 1) ny^2 ]
```

Camera frames are stored and kept orthonormal exactly as in the hyperbolic
case, but under the Euclidean product. For a rotation matrix the inverse is the
transpose.

## Regular `{p, q}` Tilings

A regular `{p, q}` tiling closes up on the sphere when:

```text
1 / p + 1 / q > 1 / 2      <=>      (p - 2)(q - 2) < 4
```

These are the spherical (Platonic) tilings:

```text
{3,3} tetrahedron     {4,3} cube         {3,4} octahedron
{5,3} dodecahedron    {3,5} icosahedron
```

The fundamental right triangle uses the same relations as the hyperbolic case,
with the spherical law of cosines (`arccos`) replacing the hyperbolic one
(`arcosh`):

```text
half_edge = arccos(cos(pi / p) / sin(pi / q))
edge      = 2 * half_edge
inradius  = arccos(cos(pi / q) / sin(pi / p))
R         = arccos(cot(pi / p) * cot(pi / q))
```

For the `{4,3}` cube these evaluate to:

```text
edge          = 1.2309594173407747
inradius      = 0.7853981633974483   (pi / 4)
circumradius  = 0.9553166181245093
```

Side reflections use a Euclidean unit normal at the tile-side pole,
`(sin r, cos r cos a, cos r sin a)` with `r = inradius`, and the reflection is
`I - 2 n n^T`. Composing these reflections generates the finite patch (six tiles
for the cube, eight for the octahedron, and so on), and tile centres stay on the
sphere.

## Projection And Rendering

The math layer exposes two disk projections that share their forward formula
with the hyperbolic models:

- Stereographic disk (Poincare analogue): `(x, y) / (t + 1)`
- Gnomonic disk (Klein analogue): `(x, y) / t`

Crossing detection projects to the gnomonic disk, where geodesic polygon edges
become straight segments, just as the hyperbolic path uses the Klein disk.

The perspective renderer embeds `S^2` into the unit 3-sphere and draws it in two
clipped passes, because stereographic projection diverges at the antipode. The
near hemisphere (`w >= 0`) is projected from one pole and the far hemisphere
(`w < 0`) from the other; `gl_ClipDistance` discards the wrong hemisphere and a
split depth range keeps the near half in front. Distance to the camera is
`arccos(w)`.
