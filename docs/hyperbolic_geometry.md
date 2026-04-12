# Hyperbolic Geometry Notes

The geometry layer uses the hyperbolic plane with curvature `K = -1`. Points
are stored in the hyperboloid model:

```text
H^2 = { (t, x, y) in R^(2,1) : -t^2 + x^2 + y^2 = -1, t > 0 }
```

The Minkowski inner product uses signature `(-, +, +)`:

```text
<a, b>_M = -a.t * b.t + a.x * b.x + a.y * b.y
```

Valid positions stay on the future sheet:

```text
<p, p>_M = -1
p.t > 0
```

The distance between two valid hyperboloid points is:

```text
distance(a, b) = arcosh(-<a, b>_M)
```

Tangent vectors at a point `p` are orthogonal to that point under the
Minkowski product:

```text
<p, v>_M = 0
<v, v>_M > 0
```

Camera movement will be represented with Lorentz isometries. Those transforms
must preserve the Minkowski product and keep positions on the future sheet.

## Regular `{4,6}` Tiling

The first tiling target is `{p, q} = {4, 6}`:

- `p = 4`: each tile has four sides
- `q = 6`: six tiles meet at each vertex
- generated patches are finite BFS neighborhoods of a root tile
- duplicate detection should compare hyperbolic positions with a tolerance

A regular `{p, q}` tiling is hyperbolic when:

```text
1 / p + 1 / q < 1 / 2
```

Equivalently:

```text
(p - 2)(q - 2) > 4
```

For `{4,6}`, `(p - 2)(q - 2) = 8`, so the tiling is hyperbolic.

## Fundamental Triangle

A regular `{p, q}` tile splits into right triangles with angles:

```text
A = pi / p      at the tile center
B = pi / q      at a tile vertex
C = pi / 2      at an edge midpoint
```

The side opposite `C` is the tile circumradius `R`, from tile center to vertex.
The side opposite `B` is the inradius `r`, from tile center to edge midpoint.
The side opposite `A` is half the tile edge length.

For curvature `K = -1`:

```text
half_edge = arcosh(cos(pi / p) / sin(pi / q))
edge      = 2 * half_edge
inradius  = arcosh(cos(pi / q) / sin(pi / p))
R         = arcosh(cot(pi / p) * cot(pi / q))
```

For `{4,6}` these evaluate approximately to:

```text
edge          = 1.762747174039086
inradius      = 0.6584789484624083
circumradius  = 1.1462158347805889
```

## Projection Helpers

The math layer exposes projection helpers without forcing a renderer choice:

- Poincare disk: `(x, y) / (t + 1)`
- Klein disk: `(x, y) / t`

Both projections require a valid future-sheet hyperboloid point.
