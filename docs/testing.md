# Testing

Run all tests:

```bash
ctest --test-dir build --output-on-failure
```

Current targets:

- `hyperbolica_smoke`: tiny sanity check.
- `hyperbolica_math_tests`: hyperboloid invariants, distance, normalization,
  projections, and `{4,6}` constants.
- `hyperbolica_spherical_tests`: 2-sphere invariants, the spherical law of
  cosines, rotations, stereographic/gnomonic projections, frame movement, and
  `{4,3}` cube constants.
- `hyperbolica_tiling_tests`: base polygon vertices, generated tile centers,
  duplicate checks, crossing detection, rebasing, invalid tiling input, and
  spherical `{4,3}`/`{3,4}` patch closure.
- `hyperbolica_render_tests`: shader file loading and triangle mesh data.

Shared assertion helpers are in `tests/test_support.h`. The helpers are small
on purpose, just enough for the current tests without pulling in a testing
framework yet.
