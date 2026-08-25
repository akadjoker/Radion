# libzen — vendored copy

Zen is the scripting language Radion embeds for game behaviour. This is a
copy, not a submodule: every other library under `vendor/` is a copy too, and
`vendor/zen/CMakeLists.txt` is Radion's own (the upstream one globs `src/*.cpp`,
which would pull in the three modules left out below).

| | |
|---|---|
| Upstream repo | `https://github.com/akadjoker/zenpy` — directory `libzen/` |
| Commit | `1ef74fd` (`fix(vm): run() takes its own fiber when nested`), branch `physics-box2d` |
| Date | 2026-08-24 |
| Mirror | `https://github.com/akadjoker/zenpy_lib` — same files, standalone repo |
| License | zlib, see `LICENSE` |

`include/` is byte for byte identical to upstream. `src/` is identical except
for one file that is deliberately absent:

- `builtin_numpy.cpp`

Leaving it out needed no change to the library: module imports resolve
through a runtime registry, so a module that was never registered simply is
not there. It is not listed in `CMakeLists.txt` either.

## Keeping the copy in sync

Upstream is the source of truth. A bug found here is fixed in `zenpy/libzen`
first, then copied outwards:

```
zenpy/libzen  ->  zenpy_lib  ->  Radion vendor/zen
```

To check what has drifted, and to update the table above afterwards:

```sh
diff -rq vendor/zen/include <zenpy>/libzen/include
diff -rq vendor/zen/src     <zenpy>/libzen/src   # expect only builtin_numpy.cpp
```

## Changes carried here

None. The copy is upstream as of the commit above.
