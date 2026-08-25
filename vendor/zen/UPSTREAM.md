# libzen — vendored copy

Zen is the scripting language Radion embeds for game behaviour. This is a
copy, not a submodule: every other library under `vendor/` is a copy too, and
`vendor/zen/CMakeLists.txt` is Radion's own (the upstream one globs `src/*.cpp`,
which would pull in the three modules left out below).

| | |
|---|---|
| Upstream repo | `https://github.com/akadjoker/zenpy` — directory `libzen/` |
| Commit | `2a7e9f8` (`feat: add runtime generic call syntax`), branch `main` |
| Date | 2026-08-25 |
| Mirror | `https://github.com/akadjoker/zenpy_lib` — same files, standalone repo |
| License | zlib, see `LICENSE` |

`include/` is byte for byte identical to upstream. `src/` is identical except
for one file that is deliberately absent and the small portability patch noted
below:

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

- `src/platform_time.h`, plus its use from `builtin_time.cpp` and
  `builtin_base.cpp` — portable wall-clock, monotonic-clock and sleep helpers
  for Windows and POSIX platforms.
- `builtin_os.cpp` — directory operations use C++17 `std::filesystem`, so the
  registered `os` module builds on Windows as well as Linux.
- `builtin_net.cpp` and `builtin_http.cpp` — use `int` for socket send/receive
  result values, avoiding the platform-specific `ssize_t` typedef.
- `invoke_string.inl` — uses libzen's portable `find_sep` helper instead of
  the GNU-only `memmem`, matching the Kinetix2D copy.
- `CMakeLists.txt` — uses MSVC's `/O2` instead of GCC/Clang's `-O3`.
