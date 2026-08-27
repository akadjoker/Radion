# libzen — vendored copy

Zen is the scripting language Radion embeds for game behaviour. This is a
copy, not a submodule: every other library under `vendor/` is a copy too, and
`vendor/zen/CMakeLists.txt` is Radion's own (the upstream one globs `src/*.cpp`,
which would pull in modules Radion does not want, and would also pick up
`zen_host_output.cpp`, see below).

| | |
|---|---|
| Upstream repo | `https://github.com/akadjoker/zenpy` — directory `libzen/` |
| Commit | `a284b7c` (`a284b7c895da356883805b45b22c89f42c61a6a4`), branch `physics-box2d` |
| Date | 2026-08-24 |
| Mirror | `https://github.com/akadjoker/zenpy_lib` — same files, standalone repo |
| License | zlib, see `LICENSE` |
| Copied from | `Kinetix2D/external/zen`, branch `feature/web-exporter` at `512ff03`, plus two files from `test/windows-msvc` at `6b8010c` (see below) |
| Copy date | 2026-08-27 |

`include/` and `src/` are byte for byte the Kinetix2D copy, except for one
file deliberately absent:

- `builtin_numpy.cpp`

Leaving it out needed no change to the library: module imports resolve
through a runtime registry, so a module that was never registered simply is
not there. It is not listed in `CMakeLists.txt` either.

`zen_host_output.h` / `zen_host_output.cpp` are also carried over from
Kinetix2D — a writer hook `print()` and error output go through there, not
upstream. Nothing in the rest of `zen` references it, and it is deliberately
left out of `CMakeLists.txt`: it is unused dead weight here, kept only so the
copy stays a straight mirror of Kinetix2D's.

## Keeping the copy in sync

Upstream is the source of truth. A bug found here is fixed in `zenpy/libzen`
first, then propagated outwards by hand:

```
zenpy/libzen  ->  zenpy_lib  ->  Kinetix2D external/zen  ->  Radion vendor/zen
```

`Kinetix2D/external/zen/sync.sh` pulls from a `zenpy` checkout; there is no
equivalent script here yet; syncing means copying `include/` and `src/` from
either `zenpy/libzen` or `Kinetix2D/external/zen` over this directory, minus
`CMakeLists.txt` (Radion's own) and `sync.sh` (not used here).

To check what has drifted against the Kinetix2D copy:

```sh
diff -rq vendor/zen/include <Kinetix2D>/external/zen/include
diff -rq vendor/zen/src     <Kinetix2D>/external/zen/src
```

## Windows

Radion no longer carries its own Windows patches on top of this copy. It used
to — `std::filesystem` in `builtin_os.cpp`, and `int` instead of `ssize_t` in
the socket modules — and Kinetix2D has since solved the same problems on its
side, better, so the fixes now arrive with the copy instead of being
reapplied after it:

- `src/builtin_net.cpp`, `src/builtin_http.cpp` — `int` rather than `ssize_t`
  for `send`/`recv` results, which MSVC does not define
  (Kinetix2D `512ff03`).
- `include/zen/common.h` — `__builtin_expect` and `__builtin_unreachable`
  defined away under `_MSC_VER`, for the whole library rather than only
  `vm_dispatch.cpp` where the guard used to live (`36107a9`, `6b8010c`).
- `src/builtin_os.cpp` — real Win32 directory operations behind
  `#if defined(_WIN32)`: `direct.h`/`windows.h`, `FindFirstFileA` for
  `listdir`, `_getcwd`/`_chdir`. The POSIX branch is unchanged, so Linux
  builds exactly as before (`6b8010c`).

**Where those last two come from matters.** `builtin_os.cpp` and `common.h`
are taken from Kinetix2D's `test/windows-msvc` branch, not from
`feature/web-exporter` where the rest of this copy comes from: the branch
carries `6b8010c "Complete Zen MSVC portability fixes"`, which is not merged
back. Taking them is not a Radion patch — it is the same author's later work,
pulled from where it currently lives. A future sync must remember to look
there, or it will silently undo them.

`_WIN32` is defined by both MinGW and MSVC, so this compiles for either. That
branch ends on `c6067c3`/`64aa781`, which prepare and build the release
package with MinGW; MSVC was made to work first, and the shipped binary went
out with MinGW.

`CMakeLists.txt` stays Radion's own and is not touched by a sync.

## Changes carried here that are not upstream yet

Both were made on the Kinetix2D side and have not been pushed back to
`zenpy/libzen`, which is the declared source of the chain. Until they are,
**a sync taken from `zenpy` instead of from Kinetix2D will undo them**:

- `src/platform_time.h` adds `#include <cstdint>` — the fixed-width integer
  typedefs it uses were pulled in transitively before, which mingw does not
  guarantee.
- `src/bytecode.cpp` adds `can_write_global_class()` and uses it to gate
  `should_write_global_value()`: a native class (one with a native ctor/dtor,
  or a native callback anywhere in its methods, vtable or operator slots) is
  no longer serialized as a global. Native classes are recreated by
  `VM::resolve_native_globals()` when a bytecode image loads, and their C++
  callbacks cannot be serialized — writing them out corrupted the image. This
  one matters here as soon as components are bound to the VM, which is what
  `SceneScriptBindings` does.

The public API (`def_class`, `method`, `NativeLib`) is unchanged by any of it.
