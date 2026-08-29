# libzen — vendored copy

Zen is the scripting language Radion embeds for game behaviour. This is still
a copy, but it is on its way to becoming a **submodule** — the reason for the
copy was that there was no single repo to point at, and that is being fixed
upstream. What still has to happen before the switch is listed under
"Becoming a submodule" at the end.

`vendor/zen/CMakeLists.txt` is Radion's own and stays that way: the upstream
one globs `src/*.cpp`, which would pull in modules Radion does not want.

| | |
|---|---|
| Upstream repo | `https://github.com/akadjoker/zenpy` — directory `libzen/` |
| Commit | `a284b7c` (`a284b7c895da356883805b45b22c89f42c61a6a4`), branch `physics-box2d` |
| Date | 2026-08-24 |
| License | zlib, see `LICENSE` |
| Copied from | `Kinetix2D/external/zen`, branch `feature/web-exporter` at `512ff03`, plus two files from `test/windows-msvc` at `6b8010c` (see below) |
| Copy date | 2026-08-27 |

`include/` and `src/` are byte for byte the Kinetix2D copy, except for one
file deliberately absent:

- `builtin_numpy.cpp`

Leaving it out needed no change to the library: module imports resolve
through a runtime registry, so a module that was never registered simply is
not there. It is not listed in `CMakeLists.txt` either.

`zen_host_output.h` / `zen_host_output.cpp` used to be carried here too, from
Kinetix2D, and were **deleted on 2026-08-28**. They offered a settable writer
hook for `print()` and error output, but Radion never used it: the file was
not in `CMakeLists.txt`, the header was included by nothing, and the
`zen_write`/`zen_writes` macros the library actually calls resolve through
`include/zen/zenconf.h` to plain `fwrite`/`fputs` instead. Removing them
changed no build output. They also could not have survived the move to a
submodule, since a submodule directory cannot carry local files.

## Keeping the copy in sync

`zenpy/libzen` is the source of truth, and as of 2026-08-28 the only one:
`zenpy_lib` is being retired. A bug found here is fixed in `zenpy/libzen`
first, then propagated by hand:

```
zenpy/libzen  ->  Kinetix2D external/zen  ->  Radion vendor/zen
```

Do not sync from `zenpy_lib`. It lags: as of 2026-08-28 it had no generic
call support at all (`generic_call_expr` absent from
`src/compiler_expressions.cpp`), which this copy has and
`tests/ZenBehaviourTests.cpp` depends on.

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

## Becoming a submodule

Measured 2026-08-28. The direction is settled; these are what still block it.

**1. There is no repo to point at yet.** `zenpy/libzen` is a directory inside
the `zenpy` repo, not a repo of its own, so it cannot be a submodule as it
stands. Either it gets split out, or `vendor/zen` becomes a submodule of the
whole `zenpy` repo and `CMakeLists.txt` reaches into `libzen/` — which works
and keeps one source of truth, at the cost of carrying zenpy's other files.

**2. This copy has drifted from `zenpy/libzen`.** Nine files differ. Most of
it is reformatting, not content — ignoring whitespace the totals collapse:

| file | raw | real |
|---|---|---|
| `src/builtin_base.cpp` | 2561 | 293 |
| `src/invoke_string.inl` | 761 | 35 |
| `src/builtin_os.cpp` | 453 | 41 |
| the other six | — | ~112 |

About 480 real lines, and the compiler files (`compiler_expressions.cpp`,
`compiler_statements.cpp`, `compiler.h`, `common.h`) are already identical to
upstream, which is why generics match. Each of those 480 has to be either
pushed upstream or dropped as stale before a submodule can replace this
directory — a submodule keeps none of them.

The two changes named under "Changes carried here that are not upstream yet"
are the known-good part of that drift and must go up, not be dropped.

**3. `builtin_numpy.cpp` comes back.** A submodule delivers every file, so
the module can no longer be excluded by deleting it. It gets excluded from
`CMakeLists.txt` instead, which is already ours — a one-line change, and the
runtime module registry means an unregistered module simply is not there.
