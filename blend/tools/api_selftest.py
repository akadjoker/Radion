"""End-to-end checks of the editor's HTTP API, against a real running editor.

    python3 api_selftest.py                      # editor already running with --api
    python3 api_selftest.py --launch ../../bin/radion_blender   # start one (headless if no display)
    python3 api_selftest.py --only edges,hide    # run some groups

Each group builds something small, acts on it through the API and checks numbers
(counts, bounds, part names) - not pictures. Exits non-zero on the first group
that fails, after running the others.
"""

import argparse
import math
import os
import shutil
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "examples"))
from blender_api_client import BlenderApi, BlenderApiError  # noqa: E402

FAILURES = []


def check(condition, message):
    if not condition:
        FAILURES.append(message)
        print("  FAIL:", message)


def approx(a, b, tolerance=1e-3):
    return abs(a - b) <= tolerance


def expect_error(api, code, command, **args):
    try:
        api.call(command, **args)
    except BlenderApiError as error:
        check(error.code == code, f"{command}: expected {code}, got {error.code} ({error})")
        return
    check(False, f"{command}: expected an error ({code})")


def fresh_box(api, size=2.0):
    api.call("new_document")
    api.call("add_primitive", type="box", size=[size, size, size], name="cube", color="#c0392b")


def status(api):
    return api.call("get_status")


# ------------------------------------------------------------------ groups

def group_edges(api):
    fresh_box(api)
    s = api.call("select", mode="edge", action="all")
    check(s["edgeCount"] == 18, f"a cube has 18 edges (12 + 6 diagonals), got {s['edgeCount']}")

    s = api.call("select", mode="edge", action="set",
                 box={"min": [-1.1, 0.9, -1.1], "max": [1.1, 1.1, 1.1]})
    check(s["edgeCount"] == 5, f"top face: 4 edges + 1 diagonal, got {s['edgeCount']}")
    check(approx(s["edgeBounds"]["min"][1], 1.0), "selected edges sit at y = 1")

    s = api.call("select", action="grow")
    check(s["edgeCount"] > 5, "grow adds edges")
    s = api.call("select", action="linked")
    check(s["edgeCount"] == 18, "linked floods the whole connected cube")

    # An edge picked by two vertex indices, and the error paths.
    api.call("select", mode="edge", action="set", edges=[[0, 1]])
    check(api.call("get_selection")["edgeCount"] == 1, "one edge by [a, b]")
    expect_error(api, "invalid_params", "select", mode="edge", edges=[[0, 999]])
    expect_error(api, "invalid_params", "select", mode="edge", edges=[[0, 6]])

    before = status(api)["triangles"]
    api.call("delete_selection")
    check(status(api)["triangles"] == before - 2, "deleting an edge removes the two triangles on it")

    # Moving one corner moves every vertex standing at that point: no cracks.
    fresh_box(api)
    api.call("select", mode="vertex", action="set", vertices=[0])
    api.call("transform_selection", position=[0, 0, 5])
    data = api.call("get_mesh_data", max_vertices=200)
    moved = [p for p in data["positions"] if approx(p[2], 6.0)]
    check(len(moved) == 3, f"the 3 coincident corner vertices moved together, got {len(moved)}")
    api.call("undo")
    check(approx(status(api)["bounds"]["size"][2], 2.0), "undo restores the cube")


def group_hide(api):
    fresh_box(api)
    api.call("select", mode="face", action="set", box={"min": [-1.1, 0.9, -1.1], "max": [1.1, 1.1, 1.1]})
    check(api.call("hide")["hiddenTriangles"] == 2, "the top face (2 triangles) is hidden")
    check(status(api)["hiddenTriangles"] == 2, "status reports them")
    check(api.call("select", mode="face", action="all")["faceCount"] == 10, "hidden faces cannot be selected")
    expect_error(api, "failed", "hide", what="unselected")  # everything selectable is selected
    check(api.call("unhide")["revealedTriangles"] == 2, "unhide reveals them")
    check(api.call("select", mode="face", action="all")["faceCount"] == 12, "all 12 selectable again")

    api.call("select", mode="face", action="set", faces=[0, 1])
    api.call("hide")
    api.call("select", mode="face", action="set", faces=[4])
    api.call("extrude", distance=0.3)
    check(status(api)["hiddenTriangles"] == 0, "an edit that changes the triangles shows everything")
    api.call("select", action="clear")
    expect_error(api, "failed", "hide")  # nothing selected, nothing to hide


def group_snap(api):
    api.call("new_document")
    api.call("add_primitive", type="box", size=[1, 1, 1], name="a")
    api.call("add_primitive", type="box", size=[1, 1, 1], name="b", position=[1.02, 0, 0])
    # Select part b; its -X face sits 0.02 from a's +X face.
    api.call("select", mode="vertex", action="set", part="b")
    expect_error(api, "failed", "snap_to_vertex", tolerance=0.001)
    r = api.call("snap_to_vertex", tolerance=0.05)
    check(r["moved"] >= 4, f"at least the 4 facing corners snapped, got {r['moved']}")
    data = api.call("get_mesh_data", part="b", max_vertices=100)
    xs = sorted(set(round(p[0], 3) for p in data["positions"]))
    check(approx(xs[0], 0.5), f"b's near face now lies on a's far face (x = 0.5), got {xs}")

    api.call("new_document")
    api.call("add_primitive", type="box", size=[1, 1, 1], position=[0.013, 0.027, 0.0])
    api.call("select", action="clear")
    r = api.call("snap_to_grid", step=0.1)
    check(r["moved"] > 0, "unselected snaps the whole mesh")
    data = api.call("get_mesh_data", max_vertices=100)
    check(all(approx(round(c / 0.1) * 0.1, c, 1e-4) for p in data["positions"] for c in p),
          "every coordinate is a multiple of 0.1")
    expect_error(api, "invalid_params", "snap_to_grid", step=0)


def group_subdivide(api):
    fresh_box(api)
    api.call("select", action="clear")
    r = api.call("subdivide")
    check(r["triangles"] == 48, f"flat subdivide: 12 -> 48 triangles, got {r['triangles']}")
    check(approx(r["bounds"]["size"][0], 2.0), "flat subdivide does not move the surface")

    fresh_box(api)
    r = api.call("subdivide", smooth=True, levels=2)
    check(r["triangles"] == 192, f"two smooth levels: 12 x 16 = 192, got {r['triangles']}")
    check(r["bounds"]["size"][0] < 2.0, "smooth subdivision pulls a cube's corners in")

    # One face only: its neighbours are cut just enough to stay watertight.
    fresh_box(api)
    api.call("select", mode="face", action="set", box={"min": [-1.1, 0.9, -1.1], "max": [1.1, 1.1, 1.1]})
    r = api.call("subdivide")
    check(12 < r["triangles"] < 48, f"partial subdivide adds some triangles, got {r['triangles']}")
    api.call("select", mode="edge", action="all")
    before = status(api)["triangles"]
    # Every edge still has two triangles (closed surface): collapsing/turning is what
    # tells us; here just make sure undo round-trips.
    api.call("undo")
    check(status(api)["triangles"] == 12, "undo restores the cube")
    expect_error(api, "invalid_params", "subdivide", levels=9)

    # Turn the one diagonal of a two-triangle plane; the border edges refuse.
    api.call("new_document")
    api.call("add_primitive", type="plane", size=[2, 1, 2], segments_x=1, segments_z=1)
    api.call("select", mode="edge", action="all")
    r = api.call("turn_edge")
    check(r["turned"] == 1, f"only the diagonal can turn, got {r['turned']}")

    # Split: new vertices appear and become the selection.
    fresh_box(api)
    api.call("select", mode="edge", action="set", edges=[[0, 1]])
    r = api.call("split_edge", t=0.5)
    check(r["split"] == 1 and r["triangles"] == 14, f"splitting an edge cuts its two triangles: {r}")
    check(r["selection"]["vertexCount"] >= 1, "the new vertices are selected")
    expect_error(api, "invalid_params", "split_edge", t=1.5)

    # Collapse removes the triangles on the edge.
    fresh_box(api)
    api.call("select", mode="edge", action="set", edges=[[0, 1]])
    r = api.call("collapse_edge")
    check(r["triangles"] == 10, f"collapse removes two triangles, got {r['triangles']}")
    api.call("select", action="clear")
    expect_error(api, "failed", "turn_edge")


GROUPS = {"edges": group_edges, "hide": group_hide, "snap": group_snap, "subdivide": group_subdivide}


# ------------------------------------------------------------------- driver

def launch(binary):
    env = dict(os.environ, MESA_GL_VERSION_OVERRIDE="4.5", LIBGL_ALWAYS_SOFTWARE="1")
    command = [binary, "--api-port", "7439"]
    if not os.environ.get("DISPLAY") and shutil.which("xvfb-run"):
        command = ["xvfb-run", "-a", "-s", "-screen 0 1920x1080x24"] + command
    process = subprocess.Popen(command, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    api = BlenderApi("http://127.0.0.1:7439")
    for _ in range(60):
        try:
            api._request("GET", "/api/health")
            return process, api
        except Exception:
            time.sleep(0.5)
    process.terminate()
    raise SystemExit("the editor did not start")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--launch", help="path to radion_blender; starts it for the run")
    parser.add_argument("--only", help="comma-separated group names")
    parser.add_argument("--url", default="http://127.0.0.1:7420")
    arguments = parser.parse_args()

    process = None
    if arguments.launch:
        process, api = launch(arguments.launch)
    else:
        api = BlenderApi(arguments.url)

    names = arguments.only.split(",") if arguments.only else list(GROUPS)
    try:
        for name in names:
            print(f"[{name}]")
            before = len(FAILURES)
            try:
                GROUPS[name](api)
            except Exception as error:  # a command failing unexpectedly is a failure, not a crash
                FAILURES.append(f"{name}: {type(error).__name__}: {error}")
                print("  ERROR:", error)
            print("  ok" if len(FAILURES) == before else "  FAILED")
    finally:
        if process:
            process.terminate()
            subprocess.run(["pkill", "Xvfb"], check=False)

    print(f"\n{len(FAILURES)} failure(s)")
    for failure in FAILURES:
        print(" -", failure)
    return 1 if FAILURES else 0


if __name__ == "__main__":
    sys.exit(main())
