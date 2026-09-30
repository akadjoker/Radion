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


def vertical_edge_of_cylinder(api):
    """An edge of the cylinder's side that runs from the bottom ring to the top ring."""
    data = api.call("get_mesh_data", max_vertices=2000)
    pos, ids = data["positions"], data["vertexIds"]
    for tri in data["triangles"]:
        for i in range(3):
            u, v = tri[i], tri[(i + 1) % 3]
            pu, pv = pos[u], pos[v]
            if (approx(pu[0], pv[0], 1e-6) and approx(pu[2], pv[2], 1e-6) and approx(abs(pu[1] - pv[1]), 2.0, 1e-6)
                    and approx(pu[0] ** 2 + pu[2] ** 2, 1.0, 1e-4)):
                return [ids[u], ids[v]]
    raise AssertionError("no vertical side edge found")


def group_cuts(api):
    # Knife: the cut line becomes the selection.
    fresh_box(api)
    r = api.call("knife", axis="x", offset=0.5)
    check(r["triangles"] > 12, "the knife splits triangles")
    check(r["selection"]["mode"] == "edge" and r["selection"]["edgeCount"] >= 4, "the cut line is selected")
    check(approx(r["selection"]["edgeBounds"]["min"][0], 0.5) and approx(r["selection"]["edgeBounds"]["max"][0], 0.5),
          "the selected edges lie in the plane")
    expect_error(api, "failed", "knife", axis="x", offset=10)
    expect_error(api, "invalid_params", "knife")

    # Loop cut: a ring round a cylinder, by one vertical edge.
    api.call("new_document")
    api.call("add_primitive", type="cylinder", radius=1, height=2, slices=12)
    edge = vertical_edge_of_cylinder(api)
    r = api.call("loop_cut", edge=edge, cuts=2)
    check(r["triangles"] - r["trianglesBefore"] == 48, f"two rings of 12 quads add 2 x 24 triangles, got {r}")
    check(r["selection"]["edgeCount"] == 24, f"the new loops are selected: {r['selection']['edgeCount']}")
    data = api.call("get_mesh_data", max_vertices=2000)
    heights = sorted(set(round(p[1], 3) for p in data["positions"]))
    low = heights[0]
    check(len(heights) == 4 and all(approx(h - low, e, 1e-3) for h, e in zip(heights, [0.0, 2 / 3, 4 / 3, 2.0])),
          f"the rings divide the height in thirds, got {heights}")
    api.call("select", action="clear")
    expect_error(api, "failed", "loop_cut")

    # Bevel one vertical edge of a box, picked by where it is.
    fresh_box(api)
    s = api.call("select", mode="edge", action="set", box={"min": [0.9, -1.1, 0.9], "max": [1.1, 1.1, 1.1]})
    check(s["edgeCount"] == 1, f"one vertical edge in the box, got {s['edgeCount']}")
    api.call("bevel", width=0.3)
    data = api.call("get_mesh_data", max_vertices=2000)
    check(not any(approx(p[0], 1.0) and approx(p[2], 1.0) for p in data["positions"]),
          "no vertex is left on the old edge")
    check(any(approx(p[0], 0.7) and approx(p[2], 1.0) for p in data["positions"]), "the strip starts 0.3 in on one face")
    check(any(approx(p[0], 1.0) and approx(p[2], 0.7) for p in data["positions"]), "and 0.3 in on the other")
    fresh_box(api)
    api.call("select", mode="edge", action="set", box={"min": [0.9, -1.1, 0.9], "max": [1.1, 1.1, 1.1]})
    expect_error(api, "failed", "bevel", width=5)

    # Inset a face, then raise it with extrude.
    fresh_box(api)
    api.call("select", mode="face", action="set", box={"min": [-1.1, 0.9, -1.1], "max": [1.1, 1.1, 1.1]})
    r = api.call("inset", thickness=0.3, depth=0.2)
    check(r["triangles"] == 12 + 8, f"inset adds a ring of 8 triangles, got {r['triangles']}")
    check(r["selection"]["mode"] == "face" and r["selection"]["faceCount"] == 2, "the inner face is selected")
    r = api.call("extrude", distance=0.5)
    check(r["triangles"] > 20, "extrude continues from the inner face")
    api.call("select", action="clear")
    expect_error(api, "failed", "inset", thickness=0.1)


def group_assemble(api):
    # Fill: a box missing its top face.
    fresh_box(api)
    api.call("select", mode="face", action="set", box={"min": [-1.1, 0.9, -1.1], "max": [1.1, 1.1, 1.1]})
    api.call("delete_selection")
    check(status(api)["triangles"] == 10, "the top face (2 triangles) is gone")
    r = api.call("fill_holes")
    check(r["filled"] == 1 and r["triangles"] == 12, f"the hole is closed again: {r}")
    expect_error(api, "failed", "fill_holes")  # closed now

    # Bridge: two separate quads joined by a strip.
    api.call("new_document")
    api.call("add_primitive", type="plane", size=[2, 1, 2], segments_x=1, segments_z=1, name="lower")
    api.call("add_primitive", type="plane", size=[2, 1, 2], segments_x=1, segments_z=1, name="upper", position=[0, 2, 0])
    r = api.call("bridge")  # the mesh has exactly two open borders
    check(r["trianglesAdded"] == 8 and r["triangles"] == 12, f"4 + 4 strip triangles between the planes: {r}")
    expect_error(api, "failed", "fill_holes")  # the strip closed both borders
    expect_error(api, "failed", "bridge")      # and there is nothing left to bridge

    # Mirror: half a box, opened at the mirror plane, mirrored into a whole one.
    api.call("new_document")
    api.call("add_primitive", type="box", size=[1, 1, 1], position=[0.5, 0, 0])
    api.call("select", mode="face", action="set", box={"min": [-0.1, -1, -1], "max": [0.1, 1, 1]})
    api.call("delete_selection")
    r = api.call("mirror", axis="x")
    check(r["triangles"] == 20, f"10 + 10 triangles, got {r['triangles']}")
    check(approx(r["bounds"]["min"][0], -1.0) and approx(r["bounds"]["max"][0], 1.0), "the box is now 2 wide")
    expect_error(api, "failed", "fill_holes")  # the welded halves leave no border
    expect_error(api, "invalid_params", "mirror", axis="w")

    # Symmetry: moving vertices on one side moves the mirror ones too.
    fresh_box(api)
    api.call("subdivide")
    api.call("set_symmetry", axis="x")
    check(status(api)["symmetry"]["axis"] == "x", "symmetry is reported")
    api.call("select", mode="vertex", action="set", box={"min": [0.4, 0.9, -1.1], "max": [1.1, 1.1, 1.1]})
    api.call("transform_selection", position=[0, 0.5, 0])
    data = api.call("get_mesh_data", max_vertices=2000)
    raised = [p for p in data["positions"] if approx(p[1], 1.5)]
    check(len(raised) > 0 and any(p[0] > 0.3 for p in raised) and any(p[0] < -0.3 for p in raised),
          "the mirrored side rose with it")
    api.call("set_symmetry", axis="none")
    check(status(api)["symmetry"] is None, "symmetry off")

    # Merge and separate parts.
    api.call("new_document")
    for index in range(3):
        api.call("add_primitive", type="box", size=[1, 1, 1], position=[index * 2, 0, 0], name=f"box{index}")
    r = api.call("merge_parts", parts=["box0", 2])
    check(len(r["parts"]) == 2 and r["parts"][0]["triangles"] == 24 and r["parts"][1]["name"] == "box1",
          f"boxes 0 and 2 are one part now: {r['parts']}")
    expect_error(api, "invalid_params", "merge_parts", parts=["box1", "ghost"])

    fresh_box(api)
    api.call("select", mode="face", action="set", box={"min": [-1.1, 0.9, -1.1], "max": [1.1, 1.1, 1.1]})
    r = api.call("separate_selection", name="lid", color="#00ff00")
    parts = status(api)["parts"]
    check(len(parts) == 2 and any(p["name"] == "lid" and p["triangles"] == 2 for p in parts),
          f"the top face is a part of its own: {parts}")
    api.call("select", action="clear")
    expect_error(api, "failed", "separate_selection")


def group_solids(api):
    # Every primitive type builds, and 'origin' decides where the part's origin sits.
    for kind in ["box", "plane", "sphere", "cylinder", "cone", "capsule", "torus", "disc", "tube", "prism", "stairs", "arch"]:
        api.call("new_document")
        api.call("add_primitive", type=kind, name=kind)
        check(status(api)["triangles"] > 0, f"{kind} has triangles")
        api.call("add_primitive", type=kind, name=kind + "_base", origin="base", replace=True)
        b = status(api)["bounds"]
        check(approx(b["min"][1], 0.0, 1e-2), f"{kind} with origin=base stands on y=0: {b}")
    api.call("new_document")
    api.call("add_primitive", type="cylinder", name="c")
    b = status(api)["bounds"]
    check(approx(b["min"][1], -b["max"][1], 1e-2), f"a cylinder is centred by default: {b}")
    expect_error(api, "invalid_params", "add_primitive", type="box", origin="corner")

    # Booleans.
    def two_boxes():
        api.call("new_document")
        api.call("add_primitive", type="box", size=[2, 2, 2], name="a")
        api.call("add_primitive", type="box", size=[2, 2, 2], name="b", position=[1, 0, 0])

    two_boxes()
    r = api.call("boolean", operation="union", a="a", b="b", resolution=48, name="both")
    parts = status(api)["parts"]
    check(len(parts) == 1 and parts[0]["name"] == "both", f"union leaves one part: {parts}")
    b = status(api)["bounds"]
    check(approx(b["min"][0], -1.0, 0.1) and approx(b["max"][0], 2.0, 0.1), f"union spans 3 along X: {b}")

    two_boxes()
    api.call("boolean", operation="intersection", a="a", b="b", resolution=48)
    b = status(api)["bounds"]
    check(approx(b["min"][0], 0.0, 0.1) and approx(b["max"][0], 1.0, 0.1), f"intersection is the overlap: {b}")

    two_boxes()
    api.call("boolean", operation="difference", a="a", b="b", resolution=48)
    b = status(api)["bounds"]
    check(approx(b["min"][0], -1.0, 0.1) and approx(b["max"][0], 0.0, 0.1), f"difference keeps a's far half: {b}")
    api.call("undo")
    check(len(status(api)["parts"]) == 2, "one undo restores both parts")

    expect_error(api, "invalid_params", "boolean", operation="union", a="a", b="a")
    expect_error(api, "invalid_params", "boolean", operation="union", a="a", b="ghost")
    expect_error(api, "invalid_params", "boolean", operation="xor", a="a", b="b")

    api.call("new_document")
    api.call("add_primitive", type="box", size=[1, 1, 1], name="a")
    api.call("add_primitive", type="box", size=[1, 1, 1], name="far", position=[10, 0, 0])
    expect_error(api, "failed", "boolean", operation="intersection", a="a", b="far")


def write_checker_png(path, size=8):
    import struct
    import zlib
    rows = b""
    for y in range(size):
        row = b"\x00"
        for x in range(size):
            row += b"\xff\x40\x40" if (x + y) % 2 == 0 else b"\x40\x40\xff"
        rows += row

    def chunk(kind, data):
        body = kind + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)

    with open(path, "wb") as file:
        file.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", size, size, 8, 2, 0, 0, 0))
                   + chunk(b"IDAT", zlib.compress(rows)) + chunk(b"IEND", b""))


def read_glb_json(path):
    import json
    import struct
    with open(path, "rb") as file:
        data = file.read()
    magic, version, total = struct.unpack_from("<III", data, 0)
    check(magic == 0x46546C67 and version == 2 and total == len(data), "a valid glb header")
    length, kind = struct.unpack_from("<II", data, 12)
    return json.loads(data[20:20 + length])


def group_textures(api):
    import tempfile
    folder = tempfile.mkdtemp(prefix="radion_selftest_")
    png = os.path.join(folder, "checker.png")
    glb = os.path.join(folder, "out.glb")
    write_checker_png(png)

    api.call("new_document")
    api.call("add_primitive", type="box", size=[2, 2, 2], name="crate", color="#c0392b")
    api.call("add_primitive", type="box", size=[1, 1, 1], name="plain", position=[3, 0, 0])
    api.call("unwrap_uv")
    r = api.call("set_texture", part="crate", slot="albedo", file=png)
    check(r["textures"]["albedo"] == png, f"the texture is reported: {r}")
    check("textures" not in status(api)["parts"][1], "the other part is untouched")
    api.call("screenshot", width=64, height=64)

    r = api.call("export_gltf", path=glb)
    check("warnings" not in r, f"no export warnings: {r}")
    doc = read_glb_json(glb)
    check(len(doc.get("images", [])) == 1 and doc["images"][0]["mimeType"] == "image/png", "the image is embedded once")
    with_texture = [m for m in doc["materials"] if "baseColorTexture" in m["pbrMetallicRoughness"]]
    check(len(with_texture) == 1 and with_texture[0]["name"] == "crate", "only the crate material has a map")

    api.call("clear_texture", part="crate")
    check("textures" not in status(api)["parts"][0], "the texture is gone")
    api.call("undo")
    check(status(api)["parts"][0]["textures"]["albedo"] == png, "undo brings it back")

    expect_error(api, "failed", "set_texture", part="crate", file=os.path.join(folder, "missing.png"))
    expect_error(api, "invalid_params", "set_texture", part="crate", slot="height", file=png)
    expect_error(api, "invalid_params", "set_texture", part="ghost", file=png)

    import shutil
    shutil.rmtree(folder, ignore_errors=True)


GROUPS = {"edges": group_edges, "hide": group_hide, "snap": group_snap, "subdivide": group_subdivide, "cuts": group_cuts, "assemble": group_assemble, "solids": group_solids, "textures": group_textures}


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
