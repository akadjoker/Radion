"""End-to-end checks of the editor's HTTP API, against a real running editor.

    python3 api_selftest.py                      # editor already running with --api
    python3 api_selftest.py --launch ../../bin/radion_blender   # start one
    python3 api_selftest.py --only edges,hide    # run some groups
"""

import argparse
import math
import os
import signal
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

    fresh_box(api)
    api.call("select", mode="face", action="set", box={"min": [-1.1, 0.9, -1.1], "max": [1.1, 1.1, 1.1]})
    r = api.call("subdivide")
    check(12 < r["triangles"] < 48, f"partial subdivide adds some triangles, got {r['triangles']}")
    api.call("select", mode="edge", action="all")
    before = status(api)["triangles"]
    api.call("undo")
    check(status(api)["triangles"] == 12, "undo restores the cube")
    expect_error(api, "invalid_params", "subdivide", levels=9)

    api.call("new_document")
    api.call("add_primitive", type="plane", size=[2, 1, 2], segments_x=1, segments_z=1)
    api.call("select", mode="edge", action="all")
    r = api.call("turn_edge")
    check(r["turned"] == 1, f"only the diagonal can turn, got {r['turned']}")

    fresh_box(api)
    api.call("select", mode="edge", action="set", edges=[[0, 1]])
    r = api.call("split_edge", t=0.5)
    check(r["split"] == 1 and r["triangles"] == 14, f"splitting an edge cuts its two triangles: {r}")
    check(r["selection"]["vertexCount"] >= 1, "the new vertices are selected")
    expect_error(api, "invalid_params", "split_edge", t=1.5)

    fresh_box(api)
    api.call("select", mode="edge", action="set", edges=[[0, 1]])
    r = api.call("collapse_edge")
    check(r["triangles"] == 10, f"collapse removes two triangles, got {r['triangles']}")
    api.call("select", action="clear")
    expect_error(api, "failed", "turn_edge")


def vertical_edge_of_cylinder(api):
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
    fresh_box(api)
    r = api.call("knife", axis="x", offset=0.5)
    check(r["triangles"] > 12, "the knife splits triangles")
    check(r["selection"]["mode"] == "edge" and r["selection"]["edgeCount"] >= 4, "the cut line is selected")
    check(approx(r["selection"]["edgeBounds"]["min"][0], 0.5) and approx(r["selection"]["edgeBounds"]["max"][0], 0.5),
          "the selected edges lie in the plane")
    expect_error(api, "failed", "knife", axis="x", offset=10)
    expect_error(api, "invalid_params", "knife")

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
    fresh_box(api)
    api.call("select", mode="face", action="set", box={"min": [-1.1, 0.9, -1.1], "max": [1.1, 1.1, 1.1]})
    api.call("delete_selection")
    check(status(api)["triangles"] == 10, "the top face (2 triangles) is gone")
    r = api.call("fill_holes")
    check(r["filled"] == 1 and r["triangles"] == 12, f"the hole is closed again: {r}")
    expect_error(api, "failed", "fill_holes")

    api.call("new_document")
    api.call("add_primitive", type="plane", size=[2, 1, 2], segments_x=1, segments_z=1, name="lower")
    api.call("add_primitive", type="plane", size=[2, 1, 2], segments_x=1, segments_z=1, name="upper", position=[0, 2, 0])
    r = api.call("bridge")  # the mesh has exactly two open borders
    check(r["trianglesAdded"] == 8 and r["triangles"] == 12, f"4 + 4 strip triangles between the planes: {r}")
    expect_error(api, "failed", "fill_holes")
    expect_error(api, "failed", "bridge")

    api.call("new_document")
    api.call("add_primitive", type="box", size=[1, 1, 1], position=[0.5, 0, 0])
    api.call("select", mode="face", action="set", box={"min": [-0.1, -1, -1], "max": [0.1, 1, 1]})
    api.call("delete_selection")
    r = api.call("mirror", axis="x")
    check(r["triangles"] == 20, f"10 + 10 triangles, got {r['triangles']}")
    check(approx(r["bounds"]["min"][0], -1.0) and approx(r["bounds"]["max"][0], 1.0), "the box is now 2 wide")
    expect_error(api, "failed", "fill_holes")
    expect_error(api, "invalid_params", "mirror", axis="w")

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


def group_paint(api):
    import tempfile
    glb = os.path.join(tempfile.mkdtemp(prefix="radion_selftest_"), "paint.glb")
    api.call("new_document")
    api.call("add_primitive", type="sphere", size=[2, 2, 2], name="ball", color="#ffffff")
    check(status(api)["hasVertexColors"] is False, "a new mesh has no vertex colours")

    r = api.call("paint_vertices", target="sphere", center=[0, 1, 0], radius=0.8, hardness=1.0, color="#ff0000", part="ball")
    check(r["painted"] > 0, f"the brush reached some vertices: {r}")
    check(status(api)["hasVertexColors"] is True, "vertex colours exist now")
    doc_glb = api.call("export_gltf", path=glb)
    doc = read_glb_json(glb)
    prim = doc["meshes"][0]["primitives"][0]
    check("COLOR_0" in prim["attributes"], "the glb carries COLOR_0")

    api.call("undo")
    check(status(api)["hasVertexColors"] is False, "undo removes the paint")

    api.call("select", mode="vertex", action="set", box={"min": [-2, 0.5, -2], "max": [2, 2, 2]})
    r = api.call("paint_vertices", color=[0, 0, 1], opacity=0.5)
    check(r["painted"] > 0, f"the selection was painted: {r}")
    r = api.call("clear_vertex_colors", target="selection")
    check(r["hasVertexColors"] is False, "clearing the same selection leaves nothing painted")

    api.call("paint_vertices", target="part", part="ball", color="#00ff00")
    api.call("paint_vertices", target="all", color="#0000ff", opacity=0.25)
    check(status(api)["hasVertexColors"] is True, "painted")
    r = api.call("clear_vertex_colors")
    check(r["hasVertexColors"] is False, "clear all")

    api.call("select", action="clear")
    expect_error(api, "failed", "paint_vertices", color="#ff0000")
    expect_error(api, "invalid_params", "paint_vertices", target="sphere", color="#ff0000")
    expect_error(api, "invalid_params", "paint_vertices", target="part", color="#ff0000")
    expect_error(api, "invalid_params", "paint_vertices", target="all")
    expect_error(api, "invalid_params", "paint_vertices", target="blob", color="#ff0000")


def group_uv(api):
    import tempfile
    folder = tempfile.mkdtemp(prefix="radion_selftest_")
    png = os.path.join(folder, "checker.png")
    write_checker_png(png)

    api.call("new_document")
    api.call("add_primitive", type="box", size=[2, 1, 4], name="hull")
    api.call("add_primitive", type="box", size=[1, 1, 1], name="crate", position=[5, 0, 0])

    # Box map: a 2 x 1 x 4 box at one repeat per unit spans 4 (the long side + its neighbour) in the u/v range.
    r = api.call("box_map_uv", target="part", part="hull", tile=1.0)
    data = api.call("get_uv_data", part="hull")
    check(data["vertexCount"] >= 24, f"the hull has UVs: {data['vertexCount']}")
    b = data["bounds"]
    check(b["max"][0] - b["min"][0] >= 1.99 and b["max"][1] - b["min"][1] >= 1.99, f"undistorted scale: {b}")
    check(data["islands"] >= 1, "islands are counted")

    api.call("box_map_uv", target="all", tile=1.0)
    r = api.call("fit_uv", target="all", per_part=True, margin=0.0)
    for name in ["hull", "crate"]:
        d = api.call("get_uv_data", part=name)
        check(d["insideUnitSquare"], f"{name} fits the unit square: {d['bounds']}")
        check(approx(d["bounds"]["min"][0], 0.0) or approx(d["bounds"]["min"][1], 0.0), f"{name} touches the frame")

    before = api.call("get_uv_data", part="crate")["bounds"]
    api.call("transform_uv", target="part", part="crate", scale=2, pivot=[0, 0])
    after = api.call("get_uv_data", part="crate")["bounds"]
    check(approx(after["max"][0], before["max"][0] * 2), f"scale doubles the coordinates: {before} -> {after}")
    api.call("transform_uv", target="part", part="crate", translate=[0.25, 0.0])
    moved = api.call("get_uv_data", part="crate")["bounds"]
    check(approx(moved["min"][0], after["min"][0] + 0.25), "translate slides u")
    api.call("transform_uv", target="part", part="crate", flip="u")
    api.call("undo")
    check(approx(api.call("get_uv_data", part="crate")["bounds"]["min"][0], moved["min"][0]), "undo restores the flip")

    api.call("select", mode="vertex", action="all")
    r = api.call("pin_uv", target="part", part="crate")
    check(r["pinnedVertices"] > 0, "vertices are pinned")
    expect_error(api, "failed", "transform_uv", target="part", part="crate", translate=[0.5, 0.5])
    api.call("pin_uv", target="part", part="crate", pinned=False)
    api.call("transform_uv", target="part", part="crate", translate=[0.0, 0.0], rotate=90)

    api.call("select", action="clear")
    expect_error(api, "failed", "transform_uv", target="selection", translate=[0.1, 0])
    expect_error(api, "failed", "transform_uv", target="island", translate=[0.1, 0])
    api.call("select", mode="face", action="set", faces=[0, 1])
    r = api.call("transform_uv", target="selection", translate=[0.1, 0])
    check(r["moved"] >= 4, f"a face's vertices moved: {r}")
    r = api.call("transform_uv", target="island", translate=[0.0, 0.0])
    check(r["moved"] >= r["moved"], "island target works")

    api.call("set_texture", part="hull", file=png)
    r = api.call_raw("uv_layout", part="hull", size=128)
    check(r is not None, "the layout image is returned")

    expect_error(api, "invalid_params", "transform_uv", target="part", translate=[0, 0])
    expect_error(api, "invalid_params", "transform_uv", target="part", part="crate", scale=0)
    expect_error(api, "invalid_params", "fit_uv", target="part", part="crate", per_part=True)
    expect_error(api, "invalid_params", "box_map_uv", target="part", part="ghost")

    import shutil
    shutil.rmtree(folder, ignore_errors=True)


def group_misc(api):
    import tempfile
    folder = tempfile.mkdtemp(prefix="radion_selftest_")

    api.call("new_document")
    api.call("add_lathe", profile=[[0.0, 0.0], [0.5, 0.0], [0.5, 1.0], [0.0, 1.0]], name="lathe")
    api.call("add_loft", sections=[{"at": 0.0, "width": 1.0, "height": 1.0}, {"at": 1.0, "width": 0.5, "height": 0.5}], name="loft", position=[3, 0, 0])
    api.call("add_mesh", positions=[[0, 0, 0], [1, 0, 0], [0, 1, 0]], triangles=[[0, 1, 2]], name="tri", position=[6, 0, 0])
    check(len(status(api)["parts"]) == 3, "three parts were added")

    api.call("style_part", part="tri", color="#00ff00")
    api.call("transform_part", part="tri", position=[6, 1, 0])
    api.call("duplicate_part", part="tri", name="tri2", color="#0000ff", position=[8, 0, 0])
    api.call("set_part_visible", part="tri2", visible=False)
    api.call("set_part_visible", part="tri2", visible=True)
    api.call("flip_winding", part="tri2")
    api.call("extract_part", part="tri2")
    check(len(status(api)["parts"]) == 1, "extract leaves one part")

    fresh_box(api)
    api.call("subdivide")
    api.call("transform_mesh", position=[0, 1, 0])
    api.call("center_mesh", ground=True)
    api.call("weld_vertices", distance=0.001)
    api.call("smooth_vertices", iterations=1, strength=0.2)
    api.call("recalculate_normals")
    api.call("generate_uv", mode="planar")
    api.call("bisect", axis="y", offset=0.0, keep="positive")
    api.call("convex_hull")
    api.call("optimize")
    api.call("simplify", ratio=0.9)
    expect_error(api, "failed", "set_animation", frame=0)

    obj = os.path.join(folder, "m.obj")
    rmesh = os.path.join(folder, "m.rmesh")
    api.call("export_obj", path=obj)
    api.call("save_mesh", path=rmesh)
    api.call("new_document")
    api.call("add_primitive", type="box", name="base")
    api.call("append_mesh", path=rmesh)
    check(len(status(api)["parts"]) >= 2, "the saved mesh was appended")
    api.call("delete_part", part=0)
    api.call("load_mesh", path=rmesh)
    check(status(api)["triangles"] > 0, "the saved mesh loads back")

    import shutil
    shutil.rmtree(folder, ignore_errors=True)


GROUPS = {"edges": group_edges, "hide": group_hide, "snap": group_snap, "subdivide": group_subdivide, "cuts": group_cuts, "assemble": group_assemble, "solids": group_solids, "textures": group_textures, "paint": group_paint, "uv": group_uv, "misc": group_misc}


class CheckedApi:
    """After every successful document-changing command, checks the undo stack grew as the command's `undoable` flag says."""

    SKIP = {"undo", "redo", "new_document", "load_mesh"}

    def __init__(self, api):
        self._api = api
        self._info = {c["name"]: c for c in api.commands()}
        self.checked = set()

    def __getattr__(self, name):
        return getattr(self._api, name)

    def call(self, command, **arguments):
        info = self._info.get(command)
        tracked = info is not None and not info["readOnly"] and command not in self.SKIP
        before = self._api.call("get_status")["undoSteps"] if tracked else None
        result = self._api.call(command, **arguments)
        if tracked:
            after = self._api.call("get_status")["undoSteps"]
            expected = 1 if info["undoable"] else 0
            check(after - before == expected,
                  f"{command}: undo steps changed by {after - before}, the listing says undoable={info['undoable']}")
            self.checked.add(command)
        return result


def stop(process):
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
    except (ProcessLookupError, PermissionError):
        pass
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)


def launch(binary):
    env = dict(os.environ, MESA_GL_VERSION_OVERRIDE="4.5", LIBGL_ALWAYS_SOFTWARE="1")
    command = [binary, "--api-port", "7439"]
    if not os.environ.get("DISPLAY") and shutil.which("xvfb-run"):
        command = ["xvfb-run", "-a", "-s", "-screen 0 1920x1080x24"] + command
    # Own process group, so stopping it also stops the editor xvfb-run started.
    process = subprocess.Popen(command, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                               start_new_session=True)
    api = BlenderApi("http://127.0.0.1:7439")
    for _ in range(60):
        try:
            api._request("GET", "/api/health")
            return process, api
        except Exception:
            time.sleep(0.5)
    stop(process)
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

    api = CheckedApi(api)
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
            stop(process)

    unchecked = sorted(name for name, info in api._info.items()
                       if not info["readOnly"] and name not in CheckedApi.SKIP and name not in api.checked)
    if unchecked:
        print("\nnot exercised by this run (undo flag unchecked):", ", ".join(unchecked))

    print(f"\n{len(FAILURES)} failure(s)")
    for failure in FAILURES:
        print(" -", failure)
    return 1 if FAILURES else 0


if __name__ == "__main__":
    sys.exit(main())
