"""Builds a small helicopter in a running Radion Blender, through the HTTP API.

    radion_blender --api &
    python3 helicopter.py [output_dir]

The nose points toward +Z, +Y is up.
"""

import os
import sys

from blender_api_client import BlenderApi

RED = "#c0392b"
DARK = "#2c3e50"
BLACK = "#1c1c1c"
METAL = "#555555"


def build(api):
    api.call("new_document")

    api.call("add_loft", name="fuselage", color=RED, roughness=0.4, axis="z", segments=32,
             sections=[
                 {"at": -1.2, "width": 0.7, "height": 0.8, "offset": [0, 0.1]},
                 {"at": -0.6, "width": 1.3, "height": 1.5},
                 {"at": 0.4, "width": 1.6, "height": 1.7},
                 {"at": 1.2, "width": 1.4, "height": 1.4, "offset": [0, -0.05]},
                 {"at": 1.9, "width": 0.8, "height": 0.8, "offset": [0, -0.1]},
                 {"at": 2.2, "width": 0.0, "height": 0.0, "offset": [0, -0.12]},
             ])
    api.call("add_loft", name="tail_boom", color=RED, roughness=0.4, axis="z", segments=20,
             sections=[
                 {"at": -4.8, "width": 0.14, "height": 0.2, "offset": [0, 0.4]},
                 {"at": -3.0, "width": 0.26, "height": 0.36, "offset": [0, 0.27]},
                 {"at": -1.0, "width": 0.6, "height": 0.8, "offset": [0, 0.15]},
             ])

    api.call("add_primitive", type="sphere", name="canopy", color="#7fb8d8", roughness=0.05,
             metallic=0.2, radius=0.5, scale=[1.45, 1.0, 1.5], position=[0, 0.25, 1.15])
    api.call("add_primitive", type="sphere", name="engine", color="#34495e", roughness=0.5,
             metallic=0.6, radius=0.5, scale=[1.0, 0.7, 1.7], position=[0, 0.95, -0.4])
    api.call("add_primitive", type="box", name="tail_fin", color=RED, size=[0.07, 1.0, 0.7],
             position=[0, 0.85, -4.45], rotation=[-15, 0, 0])
    api.call("add_primitive", type="box", name="stabilizer", color=RED, size=[1.5, 0.05, 0.4],
             position=[0, 0.5, -4.4])

    api.call("add_primitive", type="cylinder", name="mast", color=DARK, metallic=0.8,
             roughness=0.3, radius=0.08, height=0.6, slices=16, position=[0, 1.55, -0.25])
    api.call("add_primitive", type="cylinder", name="rotor_hub", color=DARK, metallic=0.8,
             radius=0.22, height=0.14, slices=24, position=[0, 1.9, -0.25])
    api.call("add_primitive", type="box", name="blade_a", color=BLACK,
             size=[5.2, 0.035, 0.26], position=[0, 1.98, -0.25])
    api.call("add_primitive", type="box", name="blade_b", color=BLACK,
             size=[5.2, 0.035, 0.26], position=[0, 1.98, -0.25], rotation=[0, 90, 0])

    api.call("add_primitive", type="cylinder", name="tail_hub", color=DARK, radius=0.06,
             height=0.16, slices=12, rotation=[0, 0, 90], position=[0.12, 0.78, -4.75])
    api.call("add_primitive", type="box", name="tail_blade_a", color=BLACK,
             size=[0.03, 0.9, 0.11], position=[0.2, 0.78, -4.75])
    api.call("add_primitive", type="box", name="tail_blade_b", color=BLACK,
             size=[0.03, 0.11, 0.9], position=[0.2, 0.78, -4.75])

    left = ["skid_L", "strut_front_L", "strut_rear_L"]
    api.call("add_primitive", type="cylinder", name="skid_L", color=METAL, metallic=0.7,
             roughness=0.35, radius=0.045, height=2.6, slices=12, rotation=[90, 0, 0],
             position=[0.85, -1.15, 0.35])
    for name, z in (("strut_front_L", 1.0), ("strut_rear_L", -0.4)):
        api.call("add_primitive", type="cylinder", name=name, color=METAL, metallic=0.7,
                 roughness=0.35, radius=0.035, height=0.6, slices=10, rotation=[0, 0, -18],
                 position=[0.74, -0.88, z])
    for name in left:
        api.call("duplicate_part", part=name, mirror="x", name=name.replace("_L", "_R"))


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    os.makedirs(out, exist_ok=True)
    api = BlenderApi()
    build(api)

    status = api.call("get_status")
    print(f"{len(status['parts'])} parts, {status['triangles']} triangles")

    api.screenshot(os.path.join(out, "perspective.png"), view="perspective", azimuth=-35, elevation=22)
    for view in ("right", "top", "front"):
        api.screenshot(os.path.join(out, f"{view}.png"), view=view)
    print("screenshots written to", os.path.abspath(out))


if __name__ == "__main__":
    main()
