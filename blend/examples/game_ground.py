"""Ground models for the helicopter game, built through the Radion Blender API.

    radion_blender --api &
    python3 game_ground.py [output_dir]

Models face +Z, +Y is up, and each rests on y = 0 with its footprint centred (metres).
"""

import math
import os
import sys

from blender_api_client import BlenderApi
from game_shapes import convex_solid, orient, save

OLIVE = "#56653a"
DARK_OLIVE = "#3c4829"
TAN = "#b39b6a"
STEEL = "#4a4f55"
TYRE = "#1d1d1d"
GLASS = "#8fc0d8"
CONCRETE = "#9a9a94"


def rotate_x(theta_degrees, point):
    t = math.radians(theta_degrees)
    x, y, z = point
    return [x, y * math.cos(t) - z * math.sin(t), y * math.sin(t) + z * math.cos(t)]


def missile_truck(api):
    api.call("new_document")
    api.call("add_primitive", type="box", name="chassis", color=DARK_OLIVE, roughness=0.8,
             size=[2.3, 0.4, 8.2], position=[0, 0.95, 0])
    api.call("add_primitive", type="box", name="deck", color=OLIVE, roughness=0.8,
             size=[2.5, 0.22, 5.0], position=[0, 1.26, -1.3])
    api.call("add_primitive", type="box", name="cab", color=OLIVE, roughness=0.7,
             size=[2.4, 1.5, 2.0], position=[0, 1.9, 3.0])
    api.call("add_primitive", type="box", name="hood", color=OLIVE, roughness=0.7,
             size=[2.3, 0.8, 1.3], position=[0, 1.45, 4.6])
    api.call("add_primitive", type="box", name="windshield", color=GLASS, roughness=0.05,
             metallic=0.2, size=[2.1, 0.7, 0.05], position=[0, 2.25, 4.02], rotation=[-12, 0, 0])
    api.call("add_primitive", type="box", name="bumper", color=STEEL, metallic=0.6, roughness=0.5,
             size=[2.4, 0.25, 0.25], position=[0, 0.95, 5.35])

    left = []
    for index, z in enumerate((3.9, -0.2, -2.9)):
        name = f"wheel{index}_L"
        api.call("add_primitive", type="cylinder", name=name, color=TYRE, roughness=0.95,
                 radius=0.6, height=0.5, slices=20, rotation=[0, 0, 90], position=[1.2, 0.6, z])
        left.append(name)

    # The launcher hinges at the rear of the deck and leans 35 degrees up toward
    # the front. Its local Y runs along the missiles.
    hinge = [0.0, 1.5, -3.3]
    tilt = 55  # rotation about X that puts local +Y 35 degrees above the horizon

    def place(local):
        x, y, z = rotate_x(tilt, local)
        return [hinge[0] + x, hinge[1] + y, hinge[2] + z]

    api.call("add_primitive", type="cylinder", name="turntable", color=STEEL, metallic=0.5,
             roughness=0.5, radius=0.75, height=0.25, slices=24, position=[0, 1.5, -3.0])
    api.call("add_primitive", type="box", name="launcher_frame", color=STEEL, metallic=0.5,
             roughness=0.5, size=[1.7, 2.6, 0.14], position=place([0, 1.05, 0]), rotation=[tilt, 0, 0])
    for column, x in enumerate((-0.4, 0.4)):
        for row, z in enumerate((0.3, 0.75)):
            tag = f"{column}{row}"
            api.call("add_lathe", name=f"missile{tag}", color="#dcdcd6", roughness=0.35,
                     metallic=0.4, slices=16, position=place([x, 1.0, z]), rotation=[tilt, 0, 0],
                     profile=[[0.0, -1.0], [0.16, -1.0], [0.16, 0.6]])
            api.call("add_lathe", name=f"missile_nose{tag}", color="#c0392b", roughness=0.4,
                     slices=16, position=place([x, 1.0, z]), rotation=[tilt, 0, 0],
                     profile=[[0.16, 0.6], [0.14, 0.8], [0.08, 0.97], [0.0, 1.08]])
    for name in left:
        api.call("duplicate_part", part=name, mirror="x", name=name.replace("_L", "_R"))


def radar(api):
    api.call("new_document")
    api.call("add_primitive", type="cylinder", name="base", color=CONCRETE, roughness=0.9,
             radius=2.2, height=0.4, slices=32, position=[0, 0.2, 0])
    api.call("add_primitive", type="box", name="cabin", color=OLIVE, roughness=0.8,
             size=[2.6, 1.8, 2.4], position=[-2.6, 1.3, 0.3])
    api.call("add_primitive", type="box", name="cabin_door", color=DARK_OLIVE, roughness=0.8,
             size=[0.7, 1.4, 0.05], position=[-2.6, 1.1, 1.52])
    api.call("add_primitive", type="cylinder", name="mast", color="#b8bcc2", roughness=0.5,
             metallic=0.5, radius=0.28, height=3.8, slices=20, position=[0, 2.3, 0])
    api.call("add_primitive", type="cylinder", name="turntable", color=STEEL, metallic=0.6,
             roughness=0.4, radius=0.55, height=0.3, slices=24, position=[0, 4.3, 0])
    api.call("add_primitive", type="box", name="arm", color="#b8bcc2", metallic=0.5, roughness=0.5,
             size=[0.3, 1.2, 0.3], position=[0, 4.9, -0.1])

    tip = 60
    centre = [0.0, 5.6, 0.2]
    depth = lambda r: 0.25 * r * r
    radii = [0.0, 0.35, 0.7, 1.05, 1.4]
    outer = [[r, depth(r)] for r in radii]
    inner = [[r, depth(r) + 0.06] for r in reversed(radii)]
    api.call("add_lathe", name="dish_back", color="#e6e8ea", roughness=0.45, slices=40,
             profile=outer, position=centre, rotation=[tip, 0, 0], cap_start=False, cap_end=False)
    api.call("add_lathe", name="dish_face", color="#f4f4f2", roughness=0.35, slices=40,
             profile=inner, position=centre, rotation=[tip, 0, 0], cap_start=False, cap_end=False)
    api.call("add_lathe", name="dish_rim", color="#e6e8ea", roughness=0.45, slices=40,
             profile=[[1.4, depth(1.4)], [1.4, depth(1.4) + 0.06]], position=centre,
             rotation=[tip, 0, 0], cap_start=False, cap_end=False)

    axis = rotate_x(tip, [0, 1, 0])
    feed = [centre[i] + axis[i] * 0.55 for i in range(3)]
    api.call("add_primitive", type="cylinder", name="feed", color=STEEL, metallic=0.6, radius=0.04,
             height=1.0, slices=10, position=feed, rotation=[tip, 0, 0])
    tipcentre = [centre[i] + axis[i] * 1.05 for i in range(3)]
    api.call("add_primitive", type="sphere", name="feed_tip", color="#c0392b", radius=0.1,
             slices=12, rings=8, position=tipcentre)
    api.call("add_primitive", type="cylinder", name="antenna", color=STEEL, radius=0.025,
             height=1.6, slices=8, position=[-3.3, 3.0, 0.3])
    api.call("add_primitive", type="sphere", name="beacon", color="#c0392b", radius=0.09,
             slices=10, rings=6, position=[0, 4.55, 0.4])


def tent_barracks(api):
    api.call("new_document")
    half, length, wall, ridge = 2.6, 5.4, 1.1, 3.3
    profile = [(-half, 0), (half, 0), (half, wall), (0, ridge), (-half, wall)]
    vertices = [[x, y, -length] for x, y in profile] + [[x, y, length] for x, y in profile]
    faces = [(0, 1, 2, 3, 4), (5, 6, 7, 8, 9)]
    for k in range(5):
        n = (k + 1) % 5
        faces.append((k, n, n + 5, k + 5))
    convex_solid(api, "canvas", vertices, faces, color="#6f7d4c", roughness=1.0)

    api.call("add_primitive", type="box", name="door", color="#1f241a", roughness=1.0,
             size=[1.3, 2.0, 0.06], position=[0, 1.0, length + 0.03])
    api.call("add_primitive", type="box", name="door_frame", color=DARK_OLIVE, roughness=1.0,
             size=[1.5, 2.2, 0.04], position=[0, 1.1, length + 0.015])
    for end, z in (("front", length), ("rear", -length)):
        api.call("add_primitive", type="cylinder", name=f"pole_{end}", color="#6b4a2b",
                 roughness=0.9, radius=0.05, height=0.6, slices=8, position=[0, ridge + 0.1, z])
    bag = 0
    for side in (-1, 1):
        for step in range(4):
            for layer in range(2):
                x = side * (1.25 + step * 0.58 + (0.29 if layer else 0))
                api.call("add_primitive", type="box", name=f"sandbag{bag}", color=TAN,
                         roughness=1.0, size=[0.55, 0.22, 0.34],
                         position=[x, 0.11 + layer * 0.22, length + 0.75])
                bag += 1
    api.call("add_primitive", type="cylinder", name="flagpole", color="#cfcfcf", metallic=0.6,
             radius=0.035, height=4.4, slices=8, position=[half + 1.4, 2.2, length + 0.6])
    api.call("add_primitive", type="box", name="flag", color="#c0392b", roughness=0.9,
             size=[0.9, 0.55, 0.02], position=[half + 1.85, 3.95, length + 0.6])


def hut_barracks(api):
    api.call("new_document")
    radius, length, segments = 3.0, 6.4, 18

    arch = []
    for j in range(segments + 1):
        phi = math.pi * j / segments
        x, y = radius * math.cos(phi), radius * math.sin(phi)
        arch.append([x, y, -length])
        arch.append([x, y, length])
    triangles = []
    for j in range(segments):
        a, b, c, d = 2 * j, 2 * j + 1, 2 * j + 3, 2 * j + 2
        mid = [(arch[a][i] + arch[c][i]) / 2 for i in range(3)]
        outward = [mid[0], mid[1], 0.0]
        triangles.append(orient((a, b, c), arch, outward))
        triangles.append(orient((a, c, d), arch, outward))
    api.call("add_mesh", name="roof", positions=arch, triangles=triangles, color="#7d8a6a",
             roughness=0.8, metallic=0.2)

    # The two end walls: their own vertices, so they stay flat against the curve.
    for end, z in (("front", length), ("rear", -length)):
        ring = [[radius * math.cos(math.pi * j / segments), radius * math.sin(math.pi * j / segments), z]
                for j in range(segments + 1)]
        centre = [0.0, 0.0, z]
        vertices = ring + [centre]
        tris = [orient((j, j + 1, len(ring)), vertices, [0, 0, 1 if z > 0 else -1])
                for j in range(segments)]
        api.call("add_mesh", name=f"wall_{end}", positions=vertices, triangles=tris,
                 color="#8b9477", roughness=0.9)

    api.call("add_primitive", type="box", name="slab", color=CONCRETE, roughness=0.95,
             size=[radius * 2 + 0.8, 0.2, length * 2 + 1.6], position=[0, -0.1, 0])
    api.call("add_primitive", type="box", name="door", color="#2a2f24", roughness=0.9,
             size=[1.3, 2.1, 0.08], position=[0, 1.05, length + 0.04])
    api.call("add_primitive", type="box", name="step", color=CONCRETE, roughness=0.95,
             size=[1.8, 0.18, 0.7], position=[0, 0.09, length + 0.45])
    for side in (-1, 1):
        api.call("add_primitive", type="box", name=f"window{side}", color=GLASS, roughness=0.1,
                 metallic=0.2, size=[0.7, 0.6, 0.06], position=[side * 1.7, 1.6, length + 0.04])
    api.call("add_primitive", type="cylinder", name="chimney", color="#3a3a3a", roughness=0.7,
             metallic=0.5, radius=0.14, height=1.3, slices=12, position=[1.4, 3.05, -2.5])
    api.call("add_primitive", type="cylinder", name="chimney_cap", color="#2a2a2a", roughness=0.7,
             radius=0.2, height=0.08, slices=12, position=[1.4, 3.75, -2.5])
    for index in range(7):
        z = -length + 1.0 + index * (2 * length - 2.0) / 6
        rib = [[(radius + 0.06) * math.cos(math.pi * j / segments),
                (radius + 0.06) * math.sin(math.pi * j / segments), z] for j in range(segments + 1)]
        inner = [[(radius - 0.02) * math.cos(math.pi * j / segments),
                  (radius - 0.02) * math.sin(math.pi * j / segments), z] for j in range(segments + 1)]
        vertices = rib + inner
        n = segments + 1
        tris = []
        for j in range(segments):
            mid = [rib[j][0], rib[j][1], 0.0]
            tris.append(orient((j, j + 1, n + j + 1), vertices, mid))
            tris.append(orient((j, n + j + 1, n + j), vertices, mid))
        api.call("add_mesh", name=f"rib{index}", positions=vertices, triangles=tris,
                 color="#66725a", roughness=0.8, position=[0, 0, 0])


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "game_ground"
    api = BlenderApi()
    for name, build in (("missile_truck", missile_truck), ("radar", radar),
                        ("barracks_tent", tent_barracks), ("barracks_hut", hut_barracks)):
        build(api)
        save(api, out, name, ground=True)


if __name__ == "__main__":
    main()
