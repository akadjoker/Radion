"""Builds the models for a helicopter-vs-jets game through the Radion Blender API
and saves each as glTF (.glb) and as the engine's own .rmesh.

    radion_blender --api &
    python3 game_shapes.py [output_dir]

Models (all face +Z, +Y up, origin at the middle of the body, metres):
  player_helicopter  the player's helicopter
  enemy_helicopter   dark attack helicopter with weapon pods
  enemy_jet          delta-wing fighter that chases the player
  rocket             the player's rocket
Each gets a screenshot next to the model.
"""

import math
import os
import sys

from blender_api_client import BlenderApi


def normal(a, b, c):
    ux, uy, uz = (b[i] - a[i] for i in range(3))
    vx, vy, vz = (c[i] - a[i] for i in range(3))
    return (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)


def orient(triangle, vertices, outward):
    """Counter-clockwise seen from outside: flips `triangle` when it faces `inward`."""
    a, b, c = (vertices[i] for i in triangle)
    n = normal(a, b, c)
    if sum(n[i] * outward[i] for i in range(3)) < 0:
        return [triangle[0], triangle[2], triangle[1]]
    return list(triangle)


def convex_solid(api, name, vertices, faces, **style):
    """A closed convex solid from polygon faces (lists of vertex indices, any
    winding). Each face is fanned into triangles and turned to face away from the
    middle of the solid, so the faces need not be listed carefully."""
    centre = [sum(v[i] for v in vertices) / len(vertices) for i in range(3)]
    triangles = []
    for face in faces:
        for k in range(1, len(face) - 1):
            tri = (face[0], face[k], face[k + 1])
            mid = [sum(vertices[i][axis] for i in tri) / 3 for axis in range(3)]
            triangles.append(orient(tri, vertices, [mid[i] - centre[i] for i in range(3)]))
    return api.call("add_mesh", name=name, positions=vertices, triangles=triangles, **style)


def prism(api, name, outline, half_thickness_root, half_thickness_tip, **style):
    """A flat swept slab: `outline` is four (x, z) corners, the first two at the
    root and the last two at the tip. Used for wings and stabilisers."""
    vertices = []
    for sign in (1, -1):
        for index, (x, z) in enumerate(outline):
            t = half_thickness_root if index < 2 else half_thickness_tip
            vertices.append([x, sign * t, z])
    faces = [(0, 1, 2, 3), (4, 5, 6, 7), (0, 4, 5, 1), (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0)]
    return convex_solid(api, name, vertices, faces, **style)


def helicopter(api, body, trim, glass, weapons=False):
    """Shared airframe; the enemy differs in paint and in carrying weapons."""
    api.call("new_document")
    api.call("add_loft", name="fuselage", color=body, roughness=0.45, axis="z", segments=32,
             sections=[
                 {"at": -1.2, "width": 0.7, "height": 0.8, "offset": [0, 0.1]},
                 {"at": -0.6, "width": 1.3, "height": 1.5},
                 {"at": 0.4, "width": 1.6, "height": 1.7},
                 {"at": 1.2, "width": 1.4, "height": 1.4, "offset": [0, -0.05]},
                 {"at": 1.9, "width": 0.8, "height": 0.8, "offset": [0, -0.1]},
                 {"at": 2.2, "width": 0.0, "height": 0.0, "offset": [0, -0.12]},
             ])
    api.call("add_loft", name="tail_boom", color=body, roughness=0.45, axis="z", segments=20,
             sections=[
                 {"at": -4.8, "width": 0.14, "height": 0.2, "offset": [0, 0.4]},
                 {"at": -3.0, "width": 0.26, "height": 0.36, "offset": [0, 0.27]},
                 {"at": -1.0, "width": 0.6, "height": 0.8, "offset": [0, 0.15]},
             ])
    api.call("add_primitive", type="sphere", name="canopy", color=glass, roughness=0.05,
             metallic=0.2, radius=0.5, scale=[1.45, 1.0, 1.5], position=[0, 0.25, 1.15])
    api.call("add_primitive", type="sphere", name="engine", color="#34495e", roughness=0.5,
             metallic=0.6, radius=0.5, scale=[1.0, 0.7, 1.7], position=[0, 0.95, -0.4])
    api.call("add_primitive", type="box", name="tail_fin", color=body, size=[0.07, 1.0, 0.7],
             position=[0, 0.85, -4.45], rotation=[-15, 0, 0])
    api.call("add_primitive", type="box", name="stabilizer", color=body, size=[1.5, 0.05, 0.4],
             position=[0, 0.5, -4.4])
    api.call("add_primitive", type="cylinder", name="mast", color=trim, metallic=0.8,
             roughness=0.3, radius=0.08, height=0.6, slices=16, position=[0, 1.55, -0.25])
    api.call("add_primitive", type="cylinder", name="rotor_hub", color=trim, metallic=0.8,
             radius=0.22, height=0.14, slices=24, position=[0, 1.9, -0.25])
    api.call("add_primitive", type="box", name="blade_a", color="#1c1c1c",
             size=[5.2, 0.035, 0.26], position=[0, 1.98, -0.25])
    api.call("add_primitive", type="box", name="blade_b", color="#1c1c1c",
             size=[5.2, 0.035, 0.26], position=[0, 1.98, -0.25], rotation=[0, 90, 0])
    api.call("add_primitive", type="cylinder", name="tail_hub", color=trim, radius=0.06,
             height=0.16, slices=12, rotation=[0, 0, 90], position=[0.12, 0.78, -4.75])
    api.call("add_primitive", type="box", name="tail_blade_a", color="#1c1c1c",
             size=[0.03, 0.9, 0.11], position=[0.2, 0.78, -4.75])
    api.call("add_primitive", type="box", name="tail_blade_b", color="#1c1c1c",
             size=[0.03, 0.11, 0.9], position=[0.2, 0.78, -4.75])

    # One side of the landing gear, mirrored for the other.
    left = []
    api.call("add_primitive", type="cylinder", name="skid_L", color="#555555", metallic=0.7,
             roughness=0.35, radius=0.045, height=2.6, slices=12, rotation=[90, 0, 0],
             position=[0.85, -1.15, 0.35])
    left.append("skid_L")
    for name, z in (("strut_front_L", 1.0), ("strut_rear_L", -0.4)):
        api.call("add_primitive", type="cylinder", name=name, color="#555555", metallic=0.7,
                 roughness=0.35, radius=0.035, height=0.6, slices=10, rotation=[0, 0, -18],
                 position=[0.74, -0.88, z])
        left.append(name)

    if weapons:
        # Stub wings carry a rocket pod each; pod and wing are mirrored together.
        api.call("add_primitive", type="box", name="stub_wing_L", color=body, size=[1.3, 0.06, 0.45],
                 position=[1.25, -0.25, 0.2])
        api.call("add_primitive", type="cylinder", name="pod_L", color=trim, metallic=0.5,
                 radius=0.17, height=0.95, slices=14, rotation=[90, 0, 0], position=[1.6, -0.42, 0.3])
        left += ["stub_wing_L", "pod_L"]

    for name in left:
        api.call("duplicate_part", part=name, mirror="x", name=name.replace("_L", "_R"))


def rocket(api):
    api.call("new_document")
    # Built along Y (simplest for a spun profile), then laid along +Z at the end.
    api.call("add_lathe", name="body", color="#d8d8d8", roughness=0.35, metallic=0.5, slices=20,
             profile=[[0.0, -0.9], [0.17, -0.9], [0.17, 0.5]])
    api.call("add_lathe", name="nose", color="#c0392b", roughness=0.4, slices=20,
             profile=[[0.17, 0.5], [0.15, 0.68], [0.09, 0.85], [0.0, 1.0]])
    api.call("add_lathe", name="band", color="#c0392b", roughness=0.4, slices=20,
             profile=[[0.175, -0.2], [0.175, -0.05]])
    api.call("add_lathe", name="nozzle", color="#3a3a3a", roughness=0.5, metallic=0.8, slices=20,
             profile=[[0.0, -1.02], [0.1, -1.0], [0.13, -0.9], [0.0, -0.9]])
    # Four fins around the tail; each box's long side is radial, turned about Y
    # to its own direction and set on the skin.
    for index in range(4):
        angle = math.radians(index * 90)
        api.call("add_primitive", type="box", name=f"fin{index}", color="#c0392b", roughness=0.45,
                 size=[0.3, 0.4, 0.02], rotation=[0, index * 90, 0],
                 position=[0.26 * math.cos(angle), -0.72, -0.26 * math.sin(angle)])
    # Everything above points along +Y; lay the rocket down so it flies along +Z.
    api.call("transform_mesh", rotation=[90, 0, 0])


def jet(api):
    api.call("new_document")
    api.call("add_loft", name="fuselage", color="#7f8c8d", roughness=0.35, metallic=0.4,
             axis="z", segments=28,
             sections=[
                 {"at": -3.2, "width": 0.9, "height": 0.7, "exponent": 2.5},
                 {"at": -2.0, "width": 1.25, "height": 0.85, "exponent": 2.5},
                 {"at": 0.4, "width": 1.35, "height": 0.9, "exponent": 2.5},
                 {"at": 2.2, "width": 0.9, "height": 0.6, "offset": [0, -0.05]},
                 {"at": 3.6, "width": 0.35, "height": 0.3, "offset": [0, -0.1]},
                 {"at": 4.2, "width": 0.0, "height": 0.0, "offset": [0, -0.12]},
             ])
    api.call("add_primitive", type="sphere", name="cockpit", color="#1f3a4d", roughness=0.05,
             metallic=0.3, radius=0.5, scale=[0.62, 0.5, 1.6], position=[0, 0.42, 1.6])
    # Delta wing, right side; mirrored below.
    prism(api, "wing_R", [(0.5, 0.6), (0.5, -2.6), (3.2, -2.9), (3.2, -2.3)], 0.09, 0.03,
          color="#7f8c8d", roughness=0.4, metallic=0.4)
    prism(api, "tailplane_R", [(0.35, -2.6), (0.35, -3.3), (1.5, -3.5), (1.5, -3.15)], 0.05, 0.02,
          color="#7f8c8d", roughness=0.4, metallic=0.4)
    api.call("add_primitive", type="box", name="fin_R", color="#c0392b", size=[0.05, 1.0, 0.9],
             position=[0.45, 0.85, -2.9], rotation=[0, 0, -12])
    api.call("add_lathe", name="nozzle", color="#3a3a3a", roughness=0.5, metallic=0.8, slices=20,
             rotation=[-90, 0, 0], position=[0, 0, -3.25],
             profile=[[0.0, 0.0], [0.45, 0.0], [0.42, -0.5], [0.0, -0.5]])
    for name in ("wing_R", "tailplane_R", "fin_R"):
        api.call("duplicate_part", part=name, mirror="x", name=name.replace("_R", "_L"))


def save(api, out, name, ground=False):
    """Writes the model and a screenshot. `ground` rests it on y = 0 (buildings,
    vehicles); otherwise it is centred on the origin (things that fly)."""
    os.makedirs(out, exist_ok=True)
    api.call("center_mesh", ground=ground)
    status = api.call("get_status")
    api.call("export_gltf", path=os.path.join(out, name + ".glb"))
    api.call("save_mesh", path=os.path.join(out, name + ".rmesh"))
    api.screenshot(os.path.join(out, name + ".png"), view="perspective", azimuth=-35,
                   elevation=20, width=800, height=480, grid=False)
    size = status["bounds"]["size"]
    print(f"{name}: {len(status['parts'])} parts, {status['triangles']} triangles, "
          f"{size[0]:.2f} x {size[1]:.2f} x {size[2]:.2f} m")


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "game_shapes"
    api = BlenderApi()

    helicopter(api, body="#3b6ea5", trim="#2c3e50", glass="#9bd0ea")
    save(api, out, "player_helicopter")

    helicopter(api, body="#3d4a3a", trim="#222b22", glass="#3d5a4a", weapons=True)
    save(api, out, "enemy_helicopter")

    jet(api)
    save(api, out, "enemy_jet")

    rocket(api)
    save(api, out, "rocket")


if __name__ == "__main__":
    main()
