# Radion Tutorials

Seven progressive scenes that teach the engine one idea at a time. Open
`Tutorials.radion-project` in the editor, or run it with `radion_runner`, and
step through the scenes in order — each one builds on the last.

**Open the project, not a scene file.** `radion_runner` accepts either, but a
bare `.scene.json` makes it look for assets in `Scenes/Assets/`, which does
not exist — every script in that scene then fails to load. The project
manifest is what points `assetRoot` at the real `Assets/` folder:

```
radion_runner examples/tutorials/Tutorials.radion-project
```

To run one scene on its own, change `activeScene` in the manifest rather than
passing the scene path.

Every object in every scene is a procedural primitive — a `MeshRenderer` whose
mesh source is `Box`, `Plane`, `Sphere`, `Cylinder`, `Cone` or `Capsule`.
Nothing here references a mesh or texture file, so this project opens and runs
on a clean checkout with no asset library installed.

The scenes live in `Scenes/`, one `.scene.json` per tutorial. The scripts
live in `Assets/Scripts/`, one `.zen` file per script. Only the classes and
methods documented in `docs/script_api/` are used anywhere in this project —
if a script here calls something, that call is on one of those pages.

## 01 — Scene Basics

`Scenes/01_scene_basics.scene.json`. No scripts.

A `Ground` plane, a `Box`, a `Sphere` and a `Cylinder` sitting on it, a
`Main Camera`, and a directional `Sun` light. This is the minimum a scene
needs before anything is visible at all: a camera that is the scene's
`activeCamera`, and a light — without the `Sun`, every primitive here is
correctly positioned and completely black.

Look at: the `Sun`'s Transform. Its rotation is what points the directional
light, not its position — a directional light has no origin, only a
direction. Try rotating it in the editor and watch every shadow and every lit
face turn with it.

Change: the `Sun`'s `intensity` and `color`, or the camera's `fieldOfView`,
and see the whole scene react immediately, in the editor, with no script
involved.

## 02 — Hierarchy

`Scenes/02_hierarchy.scene.json`. No scripts.

A `Hub` (a flat cylinder) with four children arranged around it — `Orbit
Box`, `Orbit Sphere`, `Orbit Cone`, `Orbit Capsule` — each parented to `Hub`
in the Hierarchy panel.

Look at: select a child and read its Transform position in the Inspector.
That number is **local** — relative to `Hub`, not the world. `Orbit Box` at
local `[2, 0, 0]` sits two units east of wherever `Hub` currently is.

Change: select `Hub` and move or rotate it. Every child follows, because
their positions are stored relative to their parent. Now select just one
child, e.g. `Orbit Sphere`, and move only it — nothing else in the scene is
affected. That is the whole lesson: a parent's transform composes into its
children's world position; a child's own transform never affects its
siblings or its parent.

## 03 — First Script

`Scenes/03_first_script.scene.json`. Script: `Assets/Scripts/spin.zen`.

A single `Box` named `Spinner`, carrying a `ZenBehaviour` component that
runs `spin.zen`:

```python
class Spin:
    speed = 60.0

    def on_update(self, dt):
        self.node.yaw(self.speed * dt)
```

This is the smallest script that does anything: one class, one method.
`self.node` is the `GameObject` this script is attached to — every script
gets one, set before its own code ever runs. `on_update` fires once per
frame while the scene is playing, with `dt` (the frame's delta time in
seconds) as its only argument; `speed` is a plain class field, which is also
what the Inspector shows as this script's tunable property.

Look at: `Spinner` in the Inspector while the scene is playing — the `Spin`
script's `speed` field is editable live.

Change: `speed` in `spin.zen`, or override it per-instance from the
Inspector, and watch the spin rate change without touching anything else.

## 04 — Movement

`Scenes/04_movement.scene.json`. Script: `Assets/Scripts/patrol.zen`.

A `Patroller` capsule that sweeps back and forth along the X axis:

```python
import math

class Patrol:
    speed = 1.5
    distance = 3.0
    _time = 0.0

    def on_start(self):
        self.origin = self.node.get_position()

    def on_update(self, dt):
        self._time = self._time + dt
        offset = math.sin(self._time * self.speed) * self.distance
        self.node.set_position(Vec3(self.origin.x + offset, self.origin.y, self.origin.z))
```

This tutorial is about movement, not input: there is no keyboard, mouse or
gamepad exposed to scripts today — `docs/script_api/` has no `Input` class
and no method on any documented class that reads a device. Movement here is
driven by elapsed time instead: `on_start` remembers where the object began,
and every frame `on_update` places it at an offset computed from a running
clock (`_time`) run through `math.sin`.

Look at: the capsule sweeping smoothly, with no acceleration snap at the
turnaround points — that is `sin()`'s own shape, not anything hand-tuned.

Change: `speed` (how fast it oscillates) and `distance` (how far it travels)
independently.

## 05 — Spawn and Lifetime

`Scenes/05_spawn_and_lifetime.scene.json`. Script: `Assets/Scripts/spawner.zen`.

A `Spawner` object — it carries no `MeshRenderer` of its own, only a script —
and a small `Spawn Point` marker sphere next to it. Every second, `Spawner`
creates a new object with `self.scene.create()`, places it one unit further
along from the last, and after four seconds disposes the oldest one it is
still tracking:

```python
class Spawner:
    spawn_interval = 1.0
    lifetime = 4.0
    spacing = 1.0
    _timer = 0.0
    _clock = 0.0
    _count = 0.0

    def on_start(self):
        self.origin = self.scene.find("Spawn Point").get_position()
        self.spawned = []
        self.spawn_times = []
    ...
```

`on_start` reaches out to a different object by name with
`self.scene.find("Spawn Point")` to read its position — the script itself
has nothing to look at, so it borrows a place to start from. **Renaming
`Spawn Point` in the editor breaks this lookup**: `find()` matches by name,
not by any stable id, so `self.scene.find("Spawn Point")` returns `None` the
moment that object is renamed, and the next line (`.get_position()` on
`None`) is where the script would fail.

Every object `Spawner` creates is tracked in two parallel lists — the
`GameObject` handle and the moment it was created — so the oldest can be
found and disposed once `lifetime` seconds have passed, checked against
`is_disposed()` first so a script can never double-dispose the same object.

A created object starts with nothing but a transform, so `Spawner` gives each
one a shape as it goes:

```python
spawned_object.add_component(MeshRenderer).set_box(0.6, 0.6, 0.6)
```

`add_component()` returns the new component's handle, which is why that reads
as one line. `set_box()` builds the mesh and assigns it, and the shape arrives
with a lit default material already on it — no extra step, or it would be
invisible. Two objects asking for the same box share one upload, so a hundred
spawned crates cost one mesh.

Look at: the Hierarchy panel while the scene plays, next to the viewport. New
`Spawned` objects appear in the outliner and as boxes in the world, and a few
seconds later both go away together — that is `Scene.create()` and
`GameObject.dispose()`.

Change: `spawn_interval`, `lifetime` and `spacing` to make more or fewer
objects alive at once, spread further apart.

## 06 — Components From Script

`Scenes/06_components_from_script.scene.json`. Script:
`Assets/Scripts/conductor.zen`.

Three pre-placed primitives — `Rotor A` (a box), `Rotor B` (a sphere) and
`Rotor C` (a cone) — plus the `Sun` light, all driven by one `Conductor`
object that has no mesh of its own:

```python
def on_start(self):
    self.rotor_a = self.scene.find("Rotor A")
    self.rotor_b = self.scene.find("Rotor B")
    self.rotor_c = self.scene.find("Rotor C")
    self.light = self.scene.find("Sun").get_component(Light)
    self.base_b = self.rotor_b.get_position()
```

`Conductor` finds every object it drives by name with `self.scene.find()`
and holds on to the handles. As with tutorial 05, **the names are load-
bearing**: rename `Rotor B` in the editor and `self.rotor_b` becomes `None`
on the next reload, and the bob animation stops with a script error instead
of silently doing nothing.

Each frame, `on_update` does four separate things to four separate objects
it does not own:

- `Rotor A` is spun with `yaw()`, the same call as tutorial 03.
- `Rotor B` bobs up and down: its position is rewritten every frame relative
  to the position it started at.
- `Rotor C` is toggled fully on and off with `set_active()` — a script-
  driven blink, not a shader effect.
- The `Sun`'s `Light` component (fetched with `get_component(Light)` off the
  `GameObject` `self.scene.find("Sun")` returned) has its `intensity` and
  `color` rewritten every frame, cycling the whole scene's lighting warmer
  and brighter, then cooler and dimmer.

That last one is the tutorial's actual subject: `get_component()` returns a
handle to a component that already exists — a script does not create a
`Light`, it reaches one that a scene file already placed, and calls the
getters and setters `docs/script_api/Light.md` documents. A `MeshRenderer`
handle can be fetched the same way, and its `set_box()`/`set_sphere()` family
will replace a shape outright (tutorial 07 builds a whole scene out of them),
but there is still no way to change a material's **colour** from a script —
which is why the rotors here change position and their active flag, while the
only colour in motion belongs to the light.

Look at: the whole scene's lighting breathing in and out while three
unrelated shapes spin, bob and blink, all timed off the same `self._time` in
one script.

Change: `pulse_speed`, `bob_height` and `bob_speed` independently, or replace
`Rotor C`'s on/off blink with a different condition on the same
`math.sin(self._time)` value.

## 07 — Build From Script

`Scenes/07_build_from_script.scene.json`. Script: `Assets/Scripts/builder.zen`.

The scene file holds three objects: a camera, a `Sun`, and an empty `Builder`
that carries only a script. Everything you see — the ground and a five-by-five
grid of alternating boxes and spheres — is created at runtime:

```python
def on_start(self):
    ground = self.scene.create("Ground")
    ground.add_component(MeshRenderer).set_plane(20.0, 20.0)

    for row in range(self.rows):
        for column in range(self.columns):
            tile = self.scene.create("Tile")
            tile.set_position(position)

            renderer = tile.add_component(MeshRenderer)
            if (row + column) % 2 == 0:
                renderer.set_box(0.9, 0.9, 0.9)
            else:
                renderer.set_sphere(0.5)
```

This is tutorial 05's `create()` and tutorial 06's `get_component()` put
together and pushed as far as they go: a scene authored entirely in code, with
the scene file reduced to the three things a script cannot make for itself.

`on_update` then walks the grid every frame and lifts each tile by a sine wave
phased on its own position, which is where the ripple comes from. The starting
positions are kept in a second list, because each frame's height is computed
from where the tile *began*, not from where it was last frame — accumulating
onto the current position instead would drift.

Look at: the Hierarchy panel. Twenty-six objects exist there that appear in no
scene file. Stop play and they are all gone; the saved scene still holds only
the camera, the light and `Builder`.

Change: `columns` and `rows` to grow the grid, `spacing` to spread it,
`wave_speed` and `wave_height` to retune the ripple. Or swap `set_sphere(0.5)`
for `set_cylinder(0.4, 1.0)`, `set_cone(0.5, 1.0)`, `set_capsule(0.3, 0.6)` or
`set_torus(0.5, 0.2)` — the full set is in `docs/script_api/MeshRenderer.md`.

## What is not here

Three things a tutorial would want and the script API does not expose yet, so
that time is not lost looking for them:

- **Input.** No keyboard, mouse or gamepad. Tutorial 04 works around it.
- **Material colour.** A script can give an object a shape but not a colour;
  meshes arrive with a lit grey default.
- **Creating lights.** `add_component()` takes `Camera`, `MeshRenderer`,
  `CharacterController` and `Animator` only. A light has to come from the
  scene file, which is why tutorial 07's `Sun` is one of the three objects
  that stayed behind.
