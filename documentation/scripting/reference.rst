Script API reference
=====================

This page lists the script API exactly as the bindings register it, class by
class. ``get_component(cls)`` returns ``None`` when the object has no such
component.

.. contents::
   :local:

The ``Component`` class
------------------------

Every script-facing component handle (``Camera``, ``Light``, ...) derives
from ``Component``.

.. py:method:: Component.is_active()

   Returns whether the component is active.

.. py:method:: Component.set_active(active)

   Sets whether the component is active.

The ``GameObject`` class
-------------------------

``GameObject`` wraps a scene object. Every script instance receives one as
``self.node`` (and, for backward compatibility, ``self.owner``).

.. py:method:: GameObject.get_name()

   Returns the object name as a string.

.. py:method:: GameObject.set_name(name)

   Sets the object name.

.. py:method:: GameObject.get_active()

.. py:method:: GameObject.set_active(active)

   Gets / sets whether the object is active.

.. py:method:: GameObject.get_position()

   Returns the local position as a ``Vec3``.

.. py:method:: GameObject.set_position(position)

   Sets the local position from a ``Vec3``.

.. py:method:: GameObject.get_scale()

   Returns the local scale as a ``Vec3``.

.. py:method:: GameObject.set_scale(scale)

   Sets the local scale from a ``Vec3``.

.. py:method:: GameObject.get_rotation()

   Returns the local rotation as Euler degrees, in a ``Vec3``.

.. py:method:: GameObject.set_rotation(rotation)

   Sets the local rotation from Euler degrees, in a ``Vec3``.

.. py:method:: GameObject.yaw(degrees)

.. py:method:: GameObject.pitch(degrees)

.. py:method:: GameObject.roll(degrees)

   Rotates the object around its local up / right / forward axis, in degrees.

.. py:method:: GameObject.get_component(cls)

   Returns the component matching ``cls``, or ``None``. ``cls`` is a class
   itself (``get_component(Light)``), not a string - the Zen compiler lowers
   ``node.get_component<Light>()`` to this call, passing the class as the
   argument. Accepts ``Camera`` for the ``Camera`` component, and either
   ``Light`` or ``DirectionalLight`` for the ``Light`` component.

The ``Camera`` class
----------------------

.. py:method:: Camera.get_field_of_view()

   Returns the vertical field of view, in degrees (perspective mode).

.. py:method:: Camera.set_perspective(fov_degrees, aspect, near, far)

   Switches the camera to perspective projection with the given vertical
   field of view (degrees), aspect ratio, and near/far clip planes.

.. py:method:: Camera.get_orthographic_size()

   Returns the vertical half-height of the orthographic view volume.

.. py:method:: Camera.set_orthographic(size, aspect, near, far)

   Switches the camera to orthographic projection with the given size, aspect
   ratio, and near/far clip planes.

.. py:method:: Camera.get_aspect()

.. py:method:: Camera.set_aspect(aspect)

   Gets / sets the aspect ratio, independently of the projection mode.

.. py:method:: Camera.get_near_plane()

.. py:method:: Camera.get_far_plane()

   Returns the near / far clip plane distance.

The ``Light`` class
----------------------

.. py:method:: Light.get_color()

   Returns the light colour as a ``Vec3`` (0-1 per channel).

.. py:method:: Light.set_color(color)

   Sets the light colour from a ``Vec3`` (0-1 per channel).

.. py:method:: Light.get_intensity()

.. py:method:: Light.set_intensity(intensity)

   Gets / sets the light intensity.

.. py:method:: Light.get_casts_shadows()

.. py:method:: Light.set_cast_shadows(enabled)

   Gets / sets whether the light casts shadows.

.. py:method:: Light.get_volumetric()

.. py:method:: Light.set_volumetric(enabled)

   Gets / sets whether the light contributes to volumetric fog.
