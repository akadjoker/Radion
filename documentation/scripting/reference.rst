Script API reference
=====================

The class-by-class reference is not kept here. It lives in ``docs/script_api/``
in the working tree, as hand-written Markdown, one page per class:

* ``index.md`` - the class list and what a script instance is handed.
* ``Lifecycle.md`` - the methods the engine calls, and handle lifetime.
* ``Component.md``, ``GameObject.md``, ``Scene.md``, ``Vec3.md``
* ``Camera.md``, ``Light.md``, ``MeshRenderer.md``, ``CharacterController.md``,
  ``MoveResult.md``, ``Animator.md``, ``AnimationLayer.md``

That folder is the one reference kept in step with the bindings: a method and
its entry land in the same change, or the reference is already a lie. This
page used to carry a copy of it and drifted out of date within two commits,
which is the whole reason it now points instead of repeating.
