Scripting
=========

Radion scripts run on the vendored Zen VM (``vendor/zen``), in a Python-like
syntax. A script is attached to a GameObject through the ``ZenBehaviour``
component; each behaviour derives from ``ScriptComponent`` and receives its
host GameObject as ``self.node`` before its own ``__init__`` runs.

Scripts run **only in Play**. A script body is compiled once per file (or
source string) no matter how many objects use it - each object gets its own
cheap instance with its own state, through ``ScriptCache``.

.. toctree::
   :maxdepth: 2

   reference

Contents
--------

* :doc:`reference` — the script API registered so far: the component handle
  classes (``Component``, ``GameObject``, ``Camera``, ``Light``) and their
  methods.
