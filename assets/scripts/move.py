import math

class MoveRotate(ScriptComponent):
    speed = 2.0
    spin = 90.0
    radius = 3.0
    _time = 0.0

    def on_start(self):
        self.center = self.node.get_position()

    def on_update(self, dt):
        self._time = self._time + dt
        self.node.set_position(Vec3(
            self.center.x + math.cos(self._time * self.speed) * self.radius,
            self.center.y,
            self.center.z + math.sin(self._time * self.speed) * self.radius
        ))
        self.node.yaw(self.spin * dt)
