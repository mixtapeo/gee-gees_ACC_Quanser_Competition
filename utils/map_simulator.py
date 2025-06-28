import math

class CarSimulator2D:
    def __init__(self, x0=0.0, y0=0.0, theta0=0.0,
                 wheelbase=0.1, max_speed=0.5, max_steer=math.radians(30)):
        # State
        self.x, self.y, self.theta = x0, y0, theta0
        # Vehicle parameters
        self.L = wheelbase        # distance from rear axle to front axle [m]
        self.max_speed = max_speed      # throttle=±1 → v=±max_speed [m/s]
        self.max_steer  = max_steer     # steering=±1 → δ=±max_steer [rad]

    def update(self, throttle, steering, dt):
        # throttle and steering are in [-1..1]
        v     = throttle * self.max_speed
        delta = steering * self.max_steer
        # bicycle model
        if abs(delta) < 1e-6:
            # straight
            dx = v * math.cos(self.theta) * dt
            dy = v * math.sin(self.theta) * dt
            dtheta = 0.0
        else:
            R = self.L / math.tan(delta)
            dtheta = v / R * dt
            dx = R * (math.sin(self.theta + dtheta) - math.sin(self.theta))
            dy = R * (math.cos(self.theta) - math.cos(self.theta + dtheta))
        self.x     += dx
        self.y     += dy
        self.theta += dtheta
        return self.x, self.y, self.theta