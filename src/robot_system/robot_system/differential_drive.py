def forward_kinematics(linear_velocity: float, angular_velocity: float, wheel):
    left_wheel_velocity = (linear_velocity - angular_velocity * wheel.track / 2.0) / wheel.radius
    right_wheel_velocity = (linear_velocity + angular_velocity * wheel.track / 2.0) / wheel.radius
    return left_wheel_velocity, right_wheel_velocity


def inverse_kinematics(left_velocity: float, right_velocity: float, wheel):
    linear_velocity = wheel.radius * (left_velocity + right_velocity) / 2.0
    angular_velocity = wheel.radius * (right_velocity - left_velocity) / wheel.track
    return linear_velocity, angular_velocity
