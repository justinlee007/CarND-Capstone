from lowpass import LowPassFilter
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858


class Controller(object):
    def __init__(self, params):
        self.yaw_controller = YawController(
            wheel_base=params.wheel_base,
            steer_ratio=params.steer_ratio,
            min_speed=params.min_speed,
            max_lat_accel=params.max_lat_accel,
            max_steer_angle=params.max_steer_angle)

        self.params = params
        self.pid = PID(kp=5, ki=0.5, kd=0.5, mn=params.decel_limit, mx=params.accel_limit)
        self.s_lpf = LowPassFilter(tau=3, ts=1)
        self.t_lpf = LowPassFilter(tau=3, ts=1)

    def reset(self):
        self.pid.reset()

    def control(self, twist_cmd, current_velocity, del_time):
        """
        Given an input TwistStamped message, velocity and delay, compute output pose.
        :param twist_cmd:
        :param current_velocity:
        :param del_time:
        :return: throttle, brake and steering values
        """
        lin_vel = abs(twist_cmd.twist.linear.x)
        ang_vel = twist_cmd.twist.angular.z
        vel_err = lin_vel - current_velocity.twist.linear.x

        if current_velocity.twist.linear.x <= 0.01 and twist_cmd.twist.linear.x <= 0.01:
            return 0, 100, 0

        next_steer = self.yaw_controller.get_steering(lin_vel, ang_vel, current_velocity.twist.linear.x)
        next_steer = self.s_lpf.filt(next_steer)

        acceleration = self.pid.step(vel_err, del_time)
        acceleration = self.t_lpf.filt(acceleration)

        if acceleration > 0.0:
            throttle = acceleration
            brake = 0.0
        else:
            throttle = 0.0
            deceleration = -acceleration

            if deceleration < self.params.brake_deadband:
                deceleration = 0.0

            brake = deceleration * (
                    self.params.vehicle_mass + self.params.fuel_capacity * GAS_DENSITY) * self.params.wheel_radius

        return throttle, brake, next_steer
