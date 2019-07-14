from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit,
                accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle, is_carla=True):

        # TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        self.is_carla = is_carla
        if self.is_carla:
            self.MAX_BRAKE = 1000.0
        else:
            self.MAX_BRAKE = 400.0

        kp = 0.3
        ki = 0.1
        kd = 0.0
        mn = 0.0
        mx = 0.2
        self.throttle_controller = PID(kp,ki,kd,mn,mx) 

        tau = 0.5
        ts = 0.2
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        self.last_time = rospy.get_time()


    def control(self, current_vel, dbw_enabled, target_vel, angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0, 0.0, 0.0
        
        current_vel = self.vel_lpf.filt(current_vel)

        steering = self.yaw_controller.get_steering(target_vel, angular_vel, current_vel)

        vel_error = target_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        vehilce_constant = self.vehicle_mass * self.wheel_radius # 418
        if self.is_carla:
            if current_vel < 1.0: 
                if vel_error <= 0:
                    throttle = 0.0
                    brake = self.MAX_BRAKE
                elif vel_error > 0 and vel_error <= 1.5:
                    throttle = 0.0
                    brake = min(self.MAX_BRAKE, (1.6 - 0.6 * vel_error) * vehilce_constant)
                else:
                    throttle = 0.1
                    brake = 0.0
            elif current_vel >= 1.0 and current_vel <= 2.0:
                if vel_error <= -1.0:
                    throttle = 0.0
                    brake = 0.8 * self.MAX_BRAKE
                elif vel_error > -1.0 and vel_error <= 1.0: # speed diff is [-1.5,1.5], use brake pedal to control speed
                    throttle = 0.0
                    brake = min(self.MAX_BRAKE, (1.5 - vel_error) * vehilce_constant)
                else:  # target speed is greater than vehicle speed by 1.5
                    throttle = self.throttle_controller.step(vel_error, sample_time)
                    brake = 0.0
            elif current_vel > 2.0:
                if vel_error < 0.0:
                    throttle = 0
                    decel = max(vel_error, self.decel_limit)
                    brake = min(self.MAX_BRAKE, (abs(decel) * self.vehicle_mass * self.wheel_radius)) 
                else:
                    throttle = self.throttle_controller.step(vel_error, sample_time)
                    brake = 0.0
        else:
            if current_vel < 1.0: 
                if vel_error <= 0:
                    throttle = 0.0
                    brake = self.MAX_BRAKE
                elif vel_error > 0 and vel_error <= 1.5:
                    throttle = 0.0
                    decel = max(vel_error, self.decel_limit)
                    brake = min(self.MAX_BRAKE, 0.5 * (abs(decel) * self.vehicle_mass * self.wheel_radius))
                else:
                    throttle = 0.3
                    brake = 0.0
            elif current_vel >= 1.0 and current_vel <= 2.0:
                if vel_error <= -1.0:
                    throttle = 0.0
                    brake = 0.8 * self.MAX_BRAKE
                elif vel_error > -1.0 and vel_error <= 1.0: # speed diff is [-1.5,1.5], use brake pedal to control speed
                    throttle = 0.0
                    decel = max(vel_error, self.decel_limit)
                    brake = min(self.MAX_BRAKE, 0.5 * (abs(decel) * self.vehicle_mass * self.wheel_radius))
                else:  # target speed is greater than vehicle speed by 1.5
                    throttle = self.throttle_controller.step(vel_error, sample_time)
                    brake = 0.0
            elif current_vel > 2.0:
                if vel_error < 0.0:
                    throttle = 0
                    decel = max(vel_error, self.decel_limit)
                    brake = min(self.MAX_BRAKE, (abs(decel) * self.vehicle_mass * self.wheel_radius)) 
                else:
                    throttle = self.throttle_controller.step(vel_error, sample_time)
                    brake = 0.0

        return throttle, brake, steering
