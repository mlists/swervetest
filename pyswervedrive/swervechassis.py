import math
import numpy as np
from magicbot import tunable
from wpilib import PIDController
from wpilib.interfaces import PIDOutput
from utilities.imu import IMU
from pyswervedrive.swervemodule import SwerveModule
from libswervedrive.icrestimator import ICREstimator


class SwerveChassis:
    WIDTH = 1
    LENGTH = 0.88

    imu: IMU
    module_a: SwerveModule
    module_b: SwerveModule
    module_c: SwerveModule
    module_d: SwerveModule

    # tunables here purely for debugging
    odometry_x = tunable(0)
    odometry_y = tunable(0)
    # odometry_theta = tunable(0)
    # odometry_x_vel = tunable(0)
    # odometry_y_vel = tunable(0)
    # odometry_z_vel = tunable(0)

    def __init__(self):
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.last_vx, self.last_vy = self.vx, self.vy
        self.field_oriented = True
        self.hold_heading = True
        self.momentum = False

    def setup(self):
        # Heading PID controller
        self.heading_pid_out = ChassisPIDOutput()
        self.heading_pid = PIDController(Kp=3.0, Ki=0.0, Kd=5.0,
                                         source=self.imu.getAngle,
                                         output=self.heading_pid_out,
                                         period=1/50)
        self.heading_pid.setInputRange(-math.pi, math.pi)
        self.heading_pid.setOutputRange(-2, 2)
        self.heading_pid.setContinuous()
        self.heading_pid.enable()
        self.modules = [self.module_a, self.module_b, self.module_c, self.module_d]

        self.odometry_x = 0
        self.odometry_y = 0
        self.odometry_theta = 0
        self.odometry_x_vel = 0
        self.odometry_y_vel = 0
        self.odometry_z_vel = 0

        self.A = np.array([
            [1, 0, 1],
            [0, 1, 1],
            [1, 0, 1],
            [0, 1, 1],
            [1, 0, 1],
            [0, 1, 1],
            [1, 0, 1],
            [0, 1, 1]
        ], dtype=float)

        # figure out the contribution of the robot's overall rotation about the
        # z axis to each module's movement, and encode that information in a
        # column vector
        # self.z_axis_adjustment = np.zeros((8, 1))
        alphas = []
        ls = []
        for i, module in enumerate(self.modules):
            module_dist = math.hypot(module.x_pos, module.y_pos)
            alphas.append(module_dist)
            module_angle = math.atan2(module.y_pos, module.x_pos)
            ls.append(module_angle)
            # self.z_axis_adjustment[i*2, 0] = -module_dist*math.sin(module_angle)
            # self.z_axis_adjustment[i*2+1, 0] = module_dist*math.cos(module_angle)
            self.A[i*2, 2] = -module_dist*math.sin(module_angle)
            self.A[i*2+1, 2] = module_dist*math.cos(module_angle)

            module.reset_encoder_delta()
            module.read_steer_pos()

        self.icre = ICREstimator(np.zeros(shape=(3, 1)), np.array(alphas),
                                 np.array(ls), np.zeros(shape=(4,)))

        # TODO: re-enable if we end up not using callback method
        self.imu.imu.ahrs.registerCallback(self.update_odometry)

    def set_heading_sp_current(self):
        self.set_heading_sp(self.imu.getAngle())

    def set_heading_sp(self, setpoint):
        self.heading_pid.setSetpoint(setpoint)
        self.heading_pid.enable()

    def heading_hold_on(self):
        self.set_heading_sp_current()
        self.heading_pid.reset()
        self.hold_heading = True

    def heading_hold_off(self):
        self.heading_pid.disable()
        self.hold_heading = False

    def on_enable(self):
        self.heading_hold_on()

        self.last_heading = self.imu.getAngle()
        self.odometry_updated = False

    def execute(self):

        pid_z = 0
        if self.hold_heading:
            if self.momentum and abs(self.imu.getHeadingRate()) < 0.005:
                self.momentum = False

            if self.vz not in [0.0, None]:
                self.momentum = True

            if not self.momentum:
                pid_z = self.heading_pid_out.output
            else:
                self.set_heading_sp_current()

        input_vz = 0
        if self.vz is not None:
            input_vz = self.vz
        if abs(pid_z) < 0.1 and math.hypot(self.vx, self.vy) < 0.01:
            pid_z = 0
        vz = input_vz + pid_z

        angle = self.imu.getAngle()

        # TODO: re-enable if we end up not using callback method
        # self.update_odometry()
        # self.odometry_updated = False  # reset for next timestep

        for module in self.modules:
            # Calculate the additional vx and vy components for this module
            # required to achieve our desired angular velocity
            vz_x = -module.dist*vz*math.sin(module.angle)
            vz_y = module.dist*vz*math.cos(module.angle)
            if self.field_oriented:
                vx, vy = self.robot_orient(self.vx, self.vy, angle)
            else:
                vx, vy = self.vx, self.vy
            module.set_velocity(vx+vz_x, vy+vz_y)

    def update_odometry(self, o, sensor_timestamp):
        # TODO: re-enable if we end up not using callback method
        # if self.odometry_updated:
        #     return
        heading = self.imu.getAngle()

        odometry_outputs = np.zeros((8, 1))
        velocity_outputs = np.zeros((8, 1))

        betas = []
        phi_dots = []
        for i, module in enumerate(self.modules):
            module.update_odometry()
            odometry_x, odometry_y = module.get_cartesian_delta()
            velocity_x, velocity_y = module.get_cartesian_vel()
            odometry_outputs[i*2, 0] = odometry_x
            odometry_outputs[i*2+1, 0] = odometry_y
            velocity_outputs[i*2, 0] = velocity_x
            velocity_outputs[i*2+1, 0] = velocity_y
            module.reset_encoder_delta()
            betas.append(module.measured_azimuth)
            phi_dots.append(module.wheel_angular_vel)

        q = np.array(betas)
        lambda_e = self.icre.estimate_lmda(q)
        print(lambda_e)

        vx, vy, vz = self.robot_movement_from_odometry(velocity_outputs, heading)
        delta_x, delta_y, delta_z = self.robot_movement_from_odometry(odometry_outputs, heading, z_vel=vz)

        self.odometry_x += delta_x
        self.odometry_y += delta_y
        self.odometry_x_vel = vx
        self.odometry_y_vel = vy
        self.odometry_z_vel = vz

        self.last_heading = heading

        self.odometry_updated = True

    def robot_movement_from_odometry(self, odometry_outputs, angle, z_vel=0):
        lstsq_ret = np.linalg.lstsq(self.A, odometry_outputs,
                                    rcond=None)
        x, y, theta = lstsq_ret[0].reshape(3)
        # TODO: re-enable if we move back to running in the same thread
        x_field, y_field = self.field_orient(x, y, angle + z_vel*(1/200))
        # x_field, y_field = self.field_orient(x, y, angle)
        return x_field, y_field, theta

    def set_velocity_heading(self, vx, vy, heading):
        """Set a translational velocity and a rotational orientation to achieve.

        Args:
            vx: (forward) component of the robot's desired velocity. In m/s.
            vy: (leftward) component of the robot's desired velocity. In m/s.
            heading: the heading the robot is to face.
        """
        self.vx = vx
        self.vy = vy
        self.vz = None
        self.set_heading_sp(heading)

    def set_inputs(self, vx: float, vy: float, vz: float, *, field_oriented: bool = True):
        """Set chassis vx, vy, and vz components of inputs.
        Args:
            vx: (forward) component of the robot's desired velocity. In m/s.
            vy: (leftward) component of the robot's desired velocity. In m/s.
            vz: The vz (counter-clockwise rotation) component of the robot's
                desired [angular] velocity. In radians/s.
            field_oriented: Whether the inputs are field or robot oriented.
        """
        self.last_vx, self.last_vy = self.vx, self.vy
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.field_oriented = field_oriented

    @staticmethod
    def robot_orient(vx, vy, heading):
        """Turn a vx and vy relative to the field into a vx and vy based on the
        robot.

        Args:
            vx: vx to robot orient
            vy: vy to robot orient
            heading: current heading of the robot. In radians CCW from +x axis.
        Returns:
            float: robot oriented vx speed
            float: robot oriented vy speed
        """
        oriented_vx = vx * math.cos(heading) + vy * math.sin(heading)
        oriented_vy = -vx * math.sin(heading) + vy * math.cos(heading)
        return oriented_vx, oriented_vy

    @staticmethod
    def field_orient(vx, vy, heading):
        """Turn a vx and vy relative to the robot into a vx and vy based on the
        field.

        Args:
            vx: vx to field orient
            vy: vy to field orient
            heading: current heading of the robot. In radians CCW from +x axis.
        Returns:
            float: field oriented vx speed
            float: field oriented vy speed
        """
        oriented_vx = vx * math.cos(heading) - vy * math.sin(heading)
        oriented_vy = vx * math.sin(heading) + vy * math.cos(heading)
        return oriented_vx, oriented_vy

    @property
    def position(self):
        return np.array([[self.odometry_x], [self.odometry_y]], dtype=float)

    @property
    def speed(self):
        return math.hypot(self.odometry_x_vel, self.odometry_y_vel)

    @property
    def all_aligned(self):
        return all(module.aligned for module in self.modules)


class ChassisPIDOutput(PIDOutput):
    def pidWrite(self, output):
        self.output = output

    def reset_odometry(self):
        """Reset all 3 odometry variables to a value of 0."""
        self.odometry_x = 0
        self.odometry_y = 0
        self.odometry_theta = 0
