"""Position controller driven by the identified unloaded motor model.

Controller input is position error in mm and measured axis speed in mm/s.
Its output is a motor-voltage command.  MotorModel converts voltage to the
identified, first-order linear velocity response in m/s, which is sent through
gz_ros2_control's JointGroupVelocityController to the three simulated joints.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from gantry_control.motor_model import MotorModel
from gantry_control.sensor_model import SensorModel

AXES = ('x', 'y', 'z')
JOINT_NAMES = {
    'x': 'dof_linear_bearing_2',
    'y': 'dof_linear_bearing',
    'z': 'joint_z',
}


class GantryControlNode(Node):
    def __init__(self):
        super().__init__('gantry_control_node')
        self.declare_parameter('Ts', 0.005)
        self._setpoint = {ax: 0.0 for ax in AXES}  # mm
        self._true_pos_m = {ax: 0.0 for ax in AXES}
        self._true_vel_mps = {ax: 0.0 for ax in AXES}
        self._joint_idx = None
        self._controllers = {}
        self._motors = {}
        self._sensors = {}
        self._limits = {}
        for ax in AXES:
            self.declare_parameter(f'control.{ax}.kp', 0.0)
            self.declare_parameter(f'control.{ax}.kd', 0.0)
            self.declare_parameter(f'control.{ax}.voltage_limit', 1.0)
            self.declare_parameter(f'motor.{ax}.b', [1.0])
            self.declare_parameter(f'motor.{ax}.a', [1.0])
            self.declare_parameter(f'motor.{ax}.deadband', 0.0)
            self.declare_parameter(f'motor.{ax}.velocity_min', -0.1)
            self.declare_parameter(f'motor.{ax}.velocity_max', 0.1)
            self.declare_parameter(f'sensor.{ax}.b', [1000.0])
            self.declare_parameter(f'sensor.{ax}.a', [1.0])
            self.declare_parameter(f'sensor.{ax}.noise_std', 0.0)
            self.declare_parameter(f'sensor.{ax}.quantization', 0.0)
            self.declare_parameter(f'sensor.{ax}.delay_samples', 0)
            self._controllers[ax] = (
                self.get_parameter(f'control.{ax}.kp').value,
                self.get_parameter(f'control.{ax}.kd').value,
            )
            self._limits[ax] = self.get_parameter(f'control.{ax}.voltage_limit').value
            self._motors[ax] = MotorModel(
                self.get_parameter(f'motor.{ax}.b').value,
                self.get_parameter(f'motor.{ax}.a').value,
                deadband=self.get_parameter(f'motor.{ax}.deadband').value,
                velocity_min=self.get_parameter(f'motor.{ax}.velocity_min').value,
                velocity_max=self.get_parameter(f'motor.{ax}.velocity_max').value,
            )
            self._sensors[ax] = SensorModel(
                self.get_parameter(f'sensor.{ax}.b').value,
                self.get_parameter(f'sensor.{ax}.a').value,
                noise_std=self.get_parameter(f'sensor.{ax}.noise_std').value,
                quantization=self.get_parameter(f'sensor.{ax}.quantization').value,
                delay_samples=self.get_parameter(f'sensor.{ax}.delay_samples').value,
            )
        self.create_subscription(Float64MultiArray, '/gantry/setpoint', self._on_setpoint, 10)
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)
        self._velocity_pub = self.create_publisher(
            Float64MultiArray, '/velocity_controller/commands', 10)
        self._diag_pubs = {
            name: self.create_publisher(Float64MultiArray, f'/gantry/diag/{name}', 10)
            for name in ('setpoint', 'position_true', 'position_sensed',
                         'voltage_cmd', 'velocity_cmd')
        }
        self.create_timer(self.get_parameter('Ts').value, self._on_timer)
        self.get_logger().info('motor-ID position controller ready; axes=%s' % (AXES,))

    def _on_setpoint(self, msg: Float64MultiArray):
        if len(msg.data) == len(AXES):
            self._setpoint = dict(zip(AXES, msg.data))
        else:
            self.get_logger().warn('setpoint requires [x, y, z] in mm; ignored')

    def _on_joint_states(self, msg: JointState):
        if self._joint_idx is None:
            try:
                self._joint_idx = {ax: msg.name.index(name) for ax, name in JOINT_NAMES.items()}
            except ValueError:
                return
        for ax in AXES:
            i = self._joint_idx[ax]
            self._true_pos_m[ax] = msg.position[i]
            self._true_vel_mps[ax] = msg.velocity[i]

    def _on_timer(self):
        if self._joint_idx is None:
            return
        sensed = {}
        volts = {}
        velocities = {}
        for ax in AXES:
            # Sensor model scales the ROS joint position (m) to controller mm.
            sensed[ax] = self._sensors[ax].update(self._true_pos_m[ax])
            error_mm = self._setpoint[ax] - sensed[ax]
            speed_mm_s = 1000.0 * self._true_vel_mps[ax]
            kp, kd = self._controllers[ax]
            volts[ax] = max(-self._limits[ax], min(
                self._limits[ax], kp * error_mm - kd * speed_mm_s))
            velocities[ax] = self._motors[ax].update(volts[ax])
        self._velocity_pub.publish(Float64MultiArray(data=[velocities[ax] for ax in AXES]))
        diagnostics = {
            'setpoint': [self._setpoint[ax] for ax in AXES],
            'position_true': [1000.0 * self._true_pos_m[ax] for ax in AXES],
            'position_sensed': [sensed[ax] for ax in AXES],
            'voltage_cmd': [volts[ax] for ax in AXES],
            'velocity_cmd': [velocities[ax] for ax in AXES],
        }
        for name, values in diagnostics.items():
            self._diag_pubs[name].publish(Float64MultiArray(data=values))


def main(args=None):
    rclpy.init(args=args)
    node = GantryControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
