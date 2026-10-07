import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

ARM_JOINTS = ['joint1', 'joint2', 'joint3']
GRIPPER_JOINTS = ['gripper_finger_left_joint', 'gripper_finger_right_joint']
OPEN, CLOSED = 0.0, 0.027

# Each step: (name, [joint1, joint2, joint3] in radians, gripper position)
SEQUENCE = [
    ('home',    [0.0, 0.0, 0.0],   OPEN),
    ('reach',   [0.8, 0.9, 1.0],   OPEN),
    ('grab',    [0.8, 0.9, 1.0],   CLOSED),
    ('lift',    [0.8, 0.3, 0.6],   CLOSED),
    ('turn',    [-0.8, 0.3, 0.6],  CLOSED),
    ('place',   [-0.8, 0.9, 1.0],  CLOSED),
    ('release', [-0.8, 0.9, 1.0],  OPEN),
    ('back',    [-0.8, 0.3, 0.6],  OPEN),
]


def make_trajectory(joints, positions, seconds):
    msg = JointTrajectory()
    msg.joint_names = joints
    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = Duration(sec=seconds)
    msg.points = [point]
    return msg


class ArmDemo(Node):

    def __init__(self):
        super().__init__('arm_demo')
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.step = 0
        self.timer = self.create_timer(2.5, self.next_step)  # one step every 2.5 s

    def next_step(self):
        name, arm_positions, gripper = SEQUENCE[self.step]
        self.arm_pub.publish(make_trajectory(ARM_JOINTS, arm_positions, 2))
        self.gripper_pub.publish(make_trajectory(GRIPPER_JOINTS, [gripper, gripper], 1))
        self.get_logger().info(f'Step: {name}')
        self.step = (self.step + 1) % len(SEQUENCE)  # loop forever


def main(args=None):
    rclpy.init(args=args)
    node = ArmDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()