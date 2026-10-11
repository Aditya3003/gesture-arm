import math
import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# ===== Arm geometry (must match the URDF) =====
SHOULDER_HEIGHT = 0.345   # height of joint2 above the ground: base radius 0.045 + segment 0.3
LINK_A = 0.3              # joint2 -> joint3
LINK_B = 0.32             # joint3 -> middle of the fingers (hand_offset 0.25 + finger_height 0.07)
JOINT_LIMIT = 1.3

# ===== Hand -> target mapping =====
X_MIN, X_MAX = 0.2, 0.8            # hand left/right in the image
Y_MIN, Y_MAX = 0.2, 0.8            # hand up/down in the image
SIZE_MIN, SIZE_MAX = 0.08, 0.20    # hand size in the image: far from camera -> close to camera
JOINT1_RANGE = (-1.2, 1.2)         # base rotation: hand left -> right
HEIGHT_RANGE = (0.8, 0.3)         # gripper height (m): hand at top -> bottom
REACH_RANGE = (0.25, 0.55)         # gripper reach (m): hand far -> close to camera

# ===== Gripper =====
CLOSE_BELOW = 1.0
OPEN_ABOVE = 1.4
GRIPPER_OPEN, GRIPPER_CLOSED = 0.0, 0.027   # effort control: fully closed, stops on contact

# ===== Behaviour =====
SMOOTHING = 0.3
RATE_HZ = 15
MOVE_TIME = 0.3

ARM_JOINTS = ['joint1', 'joint2', 'joint3']
GRIPPER_JOINTS = ['gripper_finger_left_joint', 'gripper_finger_right_joint']

mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils


def distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


def scale(value, in_min, in_max, out_start, out_end):
    """Map value from [in_min, in_max] to [out_start, out_end], clamped."""
    t = (value - in_min) / (in_max - in_min)
    t = max(0.0, min(1.0, t))
    return out_start + t * (out_end - out_start)


def clamp(value, low, high):
    return max(low, min(high, value))


def inverse_kinematics(reach, height):
    """Joint2 and joint3 angles that put the gripper at (reach, height).
    Angles are measured from vertical; positive bends forward."""
    dx = reach
    dz = height - SHOULDER_HEIGHT
    d = math.hypot(dx, dz)
    d = clamp(d, abs(LINK_A - LINK_B) + 0.01, LINK_A + LINK_B - 0.01)  # keep the target reachable

    # Elbow angle from the law of cosines
    cos_q3 = (d * d - LINK_A ** 2 - LINK_B ** 2) / (2 * LINK_A * LINK_B)
    q3 = math.acos(clamp(cos_q3, -1.0, 1.0))

    # Shoulder angle: direction to the target, minus the offset caused by the bent elbow
    direction = math.atan2(dx, dz)
    offset = math.atan2(LINK_B * math.sin(q3), LINK_A + LINK_B * math.cos(q3))
    q2 = direction - offset

    return clamp(q2, -JOINT_LIMIT, JOINT_LIMIT), clamp(q3, -JOINT_LIMIT, JOINT_LIMIT)


def make_trajectory(joints, positions, seconds):
    msg = JointTrajectory()
    msg.joint_names = joints
    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
    msg.points = [point]
    return msg


class GestureControl(Node):

    def __init__(self):
        super().__init__('gesture_control')
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)

        self.cap = cv2.VideoCapture(0)
        self.hands = mp_hands.Hands(max_num_hands=1,
                                    min_detection_confidence=0.7,
                                    min_tracking_confidence=0.5)

        self.arm = None
        self.gripper = GRIPPER_OPEN

        self.timer = self.create_timer(1.0 / RATE_HZ, self.update)
        self.get_logger().info('Gesture control started. Press Ctrl+C in the terminal to stop.')

    def update(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('Could not read from webcam')
            return

        frame = cv2.flip(frame, 1)
        results = self.hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        if results.multi_hand_landmarks:
            hand = results.multi_hand_landmarks[0]
            mp_draw.draw_landmarks(frame, hand, mp_hands.HAND_CONNECTIONS)

            lm = hand.landmark
            palm = lm[9]
            hand_size = distance(lm[0], lm[9])
            openness = sum(distance(lm[0], lm[t]) for t in [8, 12, 16, 20]) / 4 / hand_size

            # Hand -> gripper target (base angle, reach, height) -> joint angles
            joint1 = scale(palm.x, X_MIN, X_MAX, *JOINT1_RANGE)
            height = scale(palm.y, Y_MIN, Y_MAX, *HEIGHT_RANGE)
            reach = scale(hand_size, SIZE_MIN, SIZE_MAX, *REACH_RANGE)
            joint2, joint3 = inverse_kinematics(reach, height)
            target = [joint1, joint2, joint3]

            # Smoothing
            if self.arm is None:
                self.arm = target
            else:
                self.arm = [a + SMOOTHING * (t - a) for a, t in zip(self.arm, target)]

            # Gripper with hysteresis
            if openness < CLOSE_BELOW:
                self.gripper = GRIPPER_CLOSED
            elif openness > OPEN_ABOVE:
                self.gripper = GRIPPER_OPEN

            self.arm_pub.publish(make_trajectory(ARM_JOINTS, self.arm, MOVE_TIME))
            self.gripper_pub.publish(make_trajectory(GRIPPER_JOINTS, [self.gripper, self.gripper], MOVE_TIME))

            state = 'CLOSED' if self.gripper == GRIPPER_CLOSED else 'OPEN'
            line1 = f'size={hand_size:.2f} open={openness:.2f} gripper={state}'
            line2 = f'reach={reach:.2f}m height={height:.2f}m'
            cv2.putText(frame, line1, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, line2, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow('Gesture control', frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        self.hands.close()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GestureControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()