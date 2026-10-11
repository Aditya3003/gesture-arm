import math
import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# ===== Tuning values =====
X_MIN, X_MAX = 0.2, 0.8          # comfortable hand range in the image (0..1)
Y_MIN, Y_MAX = 0.2, 0.8
CLOSE_BELOW = 1.0                # openness below this: close gripper
OPEN_ABOVE = 1.4                 # openness above this: open gripper
JOINT1_RANGE = (1.2, -1.2)       # base rotation: hand at left edge -> right edge
JOINT2_RANGE = (0.0, 1.2)        # bend: hand at top -> bottom
JOINT3_RANGE = (0.0, 1.2)
GRIPPER_OPEN, GRIPPER_CLOSED = 0.0, 0.027
SMOOTHING = 0.3                  # 0..1: lower = smoother but slower to react
RATE_HZ = 15                     # camera frames processed per second
MOVE_TIME = 0.3                  # seconds the controller gets to reach each target

ARM_JOINTS = ['joint1', 'joint2', 'joint3']
GRIPPER_JOINTS = ['gripper_finger_left_joint', 'gripper_finger_right_joint']

mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils


def distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


def scale(value, in_min, in_max, out_start, out_end):
    """Map value from [in_min, in_max] to [out_start, out_end], clamped to that range."""
    t = (value - in_min) / (in_max - in_min)
    t = max(0.0, min(1.0, t))
    return out_start + t * (out_end - out_start)


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

        self.arm = None                 # smoothed joint angles
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

            # Hand position -> target joint angles
            target = [
                scale(palm.x, X_MIN, X_MAX, *JOINT1_RANGE),
                scale(palm.y, Y_MIN, Y_MAX, *JOINT2_RANGE),
                scale(palm.y, Y_MIN, Y_MAX, *JOINT3_RANGE),
            ]

            # Smoothing: move only part of the way towards the new target each frame
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
            text = f'x={palm.x:.2f} y={palm.y:.2f} open={openness:.2f} gripper={state}'
            cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        # No hand detected: publish nothing, so the arm holds its last position

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