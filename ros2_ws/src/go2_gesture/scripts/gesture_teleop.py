#!/usr/bin/env python3
"""Phase 9: gesture_teleop — live hand gestures → /cmd_vel.

Runs MediaPipe HandLandmarker + a TFLite keypoint classifier in the main
thread (OpenCV display requirement), smooths detections over a short history
window, and publishes geometry_msgs/Twist on /cmd_vel at a fixed rate.
Drop-in replacement for teleop_twist_keyboard.

Gesture → command mapping:
  palm / stop          → full stop
  stop_inverted / fist → forward
  call / mute          → backward
  one                  → turn left  (angular.z > 0)
  two_up               → turn right (angular.z < 0)
  (anything else)      → full stop

If no hand is detected for more than `no_hand_timeout` seconds the node
publishes a zero Twist (safety stop). Press ESC in the camera window to quit.

Run AFTER any robot scene is up (same pattern as teleop_twist_keyboard):
  ./run.sh shell
  source ~/ros2_ws/install/setup.bash
  ros2 launch go2_gesture gesture_teleop.launch.py
"""
import copy
import csv
import itertools
import os
import threading
import time
from collections import Counter, deque

import cv2 as cv
import mediapipe as mp
import numpy as np
import tensorflow as tf
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ament_index_python.packages import get_package_share_directory


# ---------------------------------------------------------------------------
# Gesture → velocity fractions  (multiplied by max speeds at publish time).
# Label strings must match keypoint_classifier_label.csv exactly.
# ---------------------------------------------------------------------------
GESTURE_CMD: dict = {
    # label          : (linear_x_frac, angular_z_frac)
    'palm'           : ( 0.0,  0.0),   # stop
    'stop'           : ( 0.0,  0.0),   # stop
    'stop_inverted'  : ( 1.0,  0.0),   # forward
    'fist'           : ( 1.0,  0.0),   # forward
    'call'           : (-1.0,  0.0),   # backward
    'mute'           : (-1.0,  0.0),   # backward
    'one'            : ( 0.0,  1.0),   # turn left
    'two_up'         : ( 0.0, -1.0),   # turn right
}
_STOP = (0.0, 0.0)   # fallback for any unrecognised label


# ---------------------------------------------------------------------------
# Inlined TFLite classifier (from gesture repo's model/keypoint_classifier/).
# Inlined here so there are no local-module import issues after colcon install.
# ---------------------------------------------------------------------------
class _KeyPointClassifier:
    def __init__(self, model_path: str, num_threads: int = 1):
        interp = tf.lite.Interpreter(model_path=model_path,
                                     num_threads=num_threads)
        interp.allocate_tensors()
        self._interp  = interp
        self._in_idx  = interp.get_input_details()[0]['index']
        self._out_idx = interp.get_output_details()[0]['index']

    def __call__(self, landmark_list: list) -> int:
        self._interp.set_tensor(
            self._in_idx, np.array([landmark_list], dtype=np.float32))
        self._interp.invoke()
        return int(np.argmax(np.squeeze(
            self._interp.get_tensor(self._out_idx))))


# ---------------------------------------------------------------------------
# Landmark helpers (ported from app.py preprocessing)
# ---------------------------------------------------------------------------
def _landmark_list(image: np.ndarray, landmarks) -> list:
    h, w = image.shape[:2]
    return [
        [min(int(lm.x * w), w - 1), min(int(lm.y * h), h - 1)]
        for lm in landmarks
    ]


def _preprocess(landmark_list: list) -> list:
    """Relative normalised coordinates — same preprocessing as training."""
    tmp = copy.deepcopy(landmark_list)
    bx, by = tmp[0]
    for pt in tmp:
        pt[0] -= bx
        pt[1] -= by
    flat = list(itertools.chain.from_iterable(tmp))
    mx = max(map(abs, flat)) or 1.0
    return [v / mx for v in flat]


# ---------------------------------------------------------------------------
# ROS 2 node — owns the publisher and the fixed-rate publish timer.
# Gesture state is written by the camera thread, read by the timer callback.
# CPython's GIL makes single attribute assignment/read atomic; no lock needed.
# ---------------------------------------------------------------------------
class GestureTeleop(Node):
    def __init__(self):
        super().__init__('gesture_teleop')

        self.declare_parameter('max_linear_speed',         0.4)
        self.declare_parameter('max_angular_speed',        0.8)
        self.declare_parameter('control_rate',             20.0)
        self.declare_parameter('camera_device',            0)
        self.declare_parameter('cap_width',                640)
        self.declare_parameter('cap_height',               480)
        self.declare_parameter('min_detection_confidence', 0.7)
        self.declare_parameter('min_tracking_confidence',  0.5)
        self.declare_parameter('gesture_history_len',      10)
        self.declare_parameter('no_hand_timeout',          0.5)

        p = self.get_parameter
        self.max_linear  = p('max_linear_speed').value
        self.max_angular = p('max_angular_speed').value
        self.cam_device  = int(p('camera_device').value)
        self.cap_width   = int(p('cap_width').value)
        self.cap_height  = int(p('cap_height').value)
        self.det_conf    = p('min_detection_confidence').value
        self.trk_conf    = p('min_tracking_confidence').value
        self.hist_len    = int(p('gesture_history_len').value)
        self.timeout     = p('no_hand_timeout').value

        pkg = get_package_share_directory('go2_gesture')
        self.landmarker_model = os.path.join(
            pkg, 'model', 'hand_landmarker', 'hand_landmarker.task')
        self.kp_model = os.path.join(
            pkg, 'model', 'keypoint_classifier', 'keypoint_classifier.tflite')
        self.kp_labels_path = os.path.join(
            pkg, 'model', 'keypoint_classifier', 'keypoint_classifier_label.csv')

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(1.0 / p('control_rate').value, self._publish)

        # Written by camera thread (main), read by timer callback (spin thread).
        self._gesture   = 'stop'
        self._last_seen = 0.0    # wall-clock time of the last detection

        self.get_logger().info(
            f'gesture_teleop up — '
            f'v_max={self.max_linear} m/s  w_max={self.max_angular} rad/s  '
            f'timeout={self.timeout}s'
        )

    def update_gesture(self, label: str) -> None:
        """Called from the camera (main) thread each time a stable gesture is seen."""
        self._gesture   = label
        self._last_seen = time.time()

    def _publish(self) -> None:
        twist = Twist()
        if time.time() - self._last_seen > self.timeout:
            # No hand visible within the timeout window → safety stop.
            self._cmd_pub.publish(twist)
            return
        lx, wz = GESTURE_CMD.get(self._gesture, _STOP)
        twist.linear.x  = lx * self.max_linear
        twist.angular.z = wz * self.max_angular
        self._cmd_pub.publish(twist)


# ---------------------------------------------------------------------------
# Camera + detection loop — runs on the MAIN thread (cv.imshow requirement).
# ---------------------------------------------------------------------------
def _camera_loop(node: GestureTeleop) -> None:
    with open(node.kp_labels_path, encoding='utf-8-sig') as f:
        labels = [row[0] for row in csv.reader(f)]

    clf = _KeyPointClassifier(node.kp_model)

    options = mp_vision.HandLandmarkerOptions(
        base_options=mp_python.BaseOptions(
            model_asset_path=node.landmarker_model),
        running_mode=mp_vision.RunningMode.VIDEO,
        num_hands=1,
        min_hand_detection_confidence=node.det_conf,
        min_tracking_confidence=node.trk_conf,
    )
    landmarker = mp_vision.HandLandmarker.create_from_options(options)

    cap = cv.VideoCapture(node.cam_device)
    cap.set(cv.CAP_PROP_FRAME_WIDTH,  node.cap_width)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, node.cap_height)
    if not cap.isOpened():
        node.get_logger().error(
            f'Cannot open camera device {node.cam_device}. '
            'Verify /dev/video0 is in docker-compose.yml devices.')
        rclpy.shutdown()
        return

    history: deque = deque(maxlen=node.hist_len)
    current_label = 'stop'

    while rclpy.ok():
        key = cv.waitKey(1)
        if key == 27:          # ESC → clean shutdown
            rclpy.shutdown()
            break

        ret, frame = cap.read()
        if not ret:
            node.get_logger().warn('Camera read failed — retrying',
                                   throttle_duration_sec=2.0)
            time.sleep(0.05)
            continue

        frame = cv.flip(frame, 1)   # mirror so left/right feel natural
        debug = frame.copy()

        rgb    = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        result = landmarker.detect_for_video(
            mp_img, time.time_ns() // 1_000_000)

        if result.hand_landmarks:
            pts_list  = _landmark_list(frame, result.hand_landmarks[0])
            processed = _preprocess(pts_list)
            sign_id   = clf(processed)

            history.append(labels[sign_id])
            current_label = Counter(history).most_common(1)[0][0]
            node.update_gesture(current_label)

            # Draw bounding box + gesture label in green
            pts = np.array(pts_list, dtype=np.int32)
            x, y, w, h = cv.boundingRect(pts)
            cv.rectangle(debug, (x, y), (x + w, y + h), (0, 220, 0), 2)
            cv.putText(debug, current_label, (x, max(y - 8, 20)),
                       cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 220, 0), 2,
                       cv.LINE_AA)
        else:
            # Hand left frame — reset history so the next gesture is clean.
            history.clear()
            current_label = 'stop'
            cv.putText(debug, 'no hand — stopped', (10, 60),
                       cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 80, 255), 2,
                       cv.LINE_AA)

        # HUD: actual velocities being published
        lx, wz = GESTURE_CMD.get(current_label, _STOP)
        cv.putText(debug,
                   f'vx={lx * node.max_linear:+.2f}  '
                   f'wz={wz * node.max_angular:+.2f}',
                   (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7,
                   (255, 255, 255), 2, cv.LINE_AA)

        cv.imshow('Gesture Teleop', debug)

    landmarker.close()
    cap.release()
    cv.destroyAllWindows()


def main() -> None:
    rclpy.init()
    node = GestureTeleop()

    # ROS spin (timers + DDS) runs in a background daemon thread so the main
    # thread can own the OpenCV display loop (cv.imshow is not thread-safe
    # across threads on most backends).
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        _camera_loop(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
