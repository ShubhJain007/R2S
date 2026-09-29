#!/usr/bin/env python3
"""Record a KUKA LBR Med7 trajectory for r2s_pipeline, straight off the FRI.

Run inside your ROS 2 environment (source AUTOKnee-server/install/setup.bash), with the
lbr_bringup driver already running:

    python3 record_lbr.py object_spam.npz --seconds 30
    python3 record_lbr.py baseline_empty.npz --seconds 30      # empty gripper -- do this FIRST

Saves an .npz that r2s_pipeline reads directly:
    t (N,)  q (N,7)  tau (N,7)  tau_ext (N,7)  sample_time  session_state
"""
import argparse, sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from lbr_fri_idl.msg import LBRState

# FRI session states (KUKA::FRI::ESessionState)
STATE_NAMES = {0: "IDLE", 1: "MONITORING_WAIT", 2: "MONITORING_READY", 3: "COMMANDING_WAIT", 4: "COMMANDING_ACTIVE"}


class Recorder(Node):
    def __init__(self, topic, seconds):
        super().__init__("r2s_recorder")
        self.seconds = seconds
        self.t, self.q, self.tau, self.tau_ext, self.sess = [], [], [], [], []
        self.sample_time = None
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=100)
        self.sub = self.create_subscription(LBRState, topic, self.cb, qos)
        self.get_logger().info(f"listening on {topic} ... move the robot now")

    def cb(self, m):
        t = m.time_stamp_sec + m.time_stamp_nano_sec * 1e-9
        if not self.t:
            self.t0 = t
            self.sample_time = m.sample_time
            self.get_logger().info(f"first sample: session={STATE_NAMES.get(m.session_state, m.session_state)}, "
                                   f"sample_time={m.sample_time*1000:.1f} ms ({1/m.sample_time:.0f} Hz)")
        self.t.append(t - self.t0)
        self.q.append(list(m.measured_joint_position))
        self.tau.append(list(m.measured_torque))
        self.tau_ext.append(list(m.external_torque))
        self.sess.append(m.session_state)
        n = len(self.t)
        if n % 500 == 0:
            self.get_logger().info(f"  {n} samples, {self.t[-1]:.1f} s")
        if self.t[-1] >= self.seconds:
            raise SystemExit


def main():
    p = argparse.ArgumentParser()
    p.add_argument("out"); p.add_argument("--topic", default="/lbr/state")
    p.add_argument("--seconds", type=float, default=30.0)
    a = p.parse_args()

    rclpy.init()
    node = Recorder(a.topic, a.seconds)
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        rclpy.shutdown()

    if len(node.t) < 100:
        sys.exit(f"only {len(node.t)} samples -- is the driver running and the FRI connected?")
    t = np.array(node.t); q = np.array(node.q); tau = np.array(node.tau); tau_ext = np.array(node.tau_ext)
    sess = np.array(node.sess)
    # keep only samples where the FRI session is live (torque is meaningless otherwise)
    ok = sess >= 2
    if ok.sum() < len(ok):
        print(f"dropping {len(ok)-ok.sum()} samples with session_state < MONITORING_READY")
        t, q, tau, tau_ext = t[ok] - t[ok][0], q[ok], tau[ok], tau_ext[ok]
    keep = np.diff(t, prepend=-1) > 0          # strictly increasing timestamps
    t, q, tau, tau_ext = t[keep], q[keep], tau[keep], tau_ext[keep]

    np.savez(a.out, t=t, q=q, tau=tau, tau_ext=tau_ext,
             sample_time=node.sample_time or np.median(np.diff(t)))
    rate = 1 / np.median(np.diff(t))
    print(f"\n-> {a.out}")
    print(f"   {len(t)} samples, {t[-1]-t[0]:.1f} s @ {rate:.0f} Hz")
    print(f"   |tau| per joint (Nm):     {np.round(np.abs(tau).max(0), 1)}")
    print(f"   |tau_ext| per joint (Nm): {np.round(np.abs(tau_ext).max(0), 2)}")
    qd = np.gradient(q, t, axis=0)
    print(f"   max |joint velocity|: {np.abs(qd).max():.2f} rad/s")
    if np.abs(tau_ext).max() < 0.5:
        print("   NOTE: external_torque is ~0 -- the controller may already be compensating the load,")
        print("         or no external force is present. measured_torque is the primary signal either way.")


if __name__ == "__main__":
    main()
