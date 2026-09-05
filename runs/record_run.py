"""Record an autonomous run to a file that survives the machine losing power.

Every line is one message, flushed as it arrives, so a run that ends with the
Pi going off still leaves everything up to that moment. Written under runs/ in
the repository rather than /tmp, which a reboot empties -- the first attempt at
this was lost exactly that way.

    python3 runs/record_run.py            # runs/run_<timestamp>.jsonl
    python3 runs/record_run.py mine.jsonl
"""
import json, os, sys, time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TOPICS = (
    ("auto", "/mdrobot_supervisor/auto_detail"),
    ("plate", "/mdrobot_plate_ocr/plate_detail"),
)


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        here, time.strftime("run_%Y%m%d_%H%M%S.jsonl"))
    out = open(path, "w")
    rclpy.init()
    node = Node("run_recorder")
    started = time.time()
    counts = {kind: 0 for kind, _ in TOPICS}

    def writer(kind):
        def cb(msg):
            counts[kind] += 1
            out.write(json.dumps(
                {"t": round(time.time() - started, 2), "k": kind, "d": msg.data}) + "\n")
            # Flushed per line: the point of this file is to survive the run,
            # not to be efficient about it.
            out.flush()
            os.fsync(out.fileno())
        return cb

    for kind, topic in TOPICS:
        node.create_subscription(String, topic, writer(kind), 50)

    def report():
        node.get_logger().info(
            f"{path}: " + "  ".join(f"{k} {v}" for k, v in counts.items()))
    node.create_timer(10.0, report)

    print(f"recording -> {path}   (Ctrl-C to stop)", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        out.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f"\n{path}: " + "  ".join(f"{k} {v}" for k, v in counts.items()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
