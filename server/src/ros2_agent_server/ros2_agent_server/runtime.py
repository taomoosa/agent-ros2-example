"""Launch both nodes together or run either node as a separate process."""

import argparse
import logging
import threading

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
import uvicorn

from .api import create_app
from .gateway import HttpGatewayNode
from .models import RobotConfig
from .robot_node import RobotBridgeNode


def spin_until_stopped(executor, stop_event):
    # All service waits yield rclpy Futures, so a single executor thread keeps
    # subscriptions and stop requests responsive without worker-pool teardown races.
    while not stop_event.is_set():
        executor.spin_once(timeout_sec=0.1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, help="Agent-compatible robot topology JSON")
    parser.add_argument("--role", choices=("both", "http", "robot"), default="both")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--diagnostic-log-level", choices=("DEBUG", "INFO", "WARNING"), default="INFO")
    args, ros_args = parser.parse_known_args()
    logging.basicConfig(level=logging.INFO)
    logging.getLogger("ros2_agent_server.http").setLevel(args.diagnostic_log_level)
    config = RobotConfig.load(args.config)
    context = Context()
    rclpy.init(args=ros_args, context=context, signal_handler_options=SignalHandlerOptions.NO)
    executor = SingleThreadedExecutor(context=context)
    stop_event = threading.Event()
    nodes = []
    spin_thread = None
    try:
        if args.role in {"both", "robot"}:
            nodes.append(RobotBridgeNode(config, context=context, cli_args=ros_args))
        if args.role in {"both", "http"}:
            gateway = HttpGatewayNode(config, context=context, cli_args=ros_args)
            nodes.append(gateway)
        for node in nodes:
            executor.add_node(node)
        spin_thread = threading.Thread(target=spin_until_stopped, args=(executor, stop_event), name="ros2-executor")
        spin_thread.start()
        if args.role == "robot":
            while spin_thread.is_alive():
                spin_thread.join(timeout=0.5)
        else:
            uvicorn.run(create_app(gateway, config), host=args.host, port=args.port)
    except KeyboardInterrupt:
        pass
    finally:
        for node in nodes:
            if isinstance(node, RobotBridgeNode):
                node.close_pending()
                node.wait_for_idle()
        stop_event.set()
        if spin_thread:
            spin_thread.join(timeout=5.0)
        executor.shutdown(timeout_sec=5.0)
        for node in nodes:
            node.destroy_node()
        context.try_shutdown()


if __name__ == "__main__":
    main()
