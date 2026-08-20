"""App shell: ROS spinner thread + Dear PyGui frame loop."""

from __future__ import annotations

import threading
from typing import Optional

import dearpygui.dearpygui as dpg
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

from eel_world_sim.gui.layout import build_ui, tick_ui
from eel_world_sim.gui.ros_node import WorldSimGuiNode


def _spin_ros(node: WorldSimGuiNode, stop: threading.Event) -> None:
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while not stop.is_set():
            if not rclpy.ok():
                break
            try:
                executor.spin_once(timeout_sec=0.1)
            except ExternalShutdownException:
                break
    except Exception:
        # SIGINT can invalidate the RCL context between ok() and spin_once
        # (RCLError: context is not valid).
        if stop.is_set() or not rclpy.ok():
            pass
        else:
            raise
    finally:
        try:
            executor.remove_node(node)
        except Exception:
            pass
        try:
            executor.shutdown()
        except Exception:
            pass


def run(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = WorldSimGuiNode()
    build_ui(node)

    stop = threading.Event()
    spinner = threading.Thread(target=_spin_ros, args=(node, stop), daemon=True)
    spinner.start()

    try:
        while dpg.is_dearpygui_running() and rclpy.ok():
            tick_ui(node)
            dpg.render_dearpygui_frame()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        stop.set()
        if dpg.is_dearpygui_running():
            dpg.stop_dearpygui()
        dpg.destroy_context()
        spinner.join(timeout=2.0)
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.try_shutdown()
