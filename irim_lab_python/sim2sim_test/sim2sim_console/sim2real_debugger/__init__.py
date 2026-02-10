"""Sim2Real Proprioception Debugger: policy/trajectory load, inference, ROS2 I/O, dashboard."""

import logging

from .dashboard import Sim2RealDebugger, run_app

LOG = logging.getLogger("sim2real_debugger")

__all__ = [
    "LOG",
    "Sim2RealDebugger",
    "run_app",
]
