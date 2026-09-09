"""Small structured diagnostics shared by ROS and HTTP boundaries."""

from contextvars import ContextVar
import json

request_id = ContextVar("request_id", default="")


def event(logger, name, *, level="debug", **fields):
    # Only callers' selected metadata is logged; never serialize request bodies.
    getattr(logger, level)(json.dumps(dict(event=name, **fields), default=str, allow_nan=False))


def require(condition, field, expected, actual):
    if not condition:
        # repr keeps NaN and arbitrary values out of JSON error structures.
        raise ValueError(f"{field}: expected {expected!r}; actual {actual!r}")
