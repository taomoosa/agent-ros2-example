"""FastAPI HTTP endpoints; the gateway handles all ROS2 communication."""

from fastapi import FastAPI
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse, Response

from .models import MoveArm, Gripper, Stop
from .protocol import BridgeError, validate_request


def create_app(gateway, config):
    app = FastAPI(title="ROS2 robotics server", version="0.1.0")

    @app.exception_handler(RequestValidationError)
    async def validation_error(_request, error):
        # Do not echo raw input: JSON NaN/Infinity cannot be serialized in the
        # default error response even though validation correctly rejects them.
        details = [{key: item[key] for key in ("loc", "msg", "type")} for item in error.errors()]
        return JSONResponse({"detail": details}, status_code=422)

    @app.exception_handler(BridgeError)
    async def bridge_error(_request, error):
        return JSONResponse(error.payload, status_code=error.status)

    async def request(operation, resource_id="", payload=None, timeout=5.0):
        payload = validate_request(config, operation, resource_id, payload or {})
        result = await gateway.request(operation, resource_id, payload, timeout)
        if result.status >= 400:
            return JSONResponse(result.payload, status_code=result.status)
        if operation == "camera":
            if result.status != 200 or result.content_type != "image/jpeg" or not result.data:
                raise BridgeError(502, "Invalid image response from robot bridge")
            return Response(result.data, media_type="image/jpeg", headers={
                "Cache-Control": "no-store",
                "X-Frame-Id": result.payload["frame_id"],
                "X-Stamp-Ns": str(result.payload["stamp_ns"]),
            })
        return JSONResponse(result.payload, status_code=result.status)

    @app.get("/v1/state")
    async def state():
        return await request("state")

    @app.get("/v1/cameras/{camera_id}/image")
    async def camera(camera_id: str):
        return await request("camera", camera_id, timeout=config.server.camera_timeout)

    @app.post("/v1/arms/{arm_id}/pose")
    async def move_arm(arm_id: str, body: MoveArm):
        return await request("move_arm", arm_id, body.model_dump(), body.duration + 5.0)

    @app.post("/v1/arms/{arm_id}/gripper")
    async def gripper(arm_id: str, body: Gripper):
        return await request("set_gripper", arm_id, body.model_dump())

    @app.post("/v1/stop")
    async def stop(body: Stop):
        return await request("stop", payload=body.model_dump())

    return app
