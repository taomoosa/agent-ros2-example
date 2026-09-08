"""CLI smoke test using an independent server process and a real HTTP socket."""

import asyncio
import os
import signal
import socket
import sys
import unittest

import httpx

from helpers import ROOT


class RuntimeTest(unittest.IsolatedAsyncioTestCase):
    async def test_cli_starts_both_nodes_and_shuts_down_cleanly(self):
        with socket.socket() as reservation:
            reservation.bind(("127.0.0.1", 0))
            port = reservation.getsockname()[1]
        env = dict(os.environ, ROS_DOMAIN_ID="174", ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
                   ROS_LOG_DIR="/tmp/agent-ros2-server-cli-logs")
        process = await asyncio.create_subprocess_exec(
            sys.executable, "-m", "ros2_agent_server", "--config",
            str(ROOT / "server/configs/single_arm.json"), "--port", str(port),
            env=env, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
        try:
            async with httpx.AsyncClient(base_url=f"http://127.0.0.1:{port}", timeout=1) as client:
                deadline = asyncio.get_running_loop().time() + 10
                while True:
                    self.assertIsNone(process.returncode, "Server exited before starting HTTP")
                    try:
                        response = await client.get("/openapi.json")
                        if response.status_code == 200:
                            break
                    except httpx.TransportError:
                        pass
                    if asyncio.get_running_loop().time() >= deadline:
                        self.fail("HTTP server did not start")
                    await asyncio.sleep(0.05)
                self.assertIn("/v1/arms/{arm_id}/pose", response.json()["paths"])
                response = await client.get("/v1/state")
                self.assertEqual(503, response.status_code)
                self.assertFalse(response.json()["success"])
        finally:
            if process.returncode is None:
                process.send_signal(signal.SIGINT)
            try:
                stdout, stderr = await asyncio.wait_for(process.communicate(), timeout=10)
            except asyncio.TimeoutError:
                process.kill()
                await process.communicate()
                raise
        self.assertEqual(0, process.returncode, stderr.decode())
        self.assertNotIn(b"Traceback", stderr)
        self.assertNotIn(b"exception was never retrieved", stderr)
