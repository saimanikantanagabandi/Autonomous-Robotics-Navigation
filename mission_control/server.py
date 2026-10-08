"""
FastAPI Server & WebSocket Telemetry Gateway
============================================
Serves the Mission Control Tactical Dashboard, WebSocket telemetry stream,
teleoperation commands, and CSV benchmark exporter.
"""

import os
import asyncio
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse, Response, FileResponse
from pydantic import BaseModel
from typing import Optional

from mission_control.simulation_engine import SimulationEngine

app = FastAPI(title="Autonomous UGV Mission Control API")

# Global simulation instance
sim = SimulationEngine()

# Paths
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(BASE_DIR, "static")

# Ensure static directory exists
os.makedirs(STATIC_DIR, exist_ok=True)

# Mount static folder
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")

class TeleopRequest(BaseModel):
    v: float
    omega: float

class GoalRequest(BaseModel):
    x: float
    y: float

class FaultRequest(BaseModel):
    wheel_slip: Optional[bool] = None
    vslam_blackout: Optional[bool] = None
    imu_bias: Optional[bool] = None

@app.get("/", response_class=HTMLResponse)
async def get_index():
    index_file = os.path.join(STATIC_DIR, "index.html")
    if os.path.exists(index_file):
        return FileResponse(index_file)
    return HTMLResponse("<h1>Autonomous UGV Mission Control Initializing...</h1>")

@app.get("/api/map")
async def get_bunker_map():
    """Returns static bunker geometry, obstacles, and checkpoints."""
    return {
        "bounds": {
            "min_x": sim.bunker.min_x,
            "max_x": sim.bunker.max_x,
            "min_y": sim.bunker.min_y,
            "max_y": sim.bunker.max_y,
        },
        "walls": sim.bunker.wall_segments,
        "pillars": sim.bunker.pillars,
        "dropout_zone": sim.bunker.dropout_zone,
        "checkpoints": sim.bunker.default_checkpoints,
    }

@app.get("/api/state")
async def get_state():
    return sim.get_state_payload()

@app.post("/api/teleop")
async def post_teleop(cmd: TeleopRequest):
    sim.teleop(cmd.v, cmd.omega)
    return {"status": "ok", "mode": "MANUAL", "v": cmd.v, "omega": cmd.omega}

@app.post("/api/goal")
async def post_goal(goal: GoalRequest):
    sim.set_custom_goal(goal.x, goal.y)
    return {"status": "ok", "goal": {"x": goal.x, "y": goal.y}}

@app.post("/api/mission/{action}")
async def mission_action(action: str):
    if action == "pause":
        sim.toggle_pause()
    elif action == "reset":
        sim.reset_simulation()
    elif action == "patrol":
        sim.mode = "PATROL"
    else:
        raise HTTPException(status_code=400, detail="Invalid action")
    return {"status": "ok", "action": action, "is_paused": sim.is_paused, "mode": sim.mode}

@app.post("/api/faults")
async def post_faults(req: FaultRequest):
    sim.set_faults(req.wheel_slip, req.vslam_blackout, req.imu_bias)
    return {
        "status": "ok",
        "fault_wheel_slip": sim.fault_wheel_slip,
        "fault_vslam_blackout": sim.fault_vslam_blackout,
        "fault_imu_bias": sim.fault_imu_bias,
    }

@app.get("/api/export_csv")
async def export_csv():
    csv_data = sim.export_csv_data()
    return Response(
        content=csv_data,
        media_type="text/csv",
        headers={"Content-Disposition": "attachment; filename=ugv_mission_trajectory_log.csv"}
    )

@app.websocket("/ws/telemetry")
async def websocket_telemetry(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            # Send latest state payload at ~20 Hz
            payload = sim.get_state_payload()
            await websocket.send_json(payload)

            # Receive optional client commands without blocking
            try:
                data = await asyncio.wait_for(websocket.receive_json(), timeout=0.05)
                cmd_type = data.get("type")
                if cmd_type == "teleop":
                    sim.teleop(float(data.get("v", 0.0)), float(data.get("omega", 0.0)))
                elif cmd_type == "goal":
                    sim.set_custom_goal(float(data.get("x", 0.0)), float(data.get("y", 0.0)))
                elif cmd_type == "action":
                    action = data.get("action")
                    if action == "pause":
                        sim.toggle_pause()
                    elif action == "reset":
                        sim.reset_simulation()
                    elif action == "patrol":
                        sim.mode = "PATROL"
                elif cmd_type == "fault":
                    sim.set_faults(
                        data.get("wheel_slip"),
                        data.get("vslam_blackout"),
                        data.get("imu_bias")
                    )
            except asyncio.TimeoutError:
                pass
    except WebSocketDisconnect:
        pass
    except Exception:
        pass
