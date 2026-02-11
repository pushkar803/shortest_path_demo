#!/usr/bin/env python3
"""
execute_route.py

Small runner that uses the route planner in `short/app.py` to execute a plan.
Provides a simulation mode (default) and a hardware stub you can replace with
real motor commands. Designed to be safe to run on your development machine.
"""
from __future__ import annotations

import math
import time
import argparse
from typing import List, Tuple, Dict, Any, Optional
try:
    from HW_MechDog import MechDog
except Exception:
    MechDog = None
try:
    import Hiwonder_IIC
except Exception:
    Hiwonder_IIC = None

# Import planner and simple robot stubs from app.py (it's safe because app.run
# is guarded by `if __name__ == "__main__":` in that file).
from app import plan_route, NODE_POS, DEFAULT_START_NODE, robot_take_photo, EDGES


class RobotExecutor:
    def __init__(
        self,
        mode: str = "simulate",
        speed_m_per_s: float = 0.15,
        hw_speed_m_per_s: float = 0.15,
        kp_turn: float = 1.5,
        kp_drive: float = 1.2,
        forward_cmd: int = 120,
        turn_max: int = 60,
        turn_cmd_mag: int = 50,
    ):
        """
        mode: "simulate" or "hardware". Hardware path uses `send_motor_command`
              which is a placeholder you should implement for your robot.
        speed_m_per_s: used in simulation to estimate travel time (meters/sec).
        hw_speed_m_per_s: expected hardware forward speed in meters/sec (used for timing).
        """
        if mode not in ("simulate", "hardware"):
            raise ValueError("mode must be 'simulate' or 'hardware'")
        self.mode = mode
        self.speed = float(speed_m_per_s)
        # hardware forward speed in meters/sec
        self.hw_speed_m_per_s = float(hw_speed_m_per_s)
        # heading in degrees (absolute, 0 = +x). Assume initial heading 0.
        self.heading_deg = 0.0
        # control params (tunable)
        self.Kp_turn = float(kp_turn)
        self.Kp_drive = float(kp_drive)
        self.FORWARD_CMD = int(forward_cmd)
        self.TURN_MAX = int(turn_max)
        self.TURN_CMD_MAG = int(turn_cmd_mag)
        # Hardware MechDog instance (only created in hardware mode)
        self.mechdog = None
        if self.mode == "hardware":
            if MechDog is None:
                raise RuntimeError("HW_MechDog library not available; cannot run in hardware mode")
            # instantiate with default params; tutorials often use MechDog()
            self.mechdog = MechDog()
        # Track current position as (x,y) in same coordinate space as NODE_POS.
        # We'll initialize to the start node when run() is called.
        self.pos: Tuple[float, float] = (0.0, 0.0)
        # build edge weight lookup from EDGES (weights are in meters)
        self.edge_weight: Dict[Tuple[str, str], float] = {}
        for u, v, w in EDGES:
            self.edge_weight[(u, v)] = float(w)
            self.edge_weight[(v, u)] = float(w)

    # ----- Movement primitives -----
    def send_motor_command(self, distance: float, heading_deg: float) -> None:
        """
        Closed-loop motor command using IMU (heading) and sonar (safety).
        distance: meters
        heading_deg: absolute heading (degrees, 0 = +x)
        """
        if self.mechdog is None:
            raise RuntimeError("MechDog instance not initialized")

        def _norm_angle(a: float) -> float:
            a = (a + 180) % 360 - 180
            return a

        # Initialize I2C devices on-demand
        if self.mode == "hardware" and Hiwonder_IIC is not None and not getattr(self, "_iic_initialized", False):
            try:
                self._iic = Hiwonder_IIC.IIC(1)
                try:
                    self.sonar = Hiwonder_IIC.I2CSonar(self._iic)
                except Exception:
                    self.sonar = None
                try:
                    self.imu = Hiwonder_IIC.MPU(self._iic)
                except Exception:
                    try:
                        self.imu = Hiwonder_IIC.MPU()
                    except Exception:
                        self.imu = None
            except Exception:
                self._iic = None
                self.imu = None
                self.sonar = None
            self._iic_initialized = True

        def read_yaw():
            if not getattr(self, "imu", None):
                return self.heading_deg
            try:
                ang = self.imu.read_angle()
                if isinstance(ang, (list, tuple)):
                    if len(ang) >= 3:
                        return float(ang[2])
                    return float(ang[0])
                return float(ang)
            except Exception:
                return self.heading_deg

        # Rotate to heading using IMU if available
        if getattr(self, "imu", None) is not None:
            err = _norm_angle(heading_deg - read_yaw())
            while abs(err) > 5.0:
                turn_cmd = int(max(-self.TURN_MAX, min(self.TURN_MAX, self.Kp_turn * err)))
                self.mechdog.move(0, turn_cmd)
                time.sleep(0.08)
                err = _norm_angle(heading_deg - read_yaw())
            self.mechdog.move(0, 0)
            time.sleep(0.05)
            self.heading_deg = _norm_angle(read_yaw())
        else:
            # fallback open-loop rotation using configured TURN_CMD_MAG
            err = _norm_angle(heading_deg - self.heading_deg)
            TURN_RATE_DEG_PER_SEC = 90.0 * (self.TURN_CMD_MAG / 120.0)
            while abs(err) > 8.0:
                turn_sign = 1 if err > 0 else -1
                self.mechdog.move(0, int(turn_sign * self.TURN_CMD_MAG))
                dt = 0.15
                time.sleep(dt)
                self.heading_deg = _norm_angle(self.heading_deg + turn_sign * TURN_RATE_DEG_PER_SEC * dt)
                err = _norm_angle(heading_deg - self.heading_deg)
            self.mechdog.move(0, 0)
            time.sleep(0.05)

        # Forward with heading correction; use sonar for safety stop
        duration = max(0.0, float(distance) / max(1e-6, self.hw_speed_m_per_s))
        start_t = time.time()
        step_dt = 0.08
        safety_stop_cm = 15.0
        while (time.time() - start_t) < duration:
            if getattr(self, "imu", None) is not None:
                cur_yaw = read_yaw()
                err_h = _norm_angle(heading_deg - cur_yaw)
                turn_cmd = int(max(-self.TURN_MAX, min(self.TURN_MAX, self.Kp_drive * err_h)))
            else:
                turn_cmd = 0

            self.mechdog.move(int(self.FORWARD_CMD), int(turn_cmd))
            time.sleep(step_dt)

            # sonar safety
            if getattr(self, "sonar", None) is not None:
                try:
                    dcm = float(self.sonar.getDistance())
                    if dcm > 0 and dcm < safety_stop_cm:
                        break
                except Exception:
                    pass

        self.mechdog.move(0, 0)
        self.heading_deg = heading_deg

    def _simulate_move(self, distance_m: float, heading_deg: float) -> None:
        # Simple time-based simulation using meters
        eta = max(0.0, distance_m / max(1e-6, self.speed))
        print(f"[SIM] Moving {distance_m:.2f} m at heading {heading_deg:.1f}° (eta {eta:.2f}s)...")
        time.sleep(min(eta, 2.0))  # clamp sleep so tests are quick

    def move_to_node(self, node_id: str, distance_m: Optional[float] = None) -> None:
        if node_id not in NODE_POS:
            raise ValueError(f"Unknown node id: {node_id}")
        target = NODE_POS[node_id]
        cur_x, cur_y = self.pos
        tx, ty = target
        dx, dy = tx - cur_x, ty - cur_y
        # If distance in meters provided, use it (planner leg weights are meters).
        if distance_m is None:
            # fallback: compute euclidean in NODE_POS units; assume 1 unit == 1 meter
            distance_m = math.hypot(dx, dy)
        heading_rad = math.atan2(dy, dx)
        heading_deg = math.degrees(heading_rad)

        if self.mode == "simulate":
            self._simulate_move(distance_m, heading_deg)
        else:
            # hardware mode: call the placeholder
            self.send_motor_command(distance_m, heading_deg)

        # update position to exact node coordinates (assume perfect arrival)
        self.pos = (tx, ty)

    # ----- High level execution -----
    def execute_plan(self, start: str, selected: List[str]) -> Dict[str, Any]:
        plan = plan_route(start, selected)
        print(f"Planned visit order: {plan.visit_order}")
        print(f"Expanded route: {plan.expanded_route}")
        photos = []

        # Initialize executor position to start node
        if start in NODE_POS:
            self.pos = NODE_POS[start]
        else:
            # fallback: (0,0)
            self.pos = (0.0, 0.0)

        # Walk expanded route node-by-node and take photos at selected nodes.
        # Use edge weights from planner (meters) when available.
        expanded = plan.expanded_route
        for idx in range(1, len(expanded)):
            cur = expanded[idx - 1]
            nxt = expanded[idx]
            # weight from edge map if present, else fallback to euclidean using NODE_POS
            distance_m = self.edge_weight.get((cur, nxt))
            if distance_m is None:
                # compute euclidean distance in NODE_POS units (assumed meters)
                cx, cy = NODE_POS[cur]
                nx, ny = NODE_POS[nxt]
                distance_m = math.hypot(nx - cx, ny - cy)

            print(f"[EXEC] Hop {idx}: moving {cur} -> {nxt} ({distance_m:.2f} m)")
            self.move_to_node(nxt, distance_m=distance_m)
            # If this node is one of the selected destinations, take a photo.
            if nxt in plan.selected:
                fn = robot_take_photo(nxt)
                photos.append({"location": nxt, "file": fn})

        return {"status": "ok", "photos": photos, "new_start": plan.expanded_route[-1] if plan.expanded_route else start}


def parse_selected(s: str) -> List[str]:
    if not s:
        return []
    return [x.strip() for x in s.split(",") if x.strip()]


def main():
    p = argparse.ArgumentParser(description="Execute a MechDog route using the planner in short/app.py")
    p.add_argument("--start", "-s", default=DEFAULT_START_NODE, help="Start node id")
    p.add_argument("--selected", "-l", default="", help="Comma-separated selected node ids (e.g. B,C,E)")
    p.add_argument("--mode", "-m", choices=["simulate", "hardware"], default="simulate", help="Execution mode")
    p.add_argument("--speed", type=float, default=0.15, help="Simulated speed units/sec")
    p.add_argument("--hw-speed", type=float, default=0.15, help="Hardware forward speed in meters/sec")
    p.add_argument("--kp-turn", type=float, default=1.5, help="Heading P gain for rotation")
    p.add_argument("--kp-drive", type=float, default=1.2, help="Heading P gain for driving correction")
    p.add_argument("--forward-cmd", type=int, default=120, help="Forward move command magnitude passed to mechdog.move()")
    p.add_argument("--turn-max", type=int, default=60, help="Max turn command magnitude")
    p.add_argument("--turn-cmd-mag", type=int, default=50, help="Fallback turn magnitude for open-loop rotation")
    args = p.parse_args()

    selected = parse_selected(args.selected)
    execr = RobotExecutor(
        mode=args.mode,
        speed_m_per_s=args.speed,
        hw_speed_m_per_s=getattr(args, "hw_speed", 0.15),
        kp_turn=getattr(args, "kp_turn", 1.5),
        kp_drive=getattr(args, "kp_drive", 1.2),
        forward_cmd=getattr(args, "forward_cmd", 120),
        turn_max=getattr(args, "turn_max", 60),
        turn_cmd_mag=getattr(args, "turn_cmd_mag", 50),
    )
    result = execr.execute_plan(args.start, selected)
    print("Execution result:")
    print(result)


if __name__ == "__main__":
    main()

