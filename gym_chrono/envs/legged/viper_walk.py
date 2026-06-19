# =============================================================================
# Chrono 10 + Gymnasium: Viper rover walk (scaling surrogate).
# Not the Chrono-Gymnasium paper R^45 velocity-tracking task; obs/act dims
# match legacy quadruped_walk-style stacks (18 / 12) for tooling compatibility.
# =============================================================================
from __future__ import annotations

import math
import os
from pathlib import Path

import gymnasium as gym
import numpy as np
import pychrono as chrono
import pychrono.robot as robot


def _set_chrono_data_path() -> None:
    """Set Chrono data directory without importing optional Chrono modules."""
    data = os.environ.get("CHRONO_DATA_DIR")
    if not data:
        conda = os.environ.get("CONDA_PREFIX")
        if conda:
            cand = os.path.join(conda, "share", "chrono", "data", "")
            if os.path.isdir(cand):
                data = cand
    if not data:
        # Repo layout: optional gym_chrono/envs/data per upstream README
        envs_dir = Path(__file__).resolve().parents[1]
        cand = envs_dir / "data"
        if cand.is_dir():
            data = str(cand) + os.sep
    if data:
        chrono.SetChronoDataPath(data)


class ChronoViperWalkEnv(gym.Env):
    """Viper rover forward-drive on NSC + Bullet; headless (render_mode=None)."""

    metadata = {"render_modes": [None]}

    def __init__(self, render_mode=None):
        super().__init__()
        self.action_space = gym.spaces.Box(
            low=-3.0, high=3.0, shape=(12,), dtype=np.float64
        )
        self.observation_space = gym.spaces.Box(
            low=-30.0, high=30.0, shape=(18,), dtype=np.float64
        )
        self.render_mode = render_mode

        self._step_size = 5e-4
        self._control_frequency = 20
        self._steps_per_control = max(
            1, round(1.0 / (self._step_size * self._control_frequency))
        )
        self._max_time = 50.0

        self.system: chrono.ChSystemNSC | None = None
        self.rover: robot.Viper | None = None
        self.driver: robot.ViperDCMotorControl | None = None
        self._prev_x = 0.0

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self._prev_x = 0.0

        _set_chrono_data_path()

        self.system = chrono.ChSystemNSC()
        self.system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
        self.system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
        chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
        chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)

        ground_mat = chrono.ChContactMaterialNSC()
        ground = chrono.ChBodyEasyBox(20, 20, 0.1, 1000, True, True, ground_mat)
        ground.SetPos(chrono.ChVector3d(0, 0, -0.05))
        ground.SetFixed(True)
        try:
            if ground.GetVisualShape(0) is not None:
                ground.GetVisualShape(0).SetTexture(
                    chrono.GetChronoDataFile("textures/concrete.jpg")
                )
        except Exception:
            pass
        self.system.Add(ground)

        wheel_mat = chrono.ChContactMaterialNSC()
        self.driver = robot.ViperDCMotorControl()
        self.rover = robot.Viper(self.system, robot.ViperWheelType_SimpleWheel)
        self.rover.SetWheelContactMaterial(wheel_mat)
        self.rover.SetDriver(self.driver)
        self.rover.Initialize(
            chrono.ChFramed(
                chrono.ChVector3d(0, 0, 0.4), chrono.ChQuaterniond(1, 0, 0, 0)
            )
        )

        return self._get_obs(), {}

    def step(self, action):
        assert self.system is not None and self.rover is not None and self.driver is not None

        torques = [float(np.clip(action[i], -3, 3)) * 5.0 for i in range(4)]
        steer = float(np.clip(action[4], -1, 1)) * (math.pi / 12)
        wheel_ids = (robot.FL, robot.FR, robot.RL, robot.RR)
        for wid, tq in zip(wheel_ids, torques):
            self.driver.SetMotorStallTorque(tq, wid)
        self.driver.SetSteering(steer)

        for _ in range(self._steps_per_control):
            self.rover.Update()
            self.system.DoStepDynamics(self._step_size)

        obs = self._get_obs()
        reward = self._get_reward()
        terminated = self.system.GetChTime() >= self._max_time
        truncated = False
        return obs, reward, terminated, truncated, {}

    def _get_obs(self) -> np.ndarray:
        assert self.rover is not None
        pos = self.rover.GetChassisPos()
        vel = self.rover.GetChassisVel()
        rot = self.rover.GetChassisRot()
        obs = np.zeros(18, dtype=np.float64)
        obs[0:3] = [pos.x, pos.y, pos.z]
        obs[3:6] = [vel.x, vel.y, vel.z]
        obs[6:10] = [rot.e0, rot.e1, rot.e2, rot.e3]
        obs[10:14] = [
            self.rover.GetWheelLinVel(robot.FL).x,
            self.rover.GetWheelLinVel(robot.FR).x,
            self.rover.GetWheelLinVel(robot.RL).x,
            self.rover.GetWheelLinVel(robot.RR).x,
        ]
        return obs

    def _get_reward(self) -> float:
        assert self.rover is not None
        x = self.rover.GetChassisPos().x
        reward = x - self._prev_x
        self._prev_x = x
        return float(reward)
