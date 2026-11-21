"""MuJoCo loop dynamics: only the crank is motorized; the rod and knee are passive."""

import math
import xml.etree.ElementTree as ET

from .geometry import reference_pose

TIMESTEP = 0.001


def numbers(*values):
    return " ".join(format(value, ".12g") for value in values)


def build_model(config):
    """Build a clamped planar mechanism in its geometrically closed reference pose."""
    config.validate()
    pose = reference_pose(config, config.initial_servo)
    root = ET.Element("mujoco", model="saprog_single_leg")
    ET.SubElement(root, "compiler", angle="radian", autolimits="true")
    option = ET.SubElement(root, "option", timestep=str(TIMESTEP), gravity="0 0 -9.81",
                           integrator="implicitfast", iterations="100", tolerance="1e-10")
    ET.SubElement(option, "flag", autoreset="disable")
    default = ET.SubElement(root, "default")
    ET.SubElement(default, "joint", damping="0.002", armature="0.00001")
    # Link collision is disabled for this hanging bench rig, which has no floor.
    ET.SubElement(default, "geom", type="capsule", size="0.003", contype="0", conaffinity="0")
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "site", name="O", pos="0 0 0")
    ET.SubElement(world, "site", name="D", pos=numbers(config.pivot_x, 0, config.pivot_z))
    crank = ET.SubElement(world, "body", name="crank", euler=numbers(0, -pose["theta"], 0))
    ET.SubElement(crank, "joint", name="servo", axis="0 1 0", ref=str(config.initial_servo),
                  range=numbers(config.servo_lower, config.servo_upper))
    ET.SubElement(crank, "geom", fromto=numbers(0, 0, 0, config.crank_radius, 0, 0),
                  mass=str(config.crank_mass))
    ET.SubElement(crank, "site", name="A", pos=numbers(config.crank_radius, 0, 0))
    rod = ET.SubElement(crank, "body", name="rod", pos=numbers(config.crank_radius, 0, 0),
                        euler=numbers(0, -(pose["gamma"] - pose["theta"]), 0))
    ET.SubElement(rod, "joint", name="rod_hinge", axis="0 -1 0")
    ET.SubElement(rod, "geom", fromto=numbers(0, 0, 0, config.rod_length, 0, 0),
                  mass=str(config.rod_mass))
    ET.SubElement(rod, "site", name="B", pos=numbers(config.rod_length, 0, 0))
    leg = ET.SubElement(world, "body", name="lower_leg",
                        pos=numbers(config.pivot_x, 0, config.pivot_z),
                        euler=numbers(0, -pose["beta"], 0))
    ET.SubElement(leg, "joint", name="knee", axis="0 -1 0")
    ET.SubElement(leg, "geom", fromto=numbers(0, 0, 0, config.knee_radius, 0, 0),
                  mass=str(config.leg_mass * 0.2))
    foot = (config.foot_length * math.cos(config.foot_offset), 0,
            config.foot_length * math.sin(config.foot_offset))
    ET.SubElement(leg, "geom", fromto=numbers(0, 0, 0, *foot), mass=str(config.leg_mass * 0.8))
    ET.SubElement(leg, "site", name="C", pos=numbers(config.knee_radius, 0, 0))
    ET.SubElement(leg, "site", name="F", pos=numbers(*foot))
    equality = ET.SubElement(root, "equality")
    ET.SubElement(equality, "connect", name="rod_knee_pin", body1="rod", body2="lower_leg",
                  anchor=numbers(config.rod_length, 0, 0), solref="0.003 1", solimp="0.999 0.9999 0.001")
    motor = ET.SubElement(root, "actuator")
    ET.SubElement(motor, "motor", joint="servo", ctrllimited="true",
                  ctrlrange=numbers(-config.torque_limit, config.torque_limit))
    return ET.tostring(root, encoding="unicode")


class LegPhysics:
    def __init__(self, config):
        try:
            import mujoco
            import numpy as np
        except ImportError as error:
            raise RuntimeError(
                "MuJoCo is missing from the ROS Python environment. "
                "See ros2/saprog_leg_test/README.md for setup."
            ) from error
        self.mj, self.np, self.config = mujoco, np, config
        self.xml = build_model(config)
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.servo_joint = self.model.joint("servo")
        self.q_index = int(self.servo_joint.qposadr[0])
        self.v_index = int(self.servo_joint.dofadr[0])
        self.leg_id = self.model.body("lower_leg").id
        self.sites = {name: self.model.site(name).id for name in ("O", "A", "B", "C", "D", "F")}
        self.beta = reference_pose(config, config.initial_servo)["beta"]
        self.reset()

    def reset(self):
        self.mj.mj_resetData(self.model, self.data)
        self.target = self.config.initial_servo
        self.ramped_target = self.target
        self.torque = 0.0
        self.mj.mj_forward(self.model, self.data)

    def step(self, target):
        cfg, data = self.config, self.data
        self.target = target
        delta = max(-cfg.speed_limit * TIMESTEP,
                    min(cfg.speed_limit * TIMESTEP, target - self.ramped_target))
        self.ramped_target += delta
        q, velocity = float(data.qpos[self.q_index]), float(data.qvel[self.v_index])
        torque = cfg.kp * (self.ramped_target - q) - cfg.kd * velocity
        # Approximate motor speed envelope: no driving torque beyond no-load speed.
        # Passive external forces can still backdrive the joint faster.
        limit = cfg.torque_limit * max(0, 1 - abs(velocity) / cfg.speed_limit)
        self.torque = max(-limit, min(limit, torque)) if torque * velocity > 0 else (
            max(-cfg.torque_limit, min(cfg.torque_limit, torque))
        )
        data.ctrl[0] = self.torque
        data.qfrc_applied[:] = 0
        if cfg.foot_load:
            force = self.np.array([0.0, 0.0, -cfg.foot_load])
            self.mj.mj_applyFT(self.model, data, force, self.np.zeros(3),
                               data.site_xpos[self.sites["F"]], self.leg_id, data.qfrc_applied)
        self.mj.mj_step(self.model, data)
        # Refresh body/site transforms to the newly integrated qpos.
        self.mj.mj_forward(self.model, data)
        if not self.np.isfinite(data.qpos).all() or not self.np.isfinite(data.qvel).all():
            raise RuntimeError("Physics became non-finite; reset and reduce load or gains")
        if self.closure_error() > 0.002:
            raise RuntimeError("Loop closure error exceeded 2 mm; reset and reduce load or gains")

    def closure_error(self):
        return float(self.np.linalg.norm(
            self.data.site_xpos[self.sites["B"]] - self.data.site_xpos[self.sites["C"]],
        ))

    def state(self):
        return {
            "sim_time": float(self.data.time),
            "servo_angle": float(self.data.qpos[self.q_index]),
            "servo_target": self.target,
            "servo_speed": float(self.data.qvel[self.v_index]),
            "knee_angle": self.beta + float(self.data.joint("knee").qpos[0]),
            "torque": self.torque,
            "closure_error": self.closure_error(),
            "points": {
                name: [float(self.data.site_xpos[index, axis]) for axis in (0, 2)]
                for name, index in self.sites.items()
            },
        }
