#
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
# property and proprietary rights in and to this material, related
# documentation and any modifications thereto. Any use, reproduction,
# disclosure or distribution of this material and related documentation
# without an express license agreement from NVIDIA CORPORATION or
# its affiliates is strictly prohibited.
#

# Standard Library
from typing import Dict, List

# Third Party
import math
import numpy as np
from matplotlib import cm
from omni.isaac.core import World
from omni.isaac.core.materials import OmniPBR
from omni.isaac.core.objects import cuboid
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage
from pxr import UsdPhysics, UsdLux, PhysxSchema

def set_drive_type(prim_path, drive_type):
    joint_prim = get_prim_at_path(prim_path)

    # set drive type ("angular" or "linear")
    drive = UsdPhysics.DriveAPI.Apply(joint_prim, drive_type)
    return drive

def set_drive_target_position(drive, target_value):
    if not drive.GetTargetPositionAttr():
        drive.CreateTargetPositionAttr(target_value)
    else:
        drive.GetTargetPositionAttr().Set(target_value)

def set_drive_target_velocity(drive, target_value):
    if not drive.GetTargetVelocityAttr():
        drive.CreateTargetVelocityAttr(target_value)
    else:
        drive.GetTargetVelocityAttr().Set(target_value)

def set_drive_stiffness(drive, stiffness):
    if not drive.GetStiffnessAttr():
        drive.CreateStiffnessAttr(stiffness)
    else:
        drive.GetStiffnessAttr().Set(stiffness)

def set_drive_damping(drive, damping):
    if not drive.GetDampingAttr():
        drive.CreateDampingAttr(damping)
    else:
        drive.GetDampingAttr().Set(damping)

def set_drive_max_force(drive, max_force):
    if not drive.GetMaxForceAttr():
        drive.CreateMaxForceAttr(max_force)
    else:
        drive.GetMaxForceAttr().Set(max_force)

def set_drive(prim_path, drive_type, target_type, target_value, stiffness, damping, max_force) -> None:
    drive = set_drive_type(prim_path, drive_type)

    # set target type ("position" or "velocity")
    if target_type == "position":
        set_drive_target_position(drive, target_value)
    elif target_type == "velocity":
        set_drive_target_velocity(drive, target_value)

    set_drive_stiffness(drive, stiffness)
    set_drive_damping(drive, damping)
    set_drive_max_force(drive, max_force)

def create_distant_light(prim_path="/World/defaultDistantLight", intensity=5000):
    stage = get_current_stage()
    light = UsdLux.DistantLight.Define(stage, prim_path)
    light.GetPrim().GetAttribute("intensity").Set(intensity)

# CuRobo
from curobo.util.logger import log_warn

ISAAC_SIM_23 = False
try:
    # Third Party
    from omni.isaac.urdf import _urdf  # isaacsim 2022.2
except ImportError:
    # Third Party
    from omni.importer.urdf import _urdf  # isaac sim 2023.1

    ISAAC_SIM_23 = True
# Standard Library
from typing import Optional

# Third Party
from omni.isaac.core.utils.extensions import enable_extension

# CuRobo
from curobo.util_file import get_assets_path, get_filename, get_path_of_dir, join_path


def add_extensions(simulation_app, headless_mode: Optional[str] = None):
    ext_list = [
        "omni.kit.asset_converter",
        "omni.kit.tool.asset_importer",
        "omni.isaac.asset_browser",
    ]
    if headless_mode is not None:
        log_warn("Running in headless mode: " + headless_mode)
        ext_list += ["omni.kit.livestream." + headless_mode]
    [enable_extension(x) for x in ext_list]
    simulation_app.update()

    return True


class xArm(Robot):
    def __init__(
        self,
        prim_path: str,
        name: Optional[str] = "xarm7",
        usd_path: Optional[str] = None,
        translation: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
    ) -> None:

        self._usd_path = usd_path
        self._name = name

        if self._usd_path is None:
            self._usd_path = "/pkgs/curobo/src/curobo/content/assets/robot/xarm_description/usd/xarm7.usd"

        add_reference_to_stage(self._usd_path, prim_path)

        super().__init__(
            prim_path=prim_path,
            name=name,
            translation=translation,
            orientation=orientation,
            articulation_controller=None,
        )

        dof_paths = [
            "link_base/joint1",
            "link1/joint2",
            "link2/joint3",
            "link3/joint4",
            "link4/joint5",
            "link5/joint6",
            "link6/joint7",
            "xarm_gripper_base_link/left_drive_joint",
            "xarm_gripper_base_link/right_drive_joint",
        ]

        drive_type = ["angular"] * 9
        default_dof_pos = [math.degrees(x) for x in [0.0, -0.698, 0.0, 0.349, 0.0, 1.047, 0.0, 0.0, 0.0]]
        stiffness = [400 * np.pi / 180] * 7 + [80 * np.pi / 180] * 2
        damping = [80 * np.pi / 180] * 7 + [160 * np.pi / 180] * 2
        max_force = [87, 87, 87, 87, 12, 12, 12, 200, 200]
        max_velocity = [math.degrees(x) for x in [2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61, 2.0, 2.0]]

        for i, dof in enumerate(dof_paths):
            set_drive(
                prim_path=f"{self.prim_path}/{dof}",
                drive_type=drive_type[i],
                target_type="position",
                target_value=default_dof_pos[i],
                stiffness=stiffness[i],
                damping=damping[i],
                max_force=max_force[i],
            )

            PhysxSchema.PhysxJointAPI(get_prim_at_path(f"{self.prim_path}/{dof}")).CreateMaxJointVelocityAttr().Set(
                max_velocity[i]
            )

    def set_xarm_properties(self, stage, prim):
        for link_prim in prim.GetChildren():
            if link_prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI): 
                rb = PhysxSchema.PhysxRigidBodyAPI.Get(stage, link_prim.GetPrimPath())
                rb.GetDisableGravityAttr().Set(True)


############################################################
def add_robot_to_scene(
    robot_config: Dict,
    my_world: World,
    load_from_usd: bool = False,
    subroot: str = "",
    robot_name: str = "robot",
    position: np.array = np.array([0, 0, 0]),
):
    if load_from_usd:
        robot_path = "/World/xarm7"
        robot_p = xArm(robot_path, name="xarm7", translation=position)
    else:
        urdf_interface = _urdf.acquire_urdf_interface()

        import_config = _urdf.ImportConfig()
        import_config.merge_fixed_joints = False
        import_config.convex_decomp = False
        import_config.import_inertia_tensor = True
        import_config.fix_base = True
        import_config.make_default_prim = False
        import_config.self_collision = False
        import_config.create_physics_scene = True
        import_config.import_inertia_tensor = False
        import_config.default_drive_strength = 20000
        import_config.default_position_drive_damping = 500
        import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
        import_config.distance_scale = 1
        import_config.density = 0.0
        asset_path = get_assets_path()
        if (
            "external_asset_path" in robot_config["kinematics"]
            and robot_config["kinematics"]["external_asset_path"] is not None
        ):
            asset_path = robot_config["kinematics"]["external_asset_path"]
        full_path = join_path(asset_path, robot_config["kinematics"]["urdf_path"])
        robot_path = get_path_of_dir(full_path)
        filename = get_filename(full_path)
        imported_robot = urdf_interface.parse_urdf(robot_path, filename, import_config)
        dest_path = subroot
        robot_path = urdf_interface.import_robot(
            robot_path,
            filename,
            imported_robot,
            import_config,
            dest_path,
        )
        robot_p = Robot(
            prim_path=robot_path,
            name=robot_name,
            position=position,
        )
    if ISAAC_SIM_23:
        robot_p.set_solver_velocity_iteration_count(4)
        robot_p.set_solver_position_iteration_count(44)

        my_world._physics_context.set_solver_type("PGS")

    robot = my_world.scene.add(robot_p)

    return robot, robot_path


class VoxelManager:
    def __init__(
        self,
        num_voxels: int = 5000,
        size: float = 0.02,
        color: List[float] = [1, 1, 1],
        prefix_path: str = "/World/curobo/voxel_",
        material_path: str = "/World/looks/v_",
    ) -> None:
        self.cuboid_list = []
        self.cuboid_material_list = []
        self.disable_idx = num_voxels
        for i in range(num_voxels):
            target_material = OmniPBR("/World/looks/v_" + str(i), color=np.ravel(color))

            cube = cuboid.VisualCuboid(
                prefix_path + str(i),
                position=np.array([0, 0, -10]),
                orientation=np.array([1, 0, 0, 0]),
                size=size,
                visual_material=target_material,
            )
            self.cuboid_list.append(cube)
            self.cuboid_material_list.append(target_material)
            cube.set_visibility(True)

    def update_voxels(self, voxel_position: np.ndarray, color_axis: int = 0):
        max_index = min(voxel_position.shape[0], len(self.cuboid_list))

        jet = cm.get_cmap("hot")  # .reversed()
        z_val = voxel_position[:, 0]

        jet_colors = jet(z_val)

        for i in range(max_index):
            self.cuboid_list[i].set_visibility(True)

            self.cuboid_list[i].set_local_pose(translation=voxel_position[i])
            self.cuboid_material_list[i].set_color(jet_colors[i][:3])

        for i in range(max_index, len(self.cuboid_list)):
            self.cuboid_list[i].set_local_pose(translation=np.ravel([0, 0, -10.0]))

            # self.cuboid_list[i].set_visibility(False)

    def clear(self):
        for i in range(len(self.cuboid_list)):
            self.cuboid_list[i].set_local_pose(translation=np.ravel([0, 0, -10.0]))
