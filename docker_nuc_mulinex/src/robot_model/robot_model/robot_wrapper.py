import os
import tempfile

import numpy as np
import pinocchio as pin
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory


class RobotWrapper(pin.RobotWrapper):
    def __init__(self, robot_name: str):
        # ======================== Read The YAML File ======================== #

        package_share_directory = get_package_share_directory('robot_model')

        yaml_path = package_share_directory + '/robots/all_robots.yaml'

        with open(yaml_path) as f:
            doc = yaml.load(f, yaml.SafeLoader)

        pkg_name = doc[robot_name]['pkg_name']
        urdf_path = doc[robot_name]['urdf_path']
        pkg_share = get_package_share_directory(pkg_name)

        full_urdf_path = pkg_share + urdf_path

        # ============ Create The Robot Model And The Data Objects =========== #

        if full_urdf_path.endswith('.xacro'):
            urdf_args = doc[robot_name].get('urdf_args', {})
            # Convert all values to strings for xacro
            mappings = {str(k): str(v) for k, v in urdf_args.items()}

            doc_xacro = xacro.process_file(full_urdf_path, mappings=mappings)
            urdf_xml = doc_xacro.toprettyxml(indent='  ')

            with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as tmp_file:
                tmp_file.write(urdf_xml)
                tmp_urdf_path = tmp_file.name

            try:
                model, collision_model, visual_model = pin.buildModelsFromUrdf(
                    tmp_urdf_path,
                    pkg_share,
                    root_joint=pin.JointModelFreeFlyer(),
                )
            finally:
                os.unlink(tmp_urdf_path)
        else:
            model, collision_model, visual_model = pin.buildModelsFromUrdf(
                full_urdf_path,
                pkg_share,
                root_joint=pin.JointModelFreeFlyer(),
            )

        super().__init__(model, collision_model, visual_model)

        # ==================================================================== #

        self.base_name = doc[robot_name]['base_name']
        self.feet_names = doc[robot_name]['feet_names']
        self.joint_names = doc[robot_name]['ordered_joint_names']
        self.n_joints_per_leg = doc[robot_name].get('n_joints_per_leg', 3)

    def compute_kinematics(self, joint_positions, joint_velocities=None):
        q = np.concatenate((np.zeros(6), [1], joint_positions))
        v = (
            np.concatenate((np.zeros(6), joint_velocities))
            if joint_velocities is not None
            else None
        )

        if v is not None:
            self.forwardKinematics(q, v)
        else:
            self.forwardKinematics(q)
        pin.computeJointJacobians(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

    def get_feet_pos(self):
        return [
            self.data.oMf[self.model.getFrameId(foot_name)].translation
            for foot_name in self.feet_names
        ]

    def get_feet_vel(self):
        return [
            pin.getFrameVelocity(
                self.model,
                self.data,
                self.model.getFrameId(foot_name),
                pin.LOCAL_WORLD_ALIGNED,
            )
            for foot_name in self.feet_names
        ]

    def get_feet_jacobian(self):
        feet_jacobian = [None for _ in self.feet_names]
        for i, foot_name in enumerate(self.feet_names):
            frame_id = self.model.getFrameId(foot_name)
            feet_jacobian[i] = self.getFrameJacobian(frame_id, pin.LOCAL_WORLD_ALIGNED)[0:3, :]

        return feet_jacobian
