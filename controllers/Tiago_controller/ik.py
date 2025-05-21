from ikpy.chain import Chain
from ikpy.link import URDFLink
import numpy as np
import os

class IKSolver:
    def __init__(self, urdf_path: str):
        assert os.path.exists(urdf_path), f"URDF file not found: {urdf_path}"
        
        self.chain = Chain.from_urdf_file(
            urdf_path,
            base_elements = ["base_link"],
            active_links_mask = [
                False,  # base_link
                False,  # Torso (fixed)
                False,  # torso_lift_link (fixed)
                False,  # TIAGo front arm (fixed)
                True,   # TIAGo front arm_3 ← arm_1_joint
                True,   # arm_2_link       ← arm_2_joint
                True,   # arm_3_link       ← arm_3_joint
                True,   # arm_4_link       ← arm_4_joint
                True,   # arm_5_link       ← arm_5_joint
                True,   # arm_6_link       ← arm_6_joint
                True,   # arm_7_link       ← arm_7_joint
                False,  # wrist_ft_tool_link (fixed)
                False   # front (fixed)
            ]
        )
        
    def forward_kinematics(self, joint_angles: list):
        assert len(joint_angles) == 7, "Input must be 7 joint angles."
        initial_position = [0.0] * len(self.chain.links)
        joint_values = self.chain.active_to_full(joint_angles, initial_position)
        return self.chain.forward_kinematics(joint_values)
        
def main():
    # Đường dẫn tới file URDF
    urdf_path = "/home/bibi/Webots_Projects/Autonomous_Robots_final_project/controllers/Tiago_controller/For_IK_calculation.urdf"

    # Khởi tạo IK solver
    ik_solver = IKSolver(urdf_path)

    # Cấu hình joint góc (rad) thực tế từ robot để kiểm tra
    joint_angles = [
        1.5158,   # arm_1_joint
        np.pi/4,   # arm_2_joint
        0.0,  # arm_3_joint
        np.pi/4,   # arm_4_joint
        0.0,   # arm_5_joint
        0.0,  # arm_6_joint
        np.pi/2    # arm_7_joint
    ]

    # Tính forward kinematics
    pose_matrix = ik_solver.forward_kinematics(joint_angles)

    # In kết quả
    print("=== Forward Kinematics Result ===")
    print("Pose matrix (4x4):\n", pose_matrix)
    print("\nEnd-effector position (x, y, z):", pose_matrix[0:3, 3])

if __name__ == "__main__":
    main()