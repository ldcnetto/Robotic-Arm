from transformations import SE2
from kinematics import ForwardKinematics, InverseKinematics
from trajectories import JointTrajectory, EuclideanTrajectory

if __name__ == "__main__":
    # Exemplo de uso da classe SE2
    transform_matrix = SE2.SE2_xy(1, 2)
    print("Matriz de Transformação SE2_xy(1, 2):\n", transform_matrix)

    # Exemplo de uso da classe ForwardKinematics
    fk_result = ForwardKinematics.fk(math.pi/2, math.pi/2)
    print("Cinemática Direta (fk):", fk_result)

    # Exemplo de uso da classe InverseKinematics
    ik_result = InverseKinematics.ik(1, 1)
    print("Cinemática Inversa (ik):", ik_result)

    # Exemplo de uso da classe JointTrajectory
    traj_theta1, traj_theta2 = JointTrajectory.traj_joint(0, 0, math.pi/2, math.pi/2)
    print("Trajetória das Juntas (traj_joint):", traj_theta1, traj_theta2)

    # Exemplo de uso da classe EuclideanTrajectory
    EuclideanTrajectory.plot_trajs(traj_theta1, traj_theta2)