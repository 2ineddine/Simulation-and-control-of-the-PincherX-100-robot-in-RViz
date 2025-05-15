import math
from interbotix_xs_modules.arm import InterbotixManipulatorXS
import numpy as np
import sys


# constants 
L1 = 89.54
Lm = 35
L2 = 100
Lr = math.sqrt(L2 ** 2 + Lm ** 2)
L3 = 100
L4 = 109
q_nom = np.array([0, -np.arctan2(L2, Lm), np.arctan2(L2, Lm), 0])


def coast_function(qnom, q):
    # Calculate the squared differences between qnom and all rows of q
    distances = np.linalg.norm(q - qnom, axis=1)  # Compute the Euclidean distance between qnom and each row of q
    
    # Return the configuration that minimizes the distance
    idx_min = np.argmin(distances)  # Find the index of the closest configuration
    return np.array(q[idx_min])  # Return the closest configuration



def matrice_t_h(q_dh):
    C1 = np.cos(q_dh[0])
    C2 = np.cos(q_dh[1])
    S1 = np.sin(q_dh[0])
    S2 = np.sin(q_dh[1])
    C23 = np.cos(q_dh[1] + q_dh[2])
    S23 = np.sin(q_dh[1] + q_dh[2])
    C234 = np.cos(q_dh[1] + q_dh[2] + q_dh[3])
    S234 = np.sin(q_dh[1] + q_dh[2] + q_dh[3])

    mat = np.array([[C1 * C234, -C1 * S234, -S1, C1 * (Lr * C2 + L3 * C23 + L4 * C234)],
                    [S1 * C234, -S1 * S234, C1, S1 * (Lr * C2 + L3 * C23 + L4 * C234)],
                    [-S234, -C234, 0, -(Lr * S2 + L3 * S23 + L4 * S234) + L1],
                    [0, 0, 0, 1]])
    return mat[:3,3]


# from q_dh to qa
def q_dh_to_q_actuator(q_dh):
    return q_dh - q_nom


# from qa to q_dh
def q_actuator_to_q_dh(q_actuator):
    return  q_actuator + q_nom


def jacobian(q):
    C1 = np.cos(q[0])
    C2 = np.cos(q[1])
    S1 = np.sin(q[0])
    S2 = np.sin(q[1])
    C23 = np.cos(q[1] + q[2])
    S23 = np.sin(q[1] + q[2])
    C234 = np.cos(q[1] + q[2] + q[3])
    S234 = np.sin(q[1] + q[2] + q[3])

    J = np.array([[-(Lr * C2 + L3 * C23 + L4 * C234) * S1, -(Lr * S2 + L3 * S23 + L4 * S234) * C1,
                   -(L3 * S23 + L4 * S234) * C1, -L4 * S234 * C1],
                  [(Lr * C2 + L3 * C23 + L4 * C234) * C1, -(Lr * S2 + L3 * S23 + L4 * S234) * S1,
                   -(L3 * S23 + L4 * S234) * S1, -L4 * S234 * S1],
                  [0, -Lr * C2 - L3 * C23 - L4 * C234, -L3 * C23 - L4 * C234, -L4 * C234]])

    return J


def convert_param_op_to_qdh(param_op):
    px, py, pz, phi = param_op

    # Calculation of q1
    q1 = [np.arctan2(py, px), np.arctan2(py, px) + np.pi, np.arctan2(py, px) - np.pi]

    # Initialization of variables
    solutions_qdh = np.empty((0, 4))  # Empty matrix to store the solutions
    epsilon = np.array([-1, 1])        # Possible values for epsilon

    for j in q1:
        # Calculation of q2
        U = px * np.cos(j) + py * np.sin(j) - L4 * np.cos(phi)
        V = L1 - pz - L4 * np.sin(phi)
        A = 2 * Lr * U
        B = 2 * Lr * V
        W = U ** 2 + V ** 2 + Lr ** 2 - L3 ** 2
        value = A ** 2 + B ** 2 - W ** 2

        if value >= 0:
            for epsilon_value in epsilon:
                q2 = np.arctan2(B * W - epsilon_value * A * np.sqrt(value),
                                A * W + epsilon_value * B * np.sqrt(value))

                # Calculation of q3
                S2 = np.sin(q2)
                C2 = np.cos(q2)
                q3 = np.arctan2(-U * S2 + V * C2, U * C2 + V * S2 - Lr)

                # Calculation of q4
                q4 = phi - q2 - q3

                # Stack the solution into the solutions_qdh array
                solutions_qdh = np.vstack([solutions_qdh, [j, q2, q3, q4]])

    # Check if solutions were found
    if solutions_qdh.size > 0:
        print(f"The best solution for q vector is: {coast_function(q_nom, solutions_qdh)}")
        return coast_function(q_nom, solutions_qdh)
    else:
        print("There is no solution for this position")
        return np.empty((0, 4))  # Empty array compatible with np.array



def iterative_algorithm(q_initial, x_but, tolerance=1e-11, max_iterations=100, qnom=None):
    # Parameters
    alpha = 0.3
    beta = 0.3 

    # Initialization
    x_courant = matrice_t_h(q_initial)  # Current position extracted from the homogeneous matrix
    qk0 = q_initial
    J = jacobian(q_initial)  # Initial Jacobian

    # Check the shape of the Jacobian (should be of size 3x4)
    if np.shape(J) == (3, 4):
        print("The Jacobian has the correct shape.")
    else:
        print(f"The Jacobian does not have the correct shape, current shape: {J.shape}")

    J = J.astype(np.float64)

    # Iterative algorithm
    for i in range(max_iterations):
        # Inverse of the Jacobian
        J_inverse = np.linalg.pinv(J)  # Use the pseudo-inverse to avoid singularities

        # Update calculation
        term1 = alpha * np.dot(J_inverse, (x_but - x_courant))
        term2 = beta * np.dot((np.eye(len(q_initial)) - np.dot(J_inverse, J)), (qnom - qk0) if qnom is not None else np.zeros_like(qk0))
        qk1 = qk0 + term1 + term2

        # Check stop criterion
        if np.linalg.norm(qk0 - qk1) < tolerance:
            print(f"Stop criterion reached after {i+1} iterations.")
            break

        # Update
        qk0 = qk1

        # Calculate the new Jacobian
        J = jacobian(qk0)
        J = J.astype(np.float64)

        # Update the current position
        x_courant = matrice_t_h(qk0)

    # Final calculation
    phi = np.sum(qk1[1:])  # Sum of angles q2, q3, q4
    px, py, pz = x_but
    U = px * np.cos(qk1[0]) + py * np.sin(qk1[0]) - L4 * np.cos(phi)
    V = L1 - pz - L4 * np.sin(phi)
    A = 2 * Lr * U
    B = 2 * Lr * V
    W = U**2 + V**2 + Lr**2 - L3**2
    D = A**2 + B**2 - W**2

    # Return the solution or None if impossible
    if D >= 0:
        print(f"The best coordination is: {qk1} \t for the next position {x_but} \n"
              f"Which corresponds in real parameters to: {np.insert(matrice_t_h(qk1), len(matrice_t_h(qk1)), phi)}")
        return qk1
    else:
        print("No valid solution was found.")
        return None

    
def main():
    
    bot = InterbotixManipulatorXS("px100", "arm", "gripper")
    

################# beginning of the part.1 #######################################

    # object1 = [[100,200,3,np.pi/4],[100,-200,3,np.pi/4]]
    # object2 = [[0,0,250,-np.pi/10],[20,100,20,np.pi/2]]
    # object3 = [[180,200,80,np.pi/10],[180,-200,80,np.pi/10]]
    # for i in [object1,object2,object3]:
    #     initial_position = i[0]
    #     second_position = i[1]
    #     initial_qdh_cord = convert_param_op_to_qdh(initial_position)
    #     initial_qa_cord = q_dh_to_q_actuator(initial_qdh_cord)
    #     second_qdh_cord = convert_param_op_to_qdh(second_position)
    #     second_qa_cord = q_dh_to_q_actuator(second_qdh_cord)
    #     bot.gripper.open()
    #     bot.arm.set_joint_positions(initial_qa_cord)
    #     bot.gripper.close()
    #     bot.arm.set_joint_positions(second_qa_cord)
    #     bot.gripper.open()
    # bot.arm.go_to_sleep_pose()

################# the end of the part.1 #########################################


################# the beginning of the part.2 ###################################
################ Multiples configurations for one point #########################
# the first test
    # x_but  = [100,200,3,np.pi/4]
    # q_but  = convert_param_op_to_qdh(q_actuator_to_q_dh(x_but))
    # q_test = iterative_algorithm(q_but,np.array([100,200,3]),qnom=q_nom)

# the second test 

    # x_but  = np.array([Lm + L3 + L4,0,L1 + L2,-1])
    # q_but  = convert_param_op_to_qdh(q_actuator_to_q_dh(x_but))
    # iterative_algorithm(q_but,np.array([Lm + L3 + L4,0,L1 + L2]) , qnom=q_nom)



##################The beginning of the pickup-dropoff section  ###################

    bot.gripper.open()
    object1 = [[100,250,100],[90,-100,10]]
    object2 = [[20,0,250],[150,50,10]]
    object3 = [[150,0,80],[40,-100,-30]]
    q_initial = q_nom
    for i in np.array([object1,object2,object3]):
        
        
        qdh1 = iterative_algorithm( q_initial ,i[0], qnom=q_nom)
        qa1 = q_dh_to_q_actuator(qdh1)
        bot.arm.set_joint_positions(qa1)
        bot.gripper.close()
        q_initial = qdh1

        qdh2 = iterative_algorithm( q_initial ,i[1], qnom=q_nom)
        qa2 = q_dh_to_q_actuator(qdh2)
        bot.arm.set_joint_positions(qa2)
        bot.gripper.open()
        q_initial = qdh2

    bot.arm.go_to_sleep_pose()

################# the end of the part.2 #########################################

    # bot.arm.set_ee_cartesian_trajectory(pitch=-1.5)
    # bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    # bot.arm.set_single_joint_position("waist", np.pi/2.0)
    # bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.16)
    # bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16)
    # bot.arm.set_single_joint_position("waist", -np.pi/2.0)
    # bot.arm.set_ee_cartesian_trajectory(pitch=1.5)
    # bot.arm.set_ee_cartesian_trajectory(pitch=-1.5)
    # bot.arm.set_single_joint_position("waist", np.pi/2.0)
    # bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.16)
    # bot.gripper.open()
    # bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16)





    

if __name__=='__main__':
    main()




