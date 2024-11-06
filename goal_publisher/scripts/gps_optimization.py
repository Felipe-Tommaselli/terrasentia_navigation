import numpy as np
import casadi as ca

from scipy.spatial.transform import Rotation as R

class GPSOptimization:
    def __init__(self, transformation_guess, max_num_nodes):
        self.max_num_nodes = max_num_nodes
        self.transformation_guess = transformation_guess

        # Define CasADi variables for translation (tx, ty, tz) and rotation (rx, ry, rz)
        trans = ca.MX.sym('trans', 3, 1)
        quat = ca.MX.sym('quat', 4, 1)

        # Input data (7 values per node: odom_x, odom_y, odom_z, gps_x, gps_y, gps_z, gps_accuracy)
        input_data = ca.MX.sym('input_data', 7, max_num_nodes)

        # Normalize quaternion
        q = quat / ca.norm_2(quat)  # Normalize the quaternion

        # Apply rotation using quaternion
        rotation_matrix = ca.vertcat(
            ca.horzcat(1 - (q[1] ** 2) - (q[2] ** 2), 2 * (q[0] * q[1] - q[3] * q[2]), 2 * (q[3] * q[1] + q[0] * q[2])),
            ca.horzcat(2 *  (q[0] * q[1] + q[3] * q[2]), 1 - (q[0] ** 2) - (q[2] ** 2), 2 * (q[1] * q[2] - q[3] * q[0])),
            ca.horzcat(2 * (q[0] * q[2] - q[3] * q[1]), 2 * (q[3] * q[0] + q[1] * q[2]), 1 - (q[0] ** 2) - (q[1] ** 2))
        )

        # Build the cost function
        cost_function = 0
        for i in range(max_num_nodes):
            odom_point = input_data[:3, i]  # Odom point (x,y,z)
            gps_point = input_data[3:6, i]  # GPS point (x,y,z)
            gps_accuracy = input_data[6, i]  # GPS accuracy

            weight = ca.if_else(gps_accuracy > 0, 1 / (gps_accuracy ** 2), 0.0)

            # Apply rotation using Euler angles
            transformed_gps_point = rotation_matrix @ gps_point

            # Apply translation
            transformed_gps_point = transformed_gps_point + trans

            # Compute squared distance between transformed odometry point and GPS point (x,y,z)
            residuals_squared_distance = ca.sumsqr(odom_point - transformed_gps_point)

            cost_function += weight * residuals_squared_distance

        # Create an NLP problem in CasADi (Nonlinear Programming Problem)
        nlp_problem = {
            'f': cost_function,
            'x': ca.vertcat(trans, quat),
            'p': input_data,
            'g': []
        }

        solver_options = {
            'ipopt': {
                'max_iter': 200,
                'print_level': 0,
                'tol': 1e-4,
                'linear_solver': 'mumps'
            }
        }

        # Create the solver once
        self.solver = ca.nlpsol('solver', 'ipopt', nlp_problem, solver_options)

    def run(self, input_data):
        """
        Run the optimization with a new set of data points.
        
        :param data_buffer: A numpy array of shape (7, num_nodes) containing odometry points,
                            GPS points and GPS accuracies.
                            Each column is [odom_x, odom_y, odom_z, gps_x, gps_y, gps_z, gps_accuracy].
        
        :return: The optimized transformation matrix (translation + rotation).
        """
        initial_translation = self.transformation_guess[:3, 3]
        initial_quaternions = R.from_matrix(self.transformation_guess[:3, :3]).as_quat()
        
        # Initial guess for translation and rotation angles
        initial_translation_rotation = np.hstack((
            initial_translation,    # Initial translation guess (tx, ty, tz)
            initial_quaternions     # Initial quaternions guess (qx, qy, qz, qw)
        ))

        data_buffer = np.zeros((7, self.max_num_nodes))
        data_buffer[:, :input_data.shape[1]] = input_data

        # Solve the optimization problem with new input data
        solution_result = self.solver(x0=initial_translation_rotation, p=data_buffer)

        opt_solution = solution_result['x'].full().flatten()

        # Extract optimized translation and rotation
        opt_translation = opt_solution[:3]
        opt_quaternions = opt_solution[3:]

        # Convert Euler angles to rotation matrix
        opt_rotation = R.from_quat(opt_quaternions).as_matrix()

        # Construct the full transformation matrix (3x4)
        output = np.eye(4)
        output[:3,:3] = opt_rotation
        output[:3, 3] = opt_translation

        return output