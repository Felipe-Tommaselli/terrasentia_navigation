import numpy as np
import casadi as ca

class GPSOptimization:
    def __init__(self, transformation_guess, max_num_nodes):
        self.max_num_nodes = max_num_nodes
        self.transformation_guess = transformation_guess

        # Define CasADi variables for translation (tx, ty) and rotation (theta)
        tx = ca.MX.sym('tx')
        ty = ca.MX.sym('ty')
        theta = ca.MX.sym('theta')

        # Input data (5 values per node: odom_x, odom_y, gps_x, gps_y, gps_accuracy)
        input_data = ca.MX.sym('input_data', 5, max_num_nodes)

        # Build the cost function
        cost_function = 0
        for i in range(max_num_nodes):
            odom_point = input_data[:2, i]      # Odom point (x,y)
            gps_point = input_data[2:4, i]      # GPS point (x,y)
            gps_accuracy = input_data[4, i]     # GPS accuracy

            weight = ca.if_else(gps_accuracy > 0, 1 / (gps_accuracy ** 2), 0.0)

            # Apply rotation using theta (rotation matrix in 2D)
            R_theta = ca.vertcat(
                ca.horzcat(ca.cos(theta), -ca.sin(theta)),
                ca.horzcat(ca.sin(theta), ca.cos(theta))
            )

            transformed_gps_point = R_theta @ gps_point

            # Apply translation
            transformed_gps_point = transformed_gps_point + ca.vertcat(tx, ty)

            # Compute squared distance between transformed odometry point and GPS point (x,y)
            residuals_squared_distance = ca.sumsqr(odom_point - transformed_gps_point)

            cost_function += weight * residuals_squared_distance

        # Create an NLP problem in CasADi (Nonlinear Programming Problem)
        nlp_problem = {
            'f': cost_function,
            'x': ca.vertcat(tx, ty, theta),
            'p': input_data,
            'g': []
        }

        solver_options = {
            'ipopt': {
                'max_iter': 200,
                'print_level': 0,
                'print_time': 0,
                'linear_solver': 'mumps'
            }
        }

        # Create the solver once
        self.solver = ca.nlpsol('solver', 'ipopt', nlp_problem, solver_options)

    def run(self, input_data):
        """
        Run the optimization with a new set of data points.
        
        :param input_data: A numpy array of shape (5, num_nodes) containing odometry points,
                           GPS points and GPS accuracies.
                           Each column is [odom_x, odom_y, gps_x, gps_y, gps_accuracy].
        
        :return: The optimized transformation matrix (translation + rotation).
        """
        
        initial_translation_rotation = np.array([
            self.transformation_guess[0, 2],   # Initial x translation
            self.transformation_guess[1, 2],   # Initial y translation
            np.arctan2(self.transformation_guess[1, 0], self.transformation_guess[0, 0])  # Initial theta
        ])

        data_buffer = np.zeros((5, self.max_num_nodes))
        data_buffer[:, :input_data.shape[1]] = input_data

        # Solve the optimization problem with new input data
        solution_result = self.solver(x0=initial_translation_rotation, p=data_buffer)

        opt_solution = solution_result['x'].full().flatten()

        # Extract optimized translation and rotation
        opt_translation_x = opt_solution[0]
        opt_translation_y = opt_solution[1]
        opt_theta = opt_solution[2]

        # Construct the full transformation matrix (3x3 for 2D transformations)
        output = np.eye(3)
        
        output[0, 0] = np.cos(opt_theta)
        output[0, 1] = -np.sin(opt_theta)
        output[1, 0] = np.sin(opt_theta)
        output[1, 1] = np.cos(opt_theta)

        output[0, 2] = opt_translation_x
        output[1, 2] = opt_translation_y

        self.transformation_guess = output

        return output