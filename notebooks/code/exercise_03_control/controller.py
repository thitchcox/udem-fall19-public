import numpy as np
import math

class Controller():
    def __init__(self):
        self.gain = 2.0
        self.cumulative_dist_error = 0.0
        self.cumulative_angle_error = 0.0
        pass

    def angle_control_commands(self, dist, angle):
        # Return the angular velocity in order to control the Duckiebot so that it follows the lane.
        # Parameters:
        #     dist: distance from the center of the lane. Left is negative, right is positive.
        #     angle: angle from the lane direction, in rad. Left is negative, right is positive.
        # Outputs:
        #     omega: angular velocity, in rad/sec. Right is negative, left is positive.
        
        # PD control
        if False:
            k_theta = 6

            # Set k_theta based on condition for critically damped kimenatics
            v_r = 0.5    # m / s, static
            k_d = pow(k_theta, 2) / (4 * v_r)

            # Formulate PD control law
            omega = k_d * dist + k_theta * angle
        
        # PID control
        if True:
            k_theta = 6

            # Set k_theta based on condition for critically damped kimenatics
            v_r = 0.5    # m / s, static
            k_d = pow(k_theta, 2) / (4 * v_r)

            # Integrate the error term
            self.cumulative_dist_error += dist
            self.cumulative_angle_error += angle
            
            k_i = 1.0
            
            # Formulate PID control law
            omega = k_d * dist + k_theta * angle + k_i * (self.cumulative_dist_error + self.cumulative_angle_error)
        
        return omega

    def pure_pursuit(self, env, pos, angle):
        # Return the angular velocity in order to control the Duckiebot using a pure pursuit algorithm.
        # Parameters:
        #     env: Duckietown simulator
        #     pos: global position of the Duckiebot
        #     angle: global angle of the Duckiebot
        # Outputs:
        #     v: linear veloicy in m/s.
        #     omega: angular velocity, in rad/sec. Right is negative, left is positive.
        
        closest_curve_point = env.unwrapped.closest_curve_point
        
        # Find the curve point closest to the agent, and the tangent at that point
        closest_point, closest_tangent = closest_curve_point(pos, angle)

        iterations = 0
        
        lookup_distance = 0.25
        multipler = 0.5
        curve_point = None
        
        while iterations < 10:            
                       
            follow_point = closest_point + lookup_distance * closest_tangent
            
            curve_point, _ = closest_curve_point(follow_point, angle)

            # If we have a valid point on the curve, stop
            if curve_point is not None:
                break

            iterations += 1
            lookup_distance *= multiplier
                 
        cos_theta = math.cos(angle)
        sin_theta = math.sin(angle)
        
        # Form DCM
        C_bw = np.matrix([[cos_theta, sin_theta], [-sin_theta, cos_theta]])
                
        # Get displacement in body frame
        curve_0 = np.asscalar(curve_point[0])
        curve_2 = np.asscalar(-curve_point[2])
        r_fa_w = np.matrix([[curve_0], [curve_2]])
        pos_0 = np.asscalar(pos[0])
        pos_2 = np.asscalar(-pos[2])
        r_za_w = np.matrix([[pos_0], [pos_2]])
        r_fz_b = C_bw * (r_fa_w - r_za_w)
        
        # Get sin(alpha)
        sin_alpha = r_fz_b[1] / lookup_distance
        
        # TODO: Set v as a function of sin_alpha(?)
        v = 0.5
        
        omega = (2 * v * sin_alpha) / lookup_distance
        
        return v, omega


