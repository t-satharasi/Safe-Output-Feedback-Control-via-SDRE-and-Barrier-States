#!/usr/bin/env python3
"""
SDRE based Robust-Safe Regulation ROS2 Node

This node implements a SDRE based Optimal controller for regulation while safely avoiding an obstacle using output feedback. It interfaces with MAVROS to
control PX4-Autopilot.

Adopted from "Saiedeh Akbari's Lyapunov Based Adaptive Langevin Controller"

Author: Trivikram Satharasi
"""

import numpy as np
import pandas as pd
import math
from scipy.spatial.transform import Rotation as R
import time
import traceback
import json
from typing import Optional, Tuple
from scipy.linalg import solve_continuous_are

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import asyncio
from geodesy import utm
import utm
import geodesy
import tf2_ros
from tf2_ros import TransformBroadcaster
from transforms3d.euler import euler2quat
import tf2_geometry_msgs

from mavros_msgs.msg import PositionTarget, State, Altitude, GlobalPositionTarget
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, TransformStamped
from geographic_msgs.msg import GeoPose, GeoPoseStamped, GeoPoint
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

# import LyLA funcs

from . import data_manager

class SDREBasedSafeRegulation(Node):
    """ROS2 Node implementing SDRE based Robust Safe Regulation."""

    def __init__(self):
        #-------------------- check the folder name at autonomy park
        super().__init__('lyla_controller')

        self.init_states()
        self.init_params()
        self.init_topics()
        self.init_clients()
       
        self.get_logger().info("SDRE Adaptive Node Initialized")

    def init_states(self) -> None:
        # Flight
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.target_position = np.zeros(3)
        self.target_velocity = np.zeros(3)
        self.control_input = np.zeros(3)
        self.orientation = 0.0
        self.quaternion = None
        self.global_pose = NavSatFix()
        self.altitude_amsl = -1.0
        # Plotting
        self.step = 1
        # Mavros
        self.mavros_state = None
        self.armed = False
        self.offboard_mode = False
        self.takeoff_mode = False
        self.VEL_MAX = 1.0
        self.HOME_X = -8.0
        self.HOME_Y = -0.3 
        self.tf = 1000.0
        self.INCLUDE_RMS = True
        self.INCLUDE_PLOTS = True

    def init_params(self) -> None:
        # Load park parameters for coordinate transforms
        self.declare_parameters(
            namespace='',
            parameters=[
                # Origin parameters
                ('origin_r', rclpy.parameter.Parameter.Type.DOUBLE),
                ('origin_x', rclpy.parameter.Parameter.Type.DOUBLE),
                ('origin_y', rclpy.parameter.Parameter.Type.DOUBLE),
                ('origin_z', rclpy.parameter.Parameter.Type.DOUBLE),
                ('utm_zone', rclpy.parameter.Parameter.Type.INTEGER),
                ('utm_band', rclpy.parameter.Parameter.Type.STRING),
            ]
        )

        self.origin_r = self.get_parameter('origin_r').value
        self.origin_x = self.get_parameter('origin_x').value
        self.origin_y = self.get_parameter('origin_y').value
        self.origin_z = self.get_parameter('origin_z').value
        self.utm_zone = self.get_parameter('utm_zone').value
        self.utm_band = self.get_parameter('utm_band').value
        

        # Check for missing parameters
        if (self.origin_r is None or self.origin_x is None or 
            self.origin_y is None or self.origin_z is None or 
            self.utm_zone is None or self.utm_band is None):
            raise RuntimeError("Missing required origin parameters")
        
        # Converting to quaternion
        self.q_apark_to_utm = euler_to_quaternion(0, 0, -self.origin_r)

        #Experiment Parameters:
        params = {}
        params["n"] = 4   # physical state: [x, y, xdot, ydot] in PX4 NED horizontal plane
        params["m"] = 2   # control: [ax, ay] in PX4 NED horizontal plane
        params["q"] = 2   # measurement: [x, y]

        params["Q"] = 20 * np.diag([0.05, 0.05, 0.05, 0.05, 2])
        params["R"] = 1.0 * np.identity(params["m"])
        params["gamma"] = 2.0
        params["K"] = 1.0

        # Altitude proportional gain. PX4 NED z is positive down.
        self.kp_z = 1.0

        params["M"] = 0.01 * np.identity(params["n"])
        params["alpha"] = 1.0
        params["SIG"] = 0.1*np.identity(params["q"])
        params["C"] = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0]
        ])

        params["theta_under"] = 2.0
        params["theta_over"] = 0.3
        params["x0_bar"] = 0.0
        params["L"] = 10.0

        params["obs_center"] = np.array([[-5.00, -0.0]]).T
        params["obs_radius"] = 1.5

        self.params = params

        #Initial Conditions:
        # Augmented observer state: [x, y, vx, vy, z_barrier]^T
        # Warning: Ensure that the Estimate is near the actual staring pose
        self.s_hat = np.array([[-8.5], [0], [0.0], [0.0], [0.0]])
        self.theta_obs = 0.01 * np.identity(self.params["n"])
        self.u = np.zeros((self.params["m"], 1))

        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.last_odom_time = None
        self.have_odom = False

        self.y_meas = np.zeros((2, 1))
        self.alt_measured = 0.0
        self.vel_measured = np.zeros(3)

        self.z_ref = 2.5
        self.ACCEL_MAX_XY = 2.0


    def init_topics(self) -> None:
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publishers
        self.vel_pub = self.create_publisher(TwistStamped, 'setpoint_velocity/cmd_vel', qos_profile=qos_profile_sensor_data)
        self.raw_setpoint_pub = self.create_publisher(
            PositionTarget,
            'setpoint_raw/local',
            qos_profile=qos_profile_sensor_data
        )
        self.globalposition_pub = self.create_publisher(
            GlobalPositionTarget,
            'setpoint_raw/global',
            qos_profile=qos_profile_sensor_data
        )
        # Rviz Publishers
        self.path_pub = self.create_publisher(Path, 'path/actual', 10)
        self.estimate_path_pub = self.create_publisher(Path, 'path/estimate', 10)
        self.path = Path()
        self.estimate_path = Path()
        
        # Subscribers
        self.pose_sub = self.create_subscription(PoseStamped, 'autonomy_park/pose', self.pose_callback, qos_profile=qos_profile_sensor_data)
        self.vel_sub = self.create_subscription(TwistStamped, 'local_position/velocity_local', self.velocity_callback, qos_profile=qos_profile_sensor_data)
        self.state_sub = self.create_subscription(State, 'state', self.state_callback, qos_profile=qos_profile_sensor_data)
        self.altitude_sub = self.create_subscription(Altitude, 'altitude', self.altitude_callback, qos_profile=qos_profile_sensor_data)
        self.global_pos_sub = self.create_subscription(NavSatFix, 'global_position/global', self.global_pose_callback, qos_profile=qos_profile_sensor_data)

        #Publish RViz Obstacle:
        self.pub_obstacle = self.create_publisher(Marker, '/obstacle', 10)


    def init_clients(self) -> None:
        # Service clients
        self.arming_client = self.create_client(CommandBool, 'cmd/arming')
        while not self.arming_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info(f'service {self.arming_client.srv_name} not available, waiting...')

        self.takeoff_client = self.create_client(CommandTOL, 'cmd/takeoff')
        while not self.takeoff_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info(f'service {self.takeoff_client.srv_name} not available, waiting...')

        self.set_mode_client = self.create_client(SetMode, 'set_mode')
        while not self.set_mode_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info(f'service {self.set_mode_client.srv_name} not available, waiting...')


     # ==================== CALLBACK METHODS ====================
    
    def pose_callback(self, msg: PoseStamped) -> None: 
        # Pose updates (APark)
        self.position[0] = msg.pose.position.x
        self.position[1] = msg.pose.position.y
        self.position[2] = msg.pose.position.z

        self.y_meas = np.array(self.position[:2]).reshape(2, 1).copy()

        self.quaternion = msg.pose.orientation
        self.orientation = quat_to_yaw(msg.pose.orientation)

        # Actual path
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.header.frame_id = "autonomy_park"

        msg.header.frame_id = "autonomy_park"
        self.path.poses.append(msg)
        self.path_pub.publish(self.path)


        #Publish obstacle
        marker = Marker()

        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "autonomy_park"

        marker.ns = "obstacles"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = float(self.params["obs_center"][0, 0])
        marker.pose.position.y = float(self.params["obs_center"][1, 0])
        marker.pose.position.z = 2.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 2.0 * self.params["obs_radius"]
        marker.scale.y = 2.0 * self.params["obs_radius"]
        marker.scale.z = 4.0

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.pub_obstacle.publish(marker)


    def global_pose_callback(self, msg: NavSatFix) -> None:
        # Global pose updates (LLA)
        self.global_pose = msg

    def altitude_callback(self, msg: Altitude) -> None:
        # Altitude update (global)
        self.altitude_amsl = msg.amsl
    
    def velocity_callback(self, msg: TwistStamped) -> None:
        # Velocity update (body-fixed)
        vel_east = msg.twist.linear.x
        vel_north = msg.twist.linear.y
        vel_up = msg.twist.linear.z

        # Convert to apark frame
        self.velocity[0] = math.cos(self.origin_r)*vel_east - math.sin(self.origin_r)*vel_north
        self.velocity[1] = math.sin(self.origin_r)*vel_east + math.cos(self.origin_r)*vel_north
        self.velocity[2] = vel_up
    
    def state_callback(self, msg: State) -> None:
        # Mavros state update
        self.mavros_state = msg
        self.armed = msg.armed
        self.offboard_mode = (msg.mode == "OFFBOARD")

    # ============================================================ #

    # ===================== Control Loop ========================= # 

    async def run_trajectory(self) -> None:
        self.get_logger().info("Starting SDRE safe regulation...")
        
        
        traj_start_time = self.get_clock().now()
        t = 0.0

        while rclpy.ok(): 
            try:
                self.prev_t = t
                t = (self.get_clock().now() - traj_start_time).nanoseconds / 1e9     

                if t > self.tf:
                    self.get_logger().info(f"Reached final time of {self.tf} seconds.")
                    break
                
                u = self.compute_control_input(t)

                # Ensure we have float values
                ax = float(u[0])
                ay = float(u[1])
		
                if (
                    np.linalg.norm(self.y_meas) < 1.0
                ):
                    self.get_logger().info(
                        f"Reached minimal actuation limits in {t:.3f} seconds."
                    )
                    self.HOME_X=0.0
                    self.HOME_Y=0.0
                    break
                # Send velocity command
                #self.send_command(vx, vy, vz, yaw=None, yaw_rate=None)

                # Send acceleration command
                self.send_accel_xy_position_z_command(
                    ax,
                    ay,
                    self.z_ref,
                    yaw=None,
                    yaw_rate=None
                ) 
                
                await self.sleep(0.01)
            
            except Exception as e:
                self.get_logger().error(f"Error in control loop: {e}")
                self.get_logger().error(f"Error details: {type(e)}") 

    def compute_control_input(self, t: float):
       
        x_hat = self.s_hat[:self.params["n"], :]
        z_hat = self.s_hat[self.params["n"], 0]
        y_hat = self.params["C"] @ x_hat

        s_hat_dot, theta_dot = self.observerUpdate(
            t,
            x_hat,
            z_hat,
            self.theta_obs,
            self.u,
            self.y_meas,
            y_hat,
        )
        dt= t-self.prev_t
        self.s_hat = self.s_hat + s_hat_dot * dt
        self.theta_obs = self.theta_obs + theta_dot * dt

        estimated_pose = PoseStamped()
        estimated_pose.header.stamp = self.get_clock().now().to_msg()
        estimated_pose.header.frame_id = "autonomy_park"

        estimated_pose.pose.position.x = float(self.s_hat[0, 0])
        estimated_pose.pose.position.y = float(self.s_hat[1, 0])
        estimated_pose.pose.position.z = float(self.position[2])
        estimated_pose.pose.orientation.w = 1.0

        self.estimate_path.header.stamp = estimated_pose.header.stamp
        self.estimate_path.header.frame_id = "autonomy_park"
        self.estimate_path.poses.append(estimated_pose)

        self.estimate_path_pub.publish(self.estimate_path)
       
        u = self.controlLaw(t, self.s_hat)

        u_norm = np.linalg.norm(u)
        if u_norm > self.ACCEL_MAX_XY:
            u = u * self.ACCEL_MAX_XY / u_norm

        self.u = u.copy()
        u= u.reshape(-1)
       
        # x = torch.tensor([self.position[0], self.position[1], self.position[2],
        #                     self.velocity[0], self.velocity[1], self.velocity[2]],
        #                     dtype=torch.float32)

        # # Convert t to tensor 
        # t_tensor = torch.tensor(t, dtype=torch.float32)
        # self.get_logger().info(f"Time: {t_tensor.item()}")
        
        # xd, xd_dot = LyLA_forROS.Dynamics.desired_trajectory(t_tensor)
        # u, Phi = controller.parameter_adaptation(x, t_tensor)

        # theta = torch.cat([p.view(-1) for p in controller.nn.parameters()])
        data_manager.save_theta_to_csv(self.step, t, self.theta_obs)

        # Data Storage
        estimated_position = np.array([self.s_hat[0,0],self.s_hat[1,0], self.position[2]])
        data_manager.save_state_to_csv(
            self.step, 
            t,  
            np.array([self.position[0],self.position[1],self.position[2]]),
            estimated_position,
            np.array([u[0],u[1],0.0]),
        )
        self.step = self.step + 0.01

        # Construct and broadcast TF2 for desired trajectory 
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "autonomy_park"  # Parent frame
        tf.child_frame_id = "target_position"  # Child frame
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = float(self.z_ref)
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf)
        
        return u
       
    def send_command(self, vel_x: float, vel_y: float, vel_z: float, yaw: Optional[float] = None, yaw_rate: Optional[float] = None) -> None:
        """Send velocity command to PX4"""

        # rotate from park frame to enu
        vx = math.cos(self.origin_r)*vel_x + math.sin(self.origin_r)*vel_y
        vy = -math.sin(self.origin_r)*vel_x + math.cos(self.origin_r)*vel_y

        cmdvel = TwistStamped()
        cmdvel.header.stamp = self.get_clock().now().to_msg()
        cmdvel.header.frame_id = "base_link"

        # saturate cmd velocity preserving direction to limit quadcopter speed
        cmdvel.twist.linear.x, cmdvel.twist.linear.y, cmdvel.twist.linear.z = saturate_vector(vx, vy, vel_z, self.VEL_MAX)
        cmdvel.twist.angular.z = 0.0

        self.vel_pub.publish(cmdvel)

    def send_accel_xy_position_z_command(
        self,
        acc_x: float,
        acc_y: float,
        z_ref: float,
        yaw: Optional[float] = None,
        yaw_rate: Optional[float] = None
        ) -> None:
        """Send mixed raw setpoint: ax, ay acceleration control and z position control."""

        # Rotate horizontal acceleration from autonomy_park frame to MAVROS local ENU frame.
        ax = math.cos(self.origin_r) * acc_x + math.sin(self.origin_r) * acc_y
        ay = -math.sin(self.origin_r) * acc_x + math.cos(self.origin_r) * acc_y

        # Saturate horizontal acceleration.
        axy_norm = math.hypot(ax, ay)
        if axy_norm > self.ACCEL_MAX_XY and axy_norm > 0.0:
            scale = self.ACCEL_MAX_XY / axy_norm
            ax *= scale
            ay *= scale

        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # Use:
        #   position.z
        #   acceleration_or_force.x
        #   acceleration_or_force.y
        #
        # Ignore:
        #   position.x, position.y
        #   velocity.x, velocity.y, velocity.z
        #   acceleration_or_force.z
        msg.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFZ
        )

        if yaw is None:
            msg.type_mask |= PositionTarget.IGNORE_YAW
        else:
            msg.yaw = float(yaw) + float(self.origin_r)

        if yaw_rate is None:
            msg.type_mask |= PositionTarget.IGNORE_YAW_RATE
        else:
            msg.yaw_rate = float(yaw_rate)
        msg.position.x  = 0.0
        msg.position.z = float(z_ref)
        msg.acceleration_or_force.x = float(ax) #float(min(max(float(ax),-2.0),2.0))
        msg.acceleration_or_force.y = float(ay) #float(min(max(float(ay),-2.0),2.0))
        msg.acceleration_or_force.z = 0.0

        self.raw_setpoint_pub.publish(msg)

    def send_accel_command(self, accel_x, accel_y, accel_z, yaw=None, yaw_rate=None):
        """Send acceleration and attitude command to PX4"""
        msg = PositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        
        # Determine which commands to ignore based on what's provided
        msg.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | \
                    PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_VX | \
                    PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ
        
        # Add yaw control if provided
        if yaw is not None:
            msg.yaw = yaw + self.origin_r
            msg.type_mask |= PositionTarget.IGNORE_YAW_RATE
        elif yaw_rate is not None:
            msg.yaw_rate = yaw_rate
            msg.type_mask |= PositionTarget.IGNORE_YAW
        else:
            # If neither yaw nor yaw_rate provided, ignore both
            msg.type_mask |= PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
        
        # Set acceleration values - ensure they're floats
        accel_enu_x = math.cos(self.origin_r)*accel_x + math.sin(self.origin_r)*accel_y
        accel_enu_y = -math.sin(self.origin_r)*accel_x + math.cos(self.origin_r)*accel_y
        
        # Saturate acceleration
        msg.acceleration_or_force.x, msg.acceleration_or_force.y, msg.acceleration_or_force.z = saturate_vector(accel_enu_x, accel_enu_y, accel_z, 2.0)
        
        # log to check control input
        #self.get_logger().info(f"ax: {msg.acceleration_or_force.x}, ay: {msg.acceleration_or_force.y}, az: {msg.acceleration_or_force.z}")
        
        
        self.accel_pub.publish(msg)

    def send_position_command(self, sp_x, sp_y, sp_z, yaw=None, yaw_rate=None):
        """
        Send position control command to PX4
        """
        # Apark co-ordinates to UTM 

        # Un-rotate setpoint coordinates
        dx = math.cos(self.origin_r) * sp_x + math.sin(self.origin_r) * sp_y
        dy = -math.sin(self.origin_r) * sp_x + math.cos(self.origin_r) * sp_y
        
        # Convert park coordinates to UTM
        utm_pos_easting = dx + self.origin_x
        utm_pos_northing = dy + self.origin_y
        
        # Convert UTM easting/northing to lat/long
        lat, lon = utm.to_latlon(utm_pos_easting, utm_pos_northing, self.utm_zone, self.utm_band)

        # Send Global Position Target
        msg = GlobalPositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        # 6 = FRAME_GLOBAL_REL_ALT (Altitude is relative to takeoff point)
        msg.coordinate_frame = 6 
        
        # Bitmask 4088 (0x0FFA) ignores velocity/acceleration, focusing only on position
        msg.type_mask = 4088 

        # Set your target coordinates
        msg.latitude = lat    # Example Latitude
        msg.longitude = lon     # Example Longitude
        msg.altitude = sp_z         # Target altitude in meters

        self.globalposition_pub.publish(msg)
        
    # ============================================================ #

    # ===================== Client Calls ========================= # 

    async def arm(self):
        """Arm the vehicle"""

        last_request_time = self.get_clock().now()
        
        while rclpy.ok():
            current_time = self.get_clock().now()
            
            if not self.armed and (current_time - last_request_time).nanoseconds > 2e9:
                self.get_logger().info("Trying to arm...")
                req = CommandBool.Request()
                req.value = True
                future = self.arming_client.call_async(req)
                await self.spin_until_future_complete(future)
                
                if future.result().success:
                    self.get_logger().info("Vehicle armed")
                    return True
                last_request_time = self.get_clock().now()
                
            self.send_command(0.0, 0.0, 0.0, 0.0, 0.0)  # Send neutral commands while waiting
            await self.sleep(0.05)
    
    async def set_offboard(self):
        """Set to offboard mode"""

        # Send a few setpoints before starting
        for i in range(100):
            self.send_command(0.0, 0.0, 0.0, 0.0, 0.0)
            await self.sleep(0.05)

        last_request_time = self.get_clock().now()

        
        while rclpy.ok():
            current_time = self.get_clock().now()
            
            if not self.offboard_mode and (current_time - last_request_time).nanoseconds > 2e9:
                self.get_logger().info("Trying to set OFFBOARD mode...")
                req = SetMode.Request()
                req.custom_mode = "OFFBOARD"
                future = self.set_mode_client.call_async(req)
                await self.spin_until_future_complete(future)
                
                if future.result().mode_sent:
                    self.get_logger().info("OFFBOARD mode set")
                    self.offboard_mode = True
                    return True
                last_request_time = self.get_clock().now()
                
            self.send_command(0.0, 0.0, 0.0, 0.0, 0.0)  # Send neutral commands while waiting
            await self.sleep(0.05)
    
    async def takeoff(self, height: float):
        """Simple takeoff procedure"""
        last_request_time = self.get_clock().now()
        
        while rclpy.ok():
            current_time = self.get_clock().now()

            if not self.takeoff_mode and (current_time - last_request_time).nanoseconds > 2e9:
                self.get_logger().info(f"Trying to takeoff to {height} meters...")

                takeoff_pose = Pose()
                takeoff_pose.position = Point(
                    x = float(self.position[0]),
                    y = float(self.position[1]),
                    z = float(self.position[2]))
                global_pose = self.apark_to_global(apark_pose=takeoff_pose)

                # convert local takeoff (apark frame) to global (lat/long)
                req = CommandTOL.Request()
                req.min_pitch = 0.0
                req.yaw = quat_to_yaw(quat = global_pose.orientation)
                req.latitude = self.global_pose.latitude
                req.longitude = self.global_pose.longitude
                req.altitude = self.altitude_amsl - self.position[2] + height
                
                self.get_logger().info(f"lat: {req.latitude}, long: {req.longitude}")
                future = self.takeoff_client.call_async(req)
                await self.spin_until_future_complete(future)

                if future.result().success:
                    self.get_logger().info(f"Taking off to {req.altitude} meters.")
                    self.takeoff_mode = True
                    return True
                
            await self.sleep(0.02)

        self.get_logger().info("Finished Taking off")
    

    async def return_home(self) -> None:
        # create velocity setpoint msg
        setpoint_vel = TwistStamped()
        setpoint_vel.header.stamp = self.get_clock().now().to_msg()
        setpoint_vel.header.frame_id = "base_link"

        ex = self.HOME_X - self.position[0]
        ey = self.HOME_Y - self.position[1]

                
        # small p controller to get drone near home pose
        while (math.sqrt(ex**2 + ey**2) >= 1):

            ex = self.HOME_X - self.position[0]
            ey = self.HOME_Y - self.position[1]
            
            k = 0.3

            vel_x = k*ex
            vel_y = k*ey
            vel_z = 0.0
            self.get_logger().info(f"Going to Home, Error: {ex}, {ey}, Velocity: {vel_x}, {vel_y} ")
            vx = math.cos(self.origin_r)*vel_x + math.sin(self.origin_r)*vel_y
            vy = -math.sin(self.origin_r)*vel_x + math.cos(self.origin_r)*vel_y
            setpoint_vel.twist.linear.x, setpoint_vel.twist.linear.y, setpoint_vel.twist.linear.z = saturate_vector(vx, vy, vel_z, self.VEL_MAX)
            setpoint_vel.twist.angular.z = 0.0

            self.vel_pub.publish(setpoint_vel)
            await self.sleep(0.01)

    async def return_home_poscontrol(self) -> None:
        # create velocity setpoint msg
        self.get_logger().info("Returning Home")

        setpoint_vel = TwistStamped()
        setpoint_vel.header.stamp = self.get_clock().now().to_msg()
        setpoint_vel.header.frame_id = "base_link"

        ex = self.HOME_X - self.position[0]
        ey = self.HOME_Y - self.position[1]

                
        # small p controller to get drone near home pose
        while (math.sqrt(ex**2 + ey**2) >= 1):

            ex = self.HOME_X - self.position[0]
            ey = self.HOME_Y - self.position[1]
            
            self.send_position_command(self.HOME_X, self.HOME_Y, self.z_ref)
            await self.sleep(0.01)
    
    async def land(self):
        """Simple landing procedure"""
        self.get_logger().info("Landing...")
        
        # Switch to land mode
        req = SetMode.Request()
        req.custom_mode = "AUTO.LAND"
        future = self.set_mode_client.call_async(req)
        await self.spin_until_future_complete(future)
        
        if future.result().mode_sent:
            self.get_logger().info("AUTO.LAND mode set")
            
        # Wait for landing
        while self.armed and rclpy.ok():
            await self.sleep(0.5)
            
        self.get_logger().info("Landing complete")
    
    # ============================================================ #


    # ============================================================ #
    async def sleep(self, seconds: float) -> None:
        """Sleep while still processing callbacks"""
        start = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            if (self.get_clock().now() - start).nanoseconds / 1e9 > seconds:
                break
    
    async def spin_until_future_complete(self, future):
        """Spin until future is complete"""
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.01)
        return future.result()
    
    async def run_mission(self) -> None:
        """Run the complete mission"""
        try:
                # Arm first
                await self.arm()
                
                # Send some neutral commands to stabilize
                for i in range(80):
                    self.send_command(0.0, 0.0, 0.0, 0.0, 0.0)
                    await self.sleep(0.01)
                
                # Take off next
                await self.takeoff(height=2.5)
                
                # Set offboard mode after takeoff
                await self.set_offboard()

                # return to home
                await self.return_home_poscontrol()

                # Send some neutral commands to stabilize
                for i in range(80):
                    self.send_command(0.0, 0.0, 0.0, 0.0, 0.0)
                    await self.sleep(0.01)
                
                # Run trajectory
                await self.run_trajectory()

                # return to home
                await self.return_home_poscontrol()

        except Exception as e:
            self.get_logger().error(f"Error in mission: {e}")
            # Print more details about the error
            self.get_logger().error(traceback.format_exc())

        finally:
            # Land when done or if interrupted
            await self.land()
            
            if self.INCLUDE_RMS or self.INCLUDE_PLOTS:
                self.print_results()


    def apark_to_global(self, apark_pose: Pose) -> GeoPose:
        # Autonomy park setpoint coordinates
        sp_x = apark_pose.position.x
        sp_y = apark_pose.position.y
        
        # Un-rotate setpoint coordinates
        dx = math.cos(self.origin_r) * sp_x + math.sin(self.origin_r) * sp_y
        dy = -math.sin(self.origin_r) * sp_x + math.cos(self.origin_r) * sp_y
        
        # Convert park coordinates to UTM
        utm_pos = geodesy.utm.UTMPoint()
        utm_pos.zone = self.utm_zone
        utm_pos.band = self.utm_band
        utm_pos.easting = dx + self.origin_x
        utm_pos.northing = dy + self.origin_y
        
        # Convert UTM easting/northing to lat/long
        lat, lon = utm.to_latlon(self.origin_x, self.origin_y, self.utm_zone, self.utm_band)
        global_pos = GeoPoint(
            latitude = lat,
            longitude = lon,
            altitude = self.altitude_amsl
            )
        
        # IMPORTANT: Command altitude is AMSL! (feedback is WGS-84 ellipsoid)
        global_pos.altitude = self.altitude_amsl
        
        # Finally, compute global orientation
        q_utm = multiply_quaternions(q1 = self.q_apark_to_utm, q2 = apark_pose.orientation)
        
        global_pose = GeoPose(
            position = global_pos,
            orientation = q_utm)
        
        return global_pose
# --------------------- check these folder names in autonomy park computer and update
    def print_results(self) -> None:
        state_data = pd.read_csv('src/lyla_controller/simulation_data/state_data.csv')
        target_state_data = pd.read_csv('src/lyla_controller/simulation_data/target_state_data.csv')
        time_array = target_state_data['Time']

        if self.INCLUDE_RMS:
            tracking_error_norm = state_data['Tracking_Error_Norm']
            rms_tracking_error = np.sqrt(np.mean(tracking_error_norm**2))
            self.get_logger().info(f'Mean RMS Tracking Error: {rms_tracking_error} m')

        if self.INCLUDE_PLOTS:
            data_manager.plot_from_csv()
#-- Experiment Functions 
    def SDCMatrix(self, x):
        I2 = np.eye(2)
        O2 = np.zeros((2, 2))
        A = np.block([
            [np.zeros((2, 2)), I2],
            [O2, O2]
        ])
        return A

    def g(self, x):
        B = np.array([
            [0.0, 0.0],
            [0.0, 0.0],
            [1.0, 0.0],
            [0.0, 1.0]
        ])
        return B

    def f(self, x):
        return self.SDCMatrix(x) @ x

    def constraint(self, x):
        x = np.asarray(x).reshape(-1)
        obs_center = np.asarray(self.params["obs_center"]).reshape(-1)
        obs_radius = self.params["obs_radius"]
        d = x[:2] - obs_center
        return d @ d - obs_radius ** 2

    def gradh(self, x):
        x = np.asarray(x).reshape(-1)
        c = self.params["obs_center"].reshape(-1)

        grad_pos = 2.0 * (x[:2] - c)
        grad_vel = np.zeros(2)

        grad = np.hstack([grad_pos, grad_vel])
        return np.atleast_2d(grad)

    def gradb(self, x):
        K = self.params["K"]
        return -K / (x ** 2)

    def Phi(self, x):
        K = self.params["K"]
        if K == 0:
            return 0.0
        return -(x ** 2) / K

    def beta(self, x):
        K = self.params["K"]
        h = self.constraint(x)
        return K / h

    def betaHat(self, t, x):
        K = self.params["K"]
        h = self.constraint(x)
        eps = self.errorBound(t)
        denom = h - eps
        if abs(denom) < 1e-9:
            denom = np.sign(denom) * 1e-9 if denom != 0 else 1e-9
        return K / denom

    def errorBound(self, t):
        p = self.params
        eps = (
            p["L"]
            * np.sqrt(p["theta_over"] / p["theta_under"])
            * p["x0_bar"]
            * np.exp(-np.max(np.real(np.linalg.eigvals(p["M"]))) * t)
        )
        return eps

    def barrierDynamics(self, x, z, u):
        beta0 = self.beta(np.zeros_like(x))
        beta_x = self.beta(x)
        phi = self.Phi(z + beta0)
        grad = self.gradh(x)

        z_dot = (
            phi * grad @ (self.f(x) + self.g(x) @ u)
            - self.params["gamma"] * (z + beta0 - beta_x)
        )
        return z_dot

    def A(self, t, s_hat):
        n = self.params["n"]

        s_hat = np.asarray(s_hat).reshape(-1, 1)
        x_hat = s_hat[:n, :]
        z_hat = s_hat[n, 0]

        beta0t = self.betaHat(t, np.zeros_like(x_hat))
        gamma = self.params["gamma"]

        A_x = self.SDCMatrix(x_hat)
        A_z = (
            self.Phi(z_hat + beta0t) * self.gradh(x_hat) @ A_x
            + gamma * self.Psi(t, x_hat)
        )
        A_z = np.atleast_2d(A_z)

        A_hat = np.block([
            [A_x, np.zeros((n, 1))],
            [A_z, np.array([[-gamma]])]
        ])
        return A_hat

    def controlLaw(self, t, s_hat):
        Q = self.params["Q"]
        R = self.params["R"]

        s_hat = np.asarray(s_hat).reshape(-1, 1)
        A1 = self.A(t, s_hat)
        G1 = self.AugmentedControlEffect(t, s_hat)

        try:
            P = solve_continuous_are(A1, G1, Q, R)
            u = -np.linalg.solve(R, (G1.T @ P @ s_hat))
        except Exception as e:
            self.get_logger().warn(f'CARE solve failed: {e}')
            u = np.zeros((self.params["m"], 1))

        return u

    def AugmentedControlEffect(self, t, s_hat):
        n = self.params["n"]

        s_hat = np.asarray(s_hat).reshape(-1, 1)
        x_hat = s_hat[:n, :]
        z_hat = s_hat[n, 0]

        beta0t = self.betaHat(t, np.zeros_like(x_hat))
        gx = self.g(x_hat)

        bottom = self.Phi(z_hat + beta0t) * self.gradh(x_hat) @ gx
        bottom = np.atleast_2d(bottom)

        G_hat = np.vstack([gx, bottom])
        return G_hat

    def AugmentedDrift(self, t, s_hat):
        n = self.params["n"]

        s_hat = np.asarray(s_hat).reshape(-1, 1)
        x_hat = s_hat[:n, :]
        z_hat = s_hat[n, 0]

        beta0t = self.betaHat(t, np.zeros_like(x_hat))
        beta_hat = self.betaHat(t, x_hat)
        gamma = self.params["gamma"]

        fx = self.f(x_hat)

        bottom = (
            self.Phi(z_hat + beta0t) * self.gradh(x_hat) @ fx
            - gamma * (z_hat + beta0t - beta_hat)
        )
        bottom = np.asarray(bottom).reshape(1, 1)

        drift = np.vstack([fx, bottom])
        return drift

    def Psi(self, t, x_hat):
        n = self.params["n"]
        beta0t = self.betaHat(t, np.zeros_like(x_hat))
        x_hat = np.asarray(x_hat).reshape(-1)
        obs = np.asarray(self.params["obs_center"]).reshape(-1)
        K = self.params["K"]

        if K == 0:
            return np.zeros((1, n))

        beta_hat_val = self.betaHat(t, x_hat.reshape(-1, 1))
        psi_pos = (
            (1.0 / K)
            * beta_hat_val
            * beta0t
            * np.array([[-x_hat[0] + 2 * obs[0], -x_hat[1] + 2 * obs[1]]])
        )
        psi_vel = np.zeros(n - 2)
        out = np.hstack([psi_pos.ravel(), psi_vel])
        return np.atleast_2d(out)

    def observerUpdate(self, t, x_hat, z_hat, theta, u, y, y_hat):
        p = self.params

        x_hat = np.asarray(x_hat).reshape(-1, 1)
        u = np.asarray(u).reshape(-1, 1)
        y = np.asarray(y).reshape(-1, 1)
        y_hat = np.asarray(y_hat).reshape(-1, 1)
        theta = np.asarray(theta)

        C = np.asarray(p["C"])
        if C.ndim == 1:
            C = C.reshape(1, -1)

        K_theta = theta.T @ C.T @  np.linalg.inv(p["SIG"])

        beta0t = self.betaHat(t, np.zeros_like(x_hat))
        e_y = y - y_hat
        phi = self.Phi(z_hat + beta0t)
        grad = self.gradh(x_hat)

        # Use full-state origin consistent with n=4
        h0 = self.constraint(np.zeros((p["n"], 1)))
        beta_prime = self.gradb(h0 - self.errorBound(t))

        lam_max = np.max(np.real(np.linalg.eigvals(p["M"])))
        rho = (
            phi * self.errorBound(t) * (-lam_max)
            + beta_prime * self.errorBound(t) * (-lam_max)
        )

        correction = np.vstack([
            K_theta @ e_y,
            np.asarray(-rho + phi * grad @ (K_theta @ e_y)).reshape(1, 1)
        ])

        s_hat = np.vstack([x_hat, np.array([[z_hat]])])

        drift = self.AugmentedDrift(t, s_hat)
        control = self.AugmentedControlEffect(t, s_hat) @ u
        s_hat_dot = drift + control + correction

        A_x = self.SDCMatrix(x_hat)
        term1 = (A_x + p["alpha"] * np.eye(p["n"])) @ theta
        term2 = theta @ (A_x.T + p["alpha"] * np.eye(p["n"]))
        term3 = theta @ C.T @ (np.linalg.inv(p["SIG"]) @ (C @ theta))
        theta_dot = term1 + term2 - term3 + p["M"]

        return s_hat_dot, theta_dot

def saturate_vector(vec_x: float, vec_y: float, vec_z: float, max_magnitude: float) -> Tuple[float, float, float]:
    """
    Saturate a 3D vector while preserving its direction.
    
    Args:
        vec_x, vec_y, vec_z: Vector components
        max_magnitude: Maximum allowed magnitude
        
    Returns:
        Tuple of saturated (x, y, z) components
    """
    # Calculate current magnitude
    magnitude = math.sqrt(vec_x**2 + vec_y**2 + vec_z**2)
    
    # If magnitude exceeds limit, scale the vector down
    if magnitude > max_magnitude and magnitude > 0:
        scaling_factor = max_magnitude / magnitude
        return (vec_x * scaling_factor, 
                vec_y * scaling_factor, 
                vec_z * scaling_factor)
    else:
        return (vec_x, vec_y, vec_z)

def quat_to_yaw(quat: Quaternion) -> float:
    siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
    cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw

def multiply_quaternions(q1: Quaternion, q2: Quaternion) -> Quaternion:
    """Multiply two geometry_msgs.msg.Quaternion quaternions in ROS2."""
    # Convert ROS2 Quaternions to scipy Rotation objects
    r2 = R.from_quat([q2.x, q2.y, q2.z, q2.w])
    r1 = R.from_quat([q1.x, q1.y, q1.z, q1.w])

    # Multiply rotations
    r_result = r1 * r2  # Equivalent to quaternion multiplication

    # Convert back to a geometry_msgs Quaternion
    x, y, z, w = r_result.as_quat()
    return Quaternion(x=x, y=y, z=z, w=w)

def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert Euler angles (roll, pitch, yaw) to a geometry_msgs.msg.Quaternion."""
    r = R.from_euler('xyz', [roll, pitch, yaw])  # 'xyz' means rotation order
    x, y, z, w = r.as_quat()  # Convert to (x, y, z, w) format
    return Quaternion(x=x, y=y, z=z, w=w)

    
def main(args=None):
    rclpy.init(args=args)
    
    sdre_regulation = SDREBasedSafeRegulation()
    
    # Create the event loop
    loop = asyncio.get_event_loop()
    
    try:
        # Run the async method in the event loop
        loop.run_until_complete(sdre_regulation.run_mission())
        

    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        sdre_regulation.destroy_node()
        rclpy.shutdown()
        loop.close()

if __name__ == '__main__':
    main()
