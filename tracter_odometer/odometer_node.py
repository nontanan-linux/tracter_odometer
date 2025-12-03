import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np
import math
from .kinematic_model import TractorTrailerModel
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import math

class TractorOdometer(Node):
    def __init__(self):
        super().__init__('tracter_odometer')
        
        # Parameters
        self.declare_parameter('L0', 2.0)
        self.declare_parameter('dt', 0.05)
        self.declare_parameter('trailer_L_bars', [1.0, 1.0, 1.0, 1.0])
        self.declare_parameter('trailer_L_trls', [1.2, 1.2, 1.2, 1.2])
        
        # Hitch Calculation Parameters
        self.declare_parameter('tractor_tail_length', 0.15)
        self.declare_parameter('trailer_tail_length', 0.15)
        
        # Color Parameters
        self.declare_parameter('color_tractor', [1.0, 0.27, 0.0, 0.5])
        self.declare_parameter('color_trailer', [0.0, 0.0, 1.0, 0.5])
        self.declare_parameter('color_wheel', [1.0, 1.0, 1.0, 1.0])
        self.declare_parameter('color_drawbar', [0.8, 0.9, 1.0, 1.0])
        
        # Dimension Parameters
        self.declare_parameter('tractor_width', 1.5)
        self.declare_parameter('tractor_front_overhang', 0.1)
        self.declare_parameter('tractor_overhang', 0.4)
        self.declare_parameter('trailer_width', 1.5)
        self.declare_parameter('trailer_body_len', 2.0)
        self.declare_parameter('trailer_overhang', 0.4)
        self.declare_parameter('wheel_diam', 0.6)
        self.declare_parameter('wheel_width', 0.3)
        self.declare_parameter('track_width', 1.0)
        
        self.L0 = self.get_parameter('L0').value
        dt = self.get_parameter('dt').value
        
        l_bars = self.get_parameter('trailer_L_bars').value
        l_trls = self.get_parameter('trailer_L_trls').value
        
        # Visualization Parameters (Needed for hitch calc)
        self.tractor_width = self.get_parameter('tractor_width').value
        self.tractor_front_overhang = self.get_parameter('tractor_front_overhang').value
        self.tractor_overhang = self.get_parameter('tractor_overhang').value
        
        # Calculate Tractor Length dynamically
        # Length = Rear Overhang + Wheelbase + Front Overhang
        self.tractor_len = self.tractor_overhang + self.L0 + self.tractor_front_overhang
        
        self.trailer_width = self.get_parameter('trailer_width').value
        self.trailer_body_len = self.get_parameter('trailer_body_len').value
        self.trailer_overhang = self.get_parameter('trailer_overhang').value
        self.wheel_diam = self.get_parameter('wheel_diam').value
        self.wheel_width = self.get_parameter('wheel_width').value
        self.W = self.get_parameter('track_width').value
        
        # Calculate Hitch Offsets
        tractor_tail = self.get_parameter('tractor_tail_length').value
        trailer_tail = self.get_parameter('trailer_tail_length').value
        
        tractor_hitch_offset = self.tractor_overhang + tractor_tail
        trailer_hitch_offset = self.trailer_overhang + trailer_tail
        
        # Combine hitch offsets: [tractor_hitch, trailer_hitch, trailer_hitch, ...]
        # Note: The last trailer's hitch offset is not used for a next trailer, but we need N entries.
        # Actually, for N trailers, we need N dh_prev values.
        # dh_prev[0] (Trailer 1) = Tractor Hitch Offset
        # dh_prev[i] (Trailer i+1) = Trailer i Hitch Offset
        
        dh_prevs = [tractor_hitch_offset]
        if len(l_bars) > 1:
            dh_prevs.extend([trailer_hitch_offset] * (len(l_bars) - 1))
        
        self.c_tractor = self.get_parameter('color_tractor').value
        self.c_trailer = self.get_parameter('color_trailer').value
        self.c_wheel = self.get_parameter('color_wheel').value
        self.c_drawbar = self.get_parameter('color_drawbar').value
        

        
        # Construct trailers list
        self.trailers = []
        if len(l_bars) == len(l_trls) == len(dh_prevs):
            for i in range(len(l_bars)):
                self.trailers.append({
                    'L_bar': l_bars[i],
                    'L_trl': l_trls[i],
                    'dh_prev': dh_prevs[i]
                })
        else:
            self.get_logger().error("Trailer parameter arrays must have equal length!")
        
        self.model = TractorTrailerModel(self.L0, self.trailers, dt=dt)
        
        # Visualization Parameters (matching simulate.py)
        self.tractor_width = 1.5
        self.tractor_len = 2.8
        self.tractor_overhang = 0.4
        self.trailer_width = 1.5
        self.trailer_body_len = 2.0
        self.trailer_overhang = 0.4
        self.wheel_diam = 0.6
        self.wheel_width = 0.3
        self.W = 1.0 # Track Width
        
        # State: [x0, y0, theta0, theta_db1, theta_tr1, ...]
        self.state = np.zeros(3 + 2 * len(self.trailers))
        
        # Publishers/Subscribers
        # Change input from Twist (/cmd_vel) to Odometry (/localization/kinematic_state)
        self.sub_odom = self.create_subscription(Odometry, '/localization/kinematic_state', self.odom_callback, 10)
        self.pub_joints = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.timer = self.create_timer(dt, self.update)
        
        self.v0 = 0.0
        self.delta = 0.0
        
        self.get_logger().info(f'Tractor Odometer Started with {len(self.trailers)} trailers')

    def odom_callback(self, msg):
        # Extract velocity and steering from Odometry
        self.v0 = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        
        if abs(self.v0) > 0.1:
            self.delta = math.atan(omega * self.L0 / self.v0)
        else:
            self.delta = 0.0
            
        # Extract Pose (x, y, theta)
        self.state[0] = msg.pose.pose.position.x
        self.state[1] = msg.pose.pose.position.y
        
        # Quaternion to Yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.state[2] = math.atan2(siny_cosp, cosy_cosp)

    def update(self):
        # Update Model
        # Note: We overwrite the tractor state with Odom data in callback.
        # To ensure the tractor stays at the Odom position (and isn't integrated/predicted),
        # we save the current Odom state and restore it after the model update.
        
        # Save current tractor state (from Odom)
        tractor_state_odom = self.state[0:3].copy()
        
        # Update model (integrates everything, including tractor)
        new_state = self.model.update(self.state, self.v0, self.delta)
        
        # Update state with new values
        self.state = new_state
        
        # Restore tractor state to the ground truth (from Odom)
        # This ensures the tractor visualization matches the topic exactly.
        self.state[0:3] = tractor_state_odom
        
        # Get Coordinates for all units
        coords = self.model.get_coordinates(self.state)
        # coords = [p0, p0_f, h1, p1, p2, h2, p3, p4, ...]
        
        # Broadcast TFs
        # Tractor (p0) -> We assume 'map' -> 'base_link' is provided by Autoware.
        # But we need to publish trailers relative to map or base_link.
        # Since we have global coords, let's publish map -> trailer_i
        
        timestamp = self.get_clock().now().to_msg()
        
        # --- TF Broadcasting ---
        # Helper to broadcast
        def broadcast_tf(x, y, theta, child_frame, parent_frame='map'):
            t = TransformStamped()
            t.header.stamp = timestamp
            t.header.frame_id = parent_frame
            t.child_frame_id = child_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            
            q = self.euler_to_quaternion(0, 0, theta)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            
            self.tf_broadcaster.sendTransform(t)

        # Broadcast Trailers
        for i in range(len(self.trailers)):
            # Indices in coords:
            # Trailer i (0-indexed):
            # p_dolly (p_{2i+1}): 3 + 3*i
            # p_axle (p_{2i+2}): 4 + 3*i
            
            idx_dolly = 3 + 3*i
            idx_axle = 4 + 3*i
            
            p_dolly = coords[idx_dolly]
            p_axle = coords[idx_axle]
            
            # Angles
            theta_drawbar = self.state[3 + 2*i]
            theta_trailer = self.state[3 + 2*i + 1]
            
            # Broadcast Dolly (Drawbar end)
            broadcast_tf(p_dolly[0], p_dolly[1], theta_drawbar, f'trailer_{i+1}_dolly')
            
            # Broadcast Trailer Axle
            broadcast_tf(p_axle[0], p_axle[1], theta_trailer, f'trailer_{i+1}_base_link')
        
        # --- Marker Visualization ---
        marker_array = MarkerArray()
        marker_id = 0
        
        def create_cube_marker(x, y, theta, lx, ly, lz, r, g, b, a=1.0, frame_id='map'):
            nonlocal marker_id
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = timestamp
            m.ns = "shapes"
            m.id = marker_id
            marker_id += 1
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = lz/2.0
            q = self.euler_to_quaternion(0, 0, theta)
            m.pose.orientation.x = q[0]
            m.pose.orientation.y = q[1]
            m.pose.orientation.z = q[2]
            m.pose.orientation.w = q[3]
            m.scale.x = lx
            m.scale.y = ly
            m.scale.z = lz
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = a
            return m

        def create_line_marker(p1, p2, width, r, g, b, a=1.0, frame_id='map'):
            nonlocal marker_id
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = timestamp
            m.ns = "lines"
            m.id = marker_id
            marker_id += 1
            m.type = Marker.LINE_LIST
            m.action = Marker.ADD
            m.scale.x = width
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = a
            pt1 = Point(); pt1.x = p1[0]; pt1.y = p1[1]; pt1.z = 0.2
            pt2 = Point(); pt2.x = p2[0]; pt2.y = p2[1]; pt2.z = 0.2
            m.points.append(pt1)
            m.points.append(pt2)
            return m

        def create_sphere_marker(x, y, radius, r, g, b, a=1.0, frame_id='map'):
            nonlocal marker_id
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = timestamp
            m.ns = "points"
            m.id = marker_id
            marker_id += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.2
            m.pose.orientation.w = 1.0
            m.scale.x = radius
            m.scale.y = radius
            m.scale.z = radius
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = a
            return m

        # Tractor Body
        p0 = coords[0]
        p0_f = coords[1]
        theta0 = self.state[2]
        
        # Calculate center based on Overhang and Length
        # Rear Face is at -overhang from Rear Axle (p0)
        # Front Face is at -overhang + length from Rear Axle
        # Center is at (-overhang + (-overhang + length)) / 2 = length/2 - overhang
        
        tr_center_offset = self.tractor_len / 2.0 - self.tractor_overhang
        p_tractor_c = p0 + tr_center_offset * np.array([np.cos(theta0), np.sin(theta0)])
        
        marker_array.markers.append(create_cube_marker(p_tractor_c[0], p_tractor_c[1], theta0, 
                                                       self.tractor_len, self.tractor_width, 1.0, 
                                                       self.c_tractor[0], self.c_tractor[1], self.c_tractor[2], self.c_tractor[3]))
        
        # Tractor Wheels
        def add_wheels(center, angle, steered_angle=0.0):
            wl_pos = center + (self.W/2) * np.array([-np.sin(angle), np.cos(angle)])
            wr_pos = center - (self.W/2) * np.array([-np.sin(angle), np.cos(angle)])
            wa = angle + steered_angle
            marker_array.markers.append(create_cube_marker(wl_pos[0], wl_pos[1], wa, self.wheel_diam, self.wheel_width, 0.6, 
                                                           self.c_wheel[0], self.c_wheel[1], self.c_wheel[2], self.c_wheel[3]))
            marker_array.markers.append(create_cube_marker(wr_pos[0], wr_pos[1], wa, self.wheel_diam, self.wheel_width, 0.6, 
                                                           self.c_wheel[0], self.c_wheel[1], self.c_wheel[2], self.c_wheel[3]))
            
        add_wheels(p0, theta0) # Rear
        add_wheels(p0_f, theta0, self.delta) # Front

        # Tractor Tail
        p_tr_rear_face = p0 - self.tractor_overhang * np.array([np.cos(theta0), np.sin(theta0)])
        h1 = coords[2]
        marker_array.markers.append(create_line_marker(p_tr_rear_face, h1, 0.05, 
                                                       self.c_drawbar[0], self.c_drawbar[1], self.c_drawbar[2], self.c_drawbar[3]))

        # Trailers
        for k in range(len(self.trailers)):
            idx_h = 2 + 3*k
            idx_dolly = 3 + 3*k
            idx_axle = 4 + 3*k
            
            h_curr = coords[idx_h]
            p_dolly = coords[idx_dolly]
            p_axle = coords[idx_axle]
            
            theta_drawbar = self.state[3 + 2*k]
            theta_trailer = self.state[3 + 2*k + 1]
            
            # Hitch Point
            marker_array.markers.append(create_sphere_marker(h_curr[0], h_curr[1], 0.15, 
                                                             self.c_drawbar[0], self.c_drawbar[1], self.c_drawbar[2], self.c_drawbar[3]))
            
            # Drawbar
            marker_array.markers.append(create_line_marker(h_curr, p_dolly, 0.05, 
                                                           self.c_drawbar[0], self.c_drawbar[1], self.c_drawbar[2], self.c_drawbar[3]))
            
            # Trailer Body
            # Calculate center based on Overhang and Body Length relative to Axle (p_axle)
            tl_center_offset = self.trailer_body_len / 2.0 - self.trailer_overhang
            p_trailer_c = p_axle + tl_center_offset * np.array([np.cos(theta_trailer), np.sin(theta_trailer)])
            
            marker_array.markers.append(create_cube_marker(p_trailer_c[0], p_trailer_c[1], theta_trailer,
                                                           self.trailer_body_len, self.trailer_width, 1.0, 
                                                           self.c_trailer[0], self.c_trailer[1], self.c_trailer[2], self.c_trailer[3]))
            
            # Wheels
            add_wheels(p_dolly, theta_drawbar)
            add_wheels(p_axle, theta_trailer)
            
            # Tail
            p_tl_rear_face = p_axle - self.trailer_overhang * np.array([np.cos(theta_trailer), np.sin(theta_trailer)])
            if k < len(self.trailers) - 1:
                h_next = coords[2 + 3*(k+1)]
                marker_array.markers.append(create_line_marker(p_tl_rear_face, h_next, 0.05, 
                                                               self.c_drawbar[0], self.c_drawbar[1], self.c_drawbar[2], self.c_drawbar[3]))
            else:
                p_stub = p_tl_rear_face - 0.1 * np.array([np.cos(theta_trailer), np.sin(theta_trailer)])
                marker_array.markers.append(create_line_marker(p_tl_rear_face, p_stub, 0.05, 
                                                               self.c_drawbar[0], self.c_drawbar[1], self.c_drawbar[2], self.c_drawbar[3]))

        self.pub_markers.publish(marker_array)
        
        # Publish Joint States (for Rviz visualization if URDF exists)
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = timestamp
        
        # Map state to joint names (example)
        # msg.name = ['joint_tractor_steering', 'joint_hitch_1', 'joint_dolly_1', ...]
        # msg.position = [self.delta, ...]
        
        # For now, just log the state or publish as custom message if needed
        # But user asked for "odometer", so maybe Odometry msg?
        # Since it's N trailers, standard Odometry isn't enough for all.
        # I'll stick to JointState for articulation angles.
        
        msg.name = ['steering_joint']
        msg.position = [float(self.delta)]
        
        for i in range(len(self.trailers)):
            # Drawbar Angle (relative to prev)
            idx_db = 3 + 2*i
            idx_prev = 2 + 2*i
            angle_db = self.state[idx_db] - self.state[idx_prev]
            
            # Trailer Angle (relative to drawbar)
            idx_tr = 3 + 2*i + 1
            angle_tr = self.state[idx_tr] - self.state[idx_db]
            
            msg.name.append(f'hitch_{i+1}_joint')
            msg.position.append(float(angle_db))
            
            msg.name.append(f'dolly_{i+1}_joint')
            msg.position.append(float(angle_tr))
            
        self.pub_joints.publish(msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = TractorOdometer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
