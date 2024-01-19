###################################################################################
# MIT License
#
# Copyright (c) 2024 Hannibal Paul
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
###################################################################################

__author__ = "Hannibal Paul"

import rclpy
from rclpy.node import Node
from mocap_interfaces.msg import RigidBodies, RigidBody
from px4_msgs.msg import VehicleOdometry

class MocapToPx4(Node):

    def __init__(self):
        super().__init__('mocap_2_px4')
        
        # Set the ID of the body to subscribe from MOCAP and publish to PX4
        self.body_id = 6

        # Publishers and Subscribers
        self.odom_pub = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', 10)
        self.mocap_sub = self.create_subscription(RigidBodies, '/mocap/rigid_bodies', self.mocap_callback, 10)

    def mocap_callback(self, msg):
        # Get the required body based on set ID
        get_body = RigidBody()
        bodies = msg.rigid_bodies
        for body in bodies:
            if body.id == self.body_id:
                get_body = body

        # Msg to publish
        vehicle_odom = VehicleOdometry()

        # Timestamp is automatically set inside PX4
        vehicle_odom.timestamp = 0
        vehicle_odom.timestamp_sample = 0

        vehicle_odom.pose_frame = 2 # FRD world-fixed frame, arbitrary heading reference

        # Position [x, y, z] in meters
        px = get_body.pose_stamped.pose.position.x
        py = get_body.pose_stamped.pose.position.y
        pz = get_body.pose_stamped.pose.position.z
        vehicle_odom.position = [-pz, px, -py]

        # Orinetation [w, x, y, z]
        ox = get_body.pose_stamped.pose.orientation.x
        oy = get_body.pose_stamped.pose.orientation.y
        oz = get_body.pose_stamped.pose.orientation.z
        ow = get_body.pose_stamped.pose.orientation.w
        vehicle_odom.q = [ow, -oz, ox, -oy]

        # Other parameters
        vehicle_odom.velocity_frame = 2
        vehicle_odom.velocity = [0.0, 0.0, 0.0] # in m/s
        vehicle_odom.angular_velocity = [0.0, 0.0, 0.0] # in body-fixed frame (rad/s)
        vehicle_odom.position_variance = [0.0, 0.0, 0.0]
        vehicle_odom.orientation_variance = [0.0, 0.0, 0.0]
        vehicle_odom.velocity_variance = [0.0, 0.0, 0.0]

        # Publish
        self.odom_pub.publish(vehicle_odom)


def main(args=None):
    rclpy.init(args=args)

    mocap2px4 = MocapToPx4()

    rclpy.spin(mocap2px4)

    mocap2px4.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()