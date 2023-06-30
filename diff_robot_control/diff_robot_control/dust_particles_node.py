import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

def dust_positions(resolution=10):
    x_lim = [-9.1, 9.1]
    y_lim = [-5.3, 5.3]
    X = np.linspace(x_lim[0], x_lim[1], resolution)
    Y = np.linspace(y_lim[0], y_lim[1], resolution)
    dust_coords = []
    for i in range(len(X)):
        for j in range(len(Y)):
            dust_coords.append([X[i], Y[j]])
    return dust_coords

class DustPublisher(Node):
    def __init__(self):
        super().__init__('dust_particle_message_broadcaster')
        self.avg_radius = 0.3 # tunable dust particle's radius
        self.robot_r = 0.2 # tunable robot's gathering area 
        self.path_msg = Path()

        self.dust_coords = dust_positions(30) # generate dust particles
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.handleOdom, 10)
        self.dust_pub = self.create_publisher(MarkerArray, 'dust_particles', 10)
        self.path_pub = self.create_publisher(Path, 'path', 10)
        print("Starting dust particles and path publisher")

    def handleOdom(self, msg):
        robot_x = msg.pose.pose.position.x
        robot_y = msg.pose.pose.position.y
        for dust in self.dust_coords:
            x, y = dust
            if robot_x - self.robot_r < x and robot_x + self.robot_r > x and \
               robot_y - self.robot_r < y and robot_y + self.robot_r > y:
               self.dust_coords.remove(dust)

        self.publishDustParticles(msg)
        self.publishPath(msg)

    def publishCleanDustParticles(self, msg):
        dust_particles_msg = MarkerArray()
        marker = Marker()
        marker.header.stamp = msg.header.stamp
        marker.header.frame_id = msg.header.frame_id
        marker.ns = "dust_particle"

        marker.type = Marker.CYLINDER
        marker.action = Marker.DELETEALL

        dust_particles_msg.markers.append(marker)
        self.dust_pub.publish(dust_particles_msg)

    def publishDustParticles(self, msg):
        # self.publishCleanDustParticles(msg)
        dust_particles_msg = MarkerArray()
        for i in range(len(self.dust_coords)):
            marker = Marker()
            marker.header.stamp = msg.header.stamp
            marker.header.frame_id = msg.header.frame_id
            marker.ns = "dust_particle"
            marker.id = i

            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.scale.x = self.avg_radius
            marker.scale.y = self.avg_radius
            marker.scale.z = 0.01

            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            marker.pose.position.x = self.dust_coords[i][0]
            marker.pose.position.y = self.dust_coords[i][1]
            marker.pose.position.z = 0.0

            dust_particles_msg.markers.append(marker)

        self.dust_pub.publish(dust_particles_msg)

    def publishPath(self, msg):
        self.path_msg.header.stamp = msg.header.stamp
        self.path_msg.header.frame_id = msg.header.frame_id

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = msg.header.frame_id
        pose.pose = msg.pose.pose
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

def main():
    rclpy.init()
    node = DustPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
