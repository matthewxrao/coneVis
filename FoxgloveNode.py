import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from interfaces.msg import ConeArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import argparse
import sys

NODE_NAME = 'foxglove_node'

BEST_EFFORT_QOS_PROFILE = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=1
)

class FoxgloveNode(Node):
    def __init__(self, print_counts):
        super().__init__(NODE_NAME)
        
        self.print_counts = print_counts
        self.last_marker_count = 0  # Track the number of markers published previously

        self.publisher_ = self.create_publisher(
            MarkerArray,
            'cone_markers',
            10
        )
        
        self.subscriber = self.create_subscription(
            ConeArray,
            '/perc_cones',
            self.cone_array_callback,
            qos_profile=BEST_EFFORT_QOS_PROFILE
        )

    def cone_array_callback(self, msg: ConeArray):
        # Print cone counts if the flag is enabled
        if self.print_counts:
            print(
                f"{len(msg.blue_cones):<3} Blue Cones | "
                f"{len(msg.yellow_cones):<3} Yellow Cones | "
                f"{len(msg.orange_cones):<3} Orange Cones | "
                f"{len(msg.big_orange_cones):<3} Big Orange Cones | "
                f"{len(msg.unknown_color_cones):<3} Unknown Color Cones"
            )
        
        marker_array = self.create_marker_array(msg)
        self.publisher_.publish(marker_array)

    def create_marker_array(self, msg: ConeArray):
        marker_array = MarkerArray()
        marker_id = 0
        namespace = "cone_markers"

        def add_cones(cones, color):
            nonlocal marker_id
            for cone in cones:
                marker = Marker()
                marker.header.frame_id = 'hesai_lidar'
                marker.header.stamp = msg.header.stamp
                marker.ns = namespace
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                # Transforming coordinates as per the original code
                marker.pose.position.x = cone.y
                marker.pose.position.y = -cone.x
                marker.pose.position.z = -0.7
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.r = color[0] / 255.0
                marker.color.g = color[1] / 255.0
                marker.color.b = color[2] / 255.0
                marker.color.a = 1.0
                marker_array.markers.append(marker)
                marker_id += 1

        # Add cones from each category
        add_cones(msg.blue_cones, [0, 0, 255])
        add_cones(msg.yellow_cones, [255, 255, 0])
        add_cones(msg.orange_cones, [255, 165, 0])
        add_cones(msg.big_orange_cones, [255, 69, 0])
        add_cones(msg.unknown_color_cones, [128, 128, 128])

        # Delete leftover markers if any
        for marker_id_to_delete in range(marker_id, self.last_marker_count):
            delete_marker = Marker()
            delete_marker.header.frame_id = 'hesai_lidar'
            delete_marker.header.stamp = msg.header.stamp
            delete_marker.ns = namespace
            delete_marker.id = marker_id_to_delete
            delete_marker.action = Marker.DELETE
            marker_array.markers.append(delete_marker)

        # Update the last marker count
        self.last_marker_count = marker_id

        return marker_array


def main(args=None):
    parser = argparse.ArgumentParser(description="Foxglove Cone Visualization Node")
    parser.add_argument('-p', '--print', action='store_true', help="Print cone counts to the console")
    parsed_args = parser.parse_args(args=sys.argv[1:])

    rclpy.init(args=args)
    node = FoxgloveNode(print_counts=parsed_args.print)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
