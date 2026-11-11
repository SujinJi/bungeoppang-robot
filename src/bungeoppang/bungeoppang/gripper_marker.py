import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class GripperMarker(Node):
    def __init__(self):
        super().__init__('gripper_marker')
        self.pub = self.create_publisher(MarkerArray, 'gripper_markers', 10)
        self.declare_parameter('opening', 0.04)
        self.create_timer(0.1, self.publish)
        self.get_logger().info('그리퍼 마커 발행 시작')

    def box(self, mid, pos, size, color):
        m = Marker()
        m.header.frame_id = 'link_6'
        m.ns = 'gripper'
        m.id = mid
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = pos[0]
        m.pose.position.y = pos[1]
        m.pose.position.z = pos[2]
        m.pose.orientation.w = 1.0
        m.scale.x = size[0]
        m.scale.y = size[1]
        m.scale.z = size[2]
        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        m.color.a = 1.0
        return m

    def publish(self):
        w = self.get_parameter('opening').value
        arr = MarkerArray()
        arr.markers.append(self.box(0, (0.0, 0.0, 0.03), (0.06, 0.12, 0.06), (0.3, 0.3, 0.35)))
        arr.markers.append(self.box(1, (0.0, w / 2 + 0.006, 0.10), (0.02, 0.012, 0.08), (0.9, 0.6, 0.2)))
        arr.markers.append(self.box(2, (0.0, -w / 2 - 0.006, 0.10), (0.02, 0.012, 0.08), (0.9, 0.6, 0.2)))
        self.pub.publish(arr)


def main():
    rclpy.init()
    rclpy.spin(GripperMarker())


if __name__ == '__main__':
    main()