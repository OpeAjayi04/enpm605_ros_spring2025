from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
import rclpy
from std_msgs.msg import String

class SensorNodeDemo(LifecycleNode):
    def __init__(self):
        super().__init__('sensor_demo')
        self._sensor_data_publisher = None
        self._sensor_data_timer = None

    def on_configure(self, state: State):
        self.get_logger().info('Configuring sensor_demo...')
        self._sensor_data_publisher = self.create_publisher(String, 'sensor_data', 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        self.get_logger().info('Activating sensor_demo...')
        self._sensor_data_timer = self.create_timer(0.5, self._sensor_data_pub_callback)
        return TransitionCallbackReturn.SUCCESS

    def _sensor_data_pub_callback(self):
        msg = String()
        msg.data = 'sensor ready'
        self._sensor_data_publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorNodeDemo()
    try:
        node.trigger_configure()
        node.trigger_activate()
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Initialization error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()