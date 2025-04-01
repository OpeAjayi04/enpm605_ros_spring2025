from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import rclpy
from std_msgs.msg import String

class LocalizationDemo(LifecycleNode):
    def __init__(self):
        super().__init__('localization_demo')
        self._sensor_ready = False
        self._sensor_data_subscriber = None
        self._localization_status_publisher = None

    def on_configure(self, state: State):
        self.get_logger().info('Configuring localization_demo...')
        self._sensor_data_subscriber = self.create_subscription(String, 'sensor_data', self._sensor_data_sub_callback, 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        self.get_logger().info('Activating localization_demo...')
        
        # Latched publisher (TRANSIENT_LOCAL)
        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self._localization_status_publisher = self.create_publisher(String, 'localization_status', qos)

        # Publish latched "ready" message
        msg = String()
        msg.data = 'localization ready'
        self._localization_status_publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data} (latched)')
        
        return TransitionCallbackReturn.SUCCESS

    def _sensor_data_sub_callback(self, msg):
        if msg.data == 'sensor ready' and not self._sensor_ready:
            self.get_logger().info('Sensor is ready → Activating localization_demo')
            self._sensor_ready = True
            self.trigger_activate()


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationDemo()
    try:
        node.trigger_configure()
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Initialization error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()