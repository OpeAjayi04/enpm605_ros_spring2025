from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

class NavigationDemo(LifecycleNode):
    def __init__(self):
        super().__init__('navigation_demo')
        # self.loc_ready = False
        self._localization_status_sub = None
        self._activated = False
        self._drive_timer = None
        # self._sensor_data_sub = None

    def on_configure(self, state: State):
        self.get_logger().info('Configuring navigation_demo...')
        # Subscribe to latched localization status
        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self._localization_status_sub = self.create_subscription(
            String, 'localization_status', self._localization_callback, qos
        )
        return TransitionCallbackReturn.SUCCESS
    
    def _localization_callback(self, msg):
        if msg.data == 'localization ready' and not self._activated:
            self.get_logger().info('Localization ready → Activating navigation_demo')
            self._activated = True
            self.trigger_activate()
            
    def on_activate(self, state: State):
        self.get_logger().info('Activating navigation_demo...')
        self._drive_timer = self.create_timer(1.5, self._drive_callback)
        return TransitionCallbackReturn.SUCCESS

    def _drive_callback(self):
        self.get_logger().info('Driving to goal...')

    
def main(args=None):
    rclpy.init(args=args)
    node = NavigationDemo()
    try:
        node.trigger_configure()
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Initialization error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
