# This is the Python ROS2 library
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from rclpy.executors import ExternalShutdownException

class Joystick(Node):
  """This class is just here to update joystick data."""

  def __init__(self):
    super().__init__('joystick_pipeline') # Initializes this as a node

    # This node will publish joystick data (axes position and buttons)
    self.publisher = self.create_publisher(
      msg_type=Joy,
      topic='joystick_data'
    )

    # This node subscribes to a topic built into the Joy library
    # It publishes the current data from the joystick controller
    self.subscription = self.create_subscription(
      msg_type=Joy,
      topic='joy',
      callback=self.joystick_callback # This function gets triggered every time
                                      # a message is received from the topic
    )

  def joystick_callback(self, joystick):
    """Literally all this node does is publish the joystick data."""

    self.publisher.publish(joystick)

def main(args=None):
  """Initializes Joystick and starts the control loop."""

  try:
    with rclpy.init(args=args):
      joystick_pipeline = Joystick() # Create Joystick
      rclpy.spin(joystick_pipeline)  # Ensures that the code runs continuously until shutdown
  except (KeyboardInterrupt, ExternalShutdownException):
    # Shuts down if a KeyboardInterrupt or ExternalShutdownException is detected
    # i.e. if Ctrl+C is pressed or if ROS2 is shutdown externally

    # This command cleans up ROS2 resources to shutdown gracefully
    rclpy.shutdown()

if __name__ == '__main__':
  main()