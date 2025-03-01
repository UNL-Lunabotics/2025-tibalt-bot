# This is the Python ROS2 library
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from rclpy.executors import ExternalShutdownException


class Joystick(Node):
  """This class is just here to update joystick data."""

  def __init__(self):
    super().__init__('joystick_pipeline') # Initializes this as a node

    # This creates a Rate object that we will use to sleep the system at the specified frequency (in Hz)
    # We need this to make sure the node's don't publish data too quickly and take up processing power
    self.rate = self.create_rate(
      frequency=20,
      clock=self.get_clock()
    )

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
  """Initializes Tibalt and starts the control loop."""

  try:
    with rclpy.init(args=args):
      joystick_pipeline = Joystick()
    while rclpy.ok(): # Ensures that the code runs continuously until shutdown

      # This will allow the node to process pending callback requests once before
      # continuing to run this loop. This allows us to control the callback rate
      rclpy.spin_once(joystick_pipeline)  

      # Sleep the node for 20Hz without blocking the entire system
      joystick_pipeline.loop_rate.sleep()
  except (KeyboardInterrupt, ExternalShutdownException):
    # Shuts down if a KeyboardInterrupt or ExternalShutdownException is detected
    # i.e. if Ctrl+C is pressed or if ROS2 is shutdown externally

    # This command cleans up ROS2 resources to shutdown gracefully
    # TODO: make sure we test this with disconnects, closing terminal, etc
    rclpy.shutdown()

if __name__ == '__main__':
  main()