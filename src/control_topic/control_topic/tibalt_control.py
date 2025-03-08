# This is the Python ROS2 library
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# This is the Python ROS library that communicates with the joystick controller
from sensor_msgs.msg import Joy

# The Roboclaw library we use to control the motors requires Int16 types
from std_msgs.msg import Int16
from std_msgs.msg import Int16MultiArray


class TIBALT_STATE(enumerate):
  """Enum to keep track of the current robot state. Does NOT include drivetrain states."""
  REST = 0
  DIGGING = 1

class Tibalt(Node):
  """The node class for all the main Tibalt logic. This contains the functionality
  for drive train and autonomy."""

  def __init__(self):
    super().__init__('tibalt') # Initializes this as a node with the name 'tibalt'

    # This node will publish the motor speeds we want the Arduino to set things to
    self.publisher = self.create_publisher(
      msg_type=Int16MultiArray,
      topic='motor_speeds'
    )

    # This node subscribes to 'joystick_data' (published from joystick_pipeline)
    # This Joy object gives the current axis and button input from the joystick
    self.subscription = self.create_subscription(
      msg_type=Joy,
      topic='joystick_data',
      callback=self.control_callback  # This function gets triggered every time
                                      # a message is received from the topic
    )
  
  def control_callback(self, joystick) -> None:
    """This contains the main driver control code."""

    # TODO: drivetrain logic (2 motors)
    x_axis: float = joystick.axes[0]  # pos<-right, neg<-left
    y_axis: float = joystick.axes[1]  # pos<-down, neg<-up
    forward: int = int(x_axis * 63)
    turn: int = int(y_axis * 63)
    DtLeft: int = 64 + forward + turn
    DtRight: int = 64 + forward - turn
    # TODO: hopper logic (1 linear actuator to lift bin, 1 servo to open hatch, 1 vibration motor on bottom)
    
    # TODO: publish all motor speeds here (need to decide on order based on total amount of motors for each system)
    motor_speeds = Int16MultiArray()
    # [dtLeft, dtRight, exTurn, exActuator, hopperActuator, hopperHatch, hopperVibe] (variable names can change but this is the order for the motors)
    motor_speeds.data = []
    self.publisher.publish(motor_speeds)


def main(args=None):
  """Initializes Tibalt and starts the control loop."""

  try:
    with rclpy.init(args=args):
      tibalt = Tibalt()
      
      while rclpy.ok(): # Ensures that the code runs continuously until shutdown

        # This will allow the node to process pending callback requests once before
        # continuing to run this loop. This allows us to control the callback rate
        rclpy.spin_once(tibalt)  

        # Sleep the node for 10Hz without blocking the entire system
        tibalt.loop_rate.sleep()
  except (KeyboardInterrupt, ExternalShutdownException):
    # Shuts down if a KeyboardInterrupt or ExternalShutdownException is detected
    # i.e. if Ctrl+C is pressed or if ROS2 is shutdown externally

    # This command cleans up ROS2 resources to shutdown gracefully
    # TODO: make sure we test this with disconnects, closing terminal, etc
    rclpy.shutdown()

if __name__ == '__main__':
  main()