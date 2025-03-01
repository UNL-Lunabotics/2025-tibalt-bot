# This is the Python ROS2 library
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# This is the Python ROS library that communicates with the joystick controller
from sensor_msgs.msg import Joy

# The Roboclaw library we use to control the motors requires Int16 types
from std_msgs.msg import Int16
from std_msgs.msg import Int16MultiArray


class HOPPER_STATE(enumerate):
  """Enum to keep track of the current hopper state."""
  REST = 0
  VIBRATING = 1
  OPEN_HATCH = 2
  CLOSE_HATCH = 3

class Tibalt(Node):
  """The node class for all the main Tibalt logic. This contains the functionality
  for drive train and autonomy."""

  def __init__(self):
    super().__init__('tibalt') # Initializes this as a node with the name 'tibalt'

    # This parameter will be used to keep track of the current state of Tibalt
      # For example, if the excavation button gets pressed, the state is excavation,
      # so we should spin the excavation wheel in our control logic
    self.declare_parameter(
      name='tibalt_state',
      value=TIBALT_STATE.REST
    )

    # This creates a Rate object that we will use to sleep the system at the specified frequency (in Hz)
    # We need this to make sure the node's don't publish data too quickly
    self.rate = self.create_rate(
      frequency=10,
      clock=self.get_clock()
    )

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
  
  def control_callback(self, joystick):
    """This contains the main driver control code."""

    # TODO: drivetrain logic (2 motors)
    
    # TODO: excavation logic (1 motor for turning scoops, 1 linear actuator to push into ground and retract)

    # TODO: hopper logic (1 linear actuator to lift bin, 1 servo to open hatch, 1 vibration motor on bottom)
    
    # TODO: publish all motor speeds here (need to decide on order based on total amount of motors for each system)
    motor_speeds = Int16MultiArray()
    # [dtLeft, dtRight, exTurn, exActuator, hopperActuator, hopperHatch, hopperVibe] (variable names can change but this is the order for the motors)
    motor_speeds.data = []
    self.publisher.publish(motor_speeds)


def main(args=None):
  """Initializes Tibalt and starts the control loop."""

  # I think this will loop by itself, but if it doesn't try while rclpy.ok()

  try:
    with rclpy.init(args=args):
      tibalt = Tibalt() # Create tibalt
      rclpy.spin(tibalt)  # Ensures that the code runs continuously until shutdown

      # Should sleep the node for 10Hz without blocking the entire system (may not work)
      tibalt.rate.sleep() # Experimental equivalent of rate.sleep(). It may or may not work
  except (KeyboardInterrupt, ExternalShutdownException):
    # Shuts down if a KeyboardInterrupt or ExternalShutdownException is detected
    # i.e. if Ctrl+C is pressed or if ROS2 is shutdown externally

    # This command cleans up ROS2 resources to shutdown gracefully
    # TODO: make sure we test this with disconnects, closing terminal, etc
    rclpy.shutdown()

if __name__ == '__main__':
  main()