# This is the Python ROS2 library
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# This is the Python ROS library that communicates with the joystick controller
from sensor_msgs.msg import Joy

# The Roboclaw library we use to control the motors requires Int16 types
from std_msgs.msg import Int16
from std_msgs.msg import Int16MultiArray

class TERRENCE_STATE(enumerate):
  """Enum to keep track of the current robot state. Does NOT include drivetrain states."""
  REST = 0
  DIGGING = 1

class Terrence(Node):
  """The node class for all the main Terrence logic. This contains the functionality
  for drive train and autonomy."""

  def __init__(self):
    super().__init__('terrence') # Initializes this as a node with the name 'terrence'

    # This parameter will be used to keep track of the current state of Terrence
      # For example, if the excavation button gets pressed, the state is excavation,
      # so we should spin the excavation wheel in our control logic
    self.declare_parameter(
      name='terrence_state',
      value=TERRENCE_STATE.REST
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

    # Left right and front back axes. Gives an output in the -1 to 1 range
    lr_axis = joystick.axes[0]
    fb_axis = joystick.axes[1]

    # We will need these values to calculate drivetrain motor speeds
    max_axis = max(abs(fb_axis), abs(lr_axis))
    total = fb_axis + lr_axis
    diff = fb_axis - lr_axis

    # Calculate if the axes are moving forward or moving backwards
    lr_sign = 1 if lr_axis > 0 else -1
    fb_sign = 1 if fb_axis > 0 else -1

    # Calculate DriveTrainLeft (DtLeft) and DriveTrainRight (DtRight) based on signs
      # The reason we only calculate 2 values even though we have 4 motors is because
      # the motors on the left and right sides will bet set to the same speed
    # The only thing that changes based on direction is what 63 is multiplied by
    DtLeft = 64 - 63 * (
                        diff        if fb_sign == 1 and lr_sign == 1    else
                        max_axis    if fb_sign == 1 and lr_sign == -1   else
                        -max_axis   if fb_sign == -1 and lr_sign == 1   else
                        -diff
                        )
    DtRight = 64 + 63 * (
                        max_axis    if fb_sign == 1 and lr_sign == 1    else
                        total       if fb_sign == 1 and lr_sign == -1   else
                        total       if fb_sign == -1 and lr_sign == 1   else
                        -max_axis
                        )
    
    # TODO excavation logic
    
    motor_speeds = Int16MultiArray()
    motor_speeds.data = [(Int16)(DtLeft), (Int16)(DtRight)]
    self.publisher.publish(motor_speeds)

def main(args=None):
  """Initializes Terrence and starts the control loop."""

  try:
    with rclpy.init(args=args):
      terrence = Terrence() # Create terrence
      rclpy.spin(terrence)  # Ensures that the code runs continuously until shutdown
  except (KeyboardInterrupt, ExternalShutdownException):
    # Shuts down if a KeyboardInterrupt or ExternalShutdownException is detected
    # i.e. if Ctrl+C is pressed or if ROS2 is shutdown externally

    # This command cleans up ROS2 resources to shutdown gracefully
    rclpy.shutdown()

if __name__ == '__main__':
  main()