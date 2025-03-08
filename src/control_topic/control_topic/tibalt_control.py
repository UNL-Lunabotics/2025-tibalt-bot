# This is the Python ROS2 library
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# This is the Python ROS library that communicates with the joystick controller
from sensor_msgs.msg import Joy

# The Roboclaw library we use to control the motors requires Int16 types
from std_msgs.msg import Int16
from std_msgs.msg import Int16MultiArray


# Motor speed constants to help make the code more readable
# They range from 0-127, with 0 as full power backward, 64 as stop, and 127 as full power forward
MOTOR_FORWARDS = 127
MOTOR_BACKWARDS = 0
MOTOR_STOP = 64


class TIBALT_STATE(enumerate):
  """Enum to keep track of the current robot state. Does NOT include drivetrain states."""
  REST = 0
  DIGGING = 1

class EXCAV_STATE(enumerate):
  """Enum to keep track of the excavation states"""
  RESTING = 0
  EXTENDING = 1
  DIGGING = 2
  RETRACTING = 4

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

    # Excavation motor variables
    self.excavation_state = EXCAV_STATE.RESTING
    self.excavation_spin_motor == MOTOR_STOP
    self.excavation_actuator_motor == MOTOR_STOP

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

    # placeholder value for extending the linear actuator
    # this pressing this button will also begin the spinning motor
    # TODO: replace extend_button instances with the real value
    extend_button = 2
    # placeholder value for the retract button
    # TODO: replace retract_button with the real value
    retract_button = 3
    # if not currently digging
    if self.excavation_state == EXCAV_STATE.RESTING:
      # if digging process has been initiated, continue to EXTENDING
      if joystick.buttons[extend_button] == 1:

        self.excavation_spin_motor = MOTOR_FORWARDS
        self.excavation_actuator_motor = MOTOR_FORWARDS

        self.excavation_state = EXCAV_STATE.EXTENDING
      else: # do nothing
        self.excavation_spin_motor = MOTOR_STOP
        self.excavation_actuator_motor = MOTOR_STOP

    # if the actuator is extending
    if self.excavation_state == EXCAV_STATE.EXTENDING:
      # keep spinning
      self.excavation_spin_motor = MOTOR_FORWARDS
      # if the extending button is still being pressed
      if joystick.buttons[extend_button] == 1:
        # keep extending the actuator
        self.excavation_actuator_motor = MOTOR_FORWARDS
      else:
        self.excavation_actuator_motor = MOTOR_STOP
        self.excavation_state = EXCAV_STATE.DIGGING
    
    # if the tibalt is digging (actuator is still)
    if self.excavation_state == EXCAV_STATE.DIGGING:
      self.excavation_spin_motor == MOTOR_FORWARDS
      # if the extending button is pressed again
      if joystick.buttons[extend_button] == 1:
        self.excavation_actuator_motor == MOTOR_FORWARDS
      elif joystick.buttons[retract_button] == 1:
        self.excavation_actuator_motor == MOTOR_BACKWARDS
        self.excavation_state = EXCAV_STATE.RETRACTING
      else:
        self.excavation_actuator_motor == MOTOR_STOP


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