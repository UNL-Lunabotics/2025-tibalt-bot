# This is the Python ROS2 library
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# This is the Python ROS library that communicates with the joystick controller
from sensor_msgs.msg import Joy

# The Roboclaw library we use to control the motors requires Int16 types
from std_msgs.msg import Int16
from std_msgs.msg import Int16MultiArray

import time

# Motor speed constants to help make the code more readable
# They range from 0-127, with 0 as full power backward, 64 as stop, and 127 as full power forward
MOTOR_FORWARDS = 127
MOTOR_BACKWARDS = 0
MOTOR_STOP = 64

# Hopper constants that should not change during the program
HOPPER_EXTEND_AND_RETRACT_PERIOD = 10 # This is how long it should take to extend/retract the hopper linear actuators
                                      # In seconds

HOPPER_PAUSE_PERIOD = 5 # This is how long the hopper should pause when fully extended before retracting
                        # In seconds

class HOPPER_STATE(enumerate):
  """Enum to keep track of the current hopper state."""
  RESTING = 0
  EXTENDING = 1
  DUMPING = 2
  RETRACTING = 3
  
class Tibalt(Node):
  """The node class for all the main Tibalt logic. This contains the functionality
  for drive train and autonomy."""

  def __init__(self):
    super().__init__('tibalt') # Initializes this as a node with the name 'tibalt'

    # This is where we will put the hopper class variables that will change during the program
    self.hopper_state = HOPPER_STATE.REST
    self.hopper_timer = 0
    self.hopper_vibration_motor = MOTOR_STOP
    self.hopper_actuator_motor = MOTOR_STOP

    # There should be other class variables for excavation and drivetrain

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
    
    # If the hopper is not active
    if self.hopper_state == HOPPER_STATE.RESTING:
      # If the dumping process has bee initiated, continue to EXTENDING
      if joystick.buttons[0] == 0: # TODO: Placeholder button
        # TODO: Unlock latch
        self.hopper_vibration_motor = MOTOR_FORWARDS
        self.hopper_actuator_motor = MOTOR_FORWARDS

        self.hopper_timer = time.time()

        self.hopper_state = HOPPER_STATE.EXTENDING
      else: # Do nothing
        self.hopper_actuator_motor = MOTOR_STOP
        self.hopper_vibration_motor = MOTOR_STOP

    # If the linear actuators are currently extending
    elif self.hopper_state == HOPPER_STATE.EXTENDING:
      # If it hasn't been extending long enough, continue extending
      if time.time() - self.hopper_timer < HOPPER_EXTEND_AND_RETRACT_PERIOD:
        self.hopper_vibration_motor = MOTOR_FORWARDS
        self.hopper_actuator_motor = MOTOR_FORWARDS
      
      # If it has been extending long enough, continue to DUMPING phase
      elif time.time() - self.hopper_timer > HOPPER_EXTEND_AND_RETRACT_PERIOD:
        self.hopper_vibration_motor = MOTOR_FORWARDS
        self.hopper_actuator_motor = MOTOR_STOP

        self.hopper_timer = time.time()

        self.hopper_state = HOPPER_STATE.DUMPING

    # If the hopper has extended but is pausing at full extension to dump all the material
    elif self.hopper_state == HOPPER_STATE.DUMPING:
      # If it hasn't been paused long enough, continue vibrating
      if time.time() - self.hopper_timer < HOPPER_PAUSE_PERIOD:
        self.hopper_actuator_motor = MOTOR_STOP
        self.hopper_vibration_motor = MOTOR_FORWARDS
      
      # If it has been paused long enough, continue to RETRACTING
      elif time.time() - self.hopper_timer > HOPPER_PAUSE_PERIOD:
        self.hopper_actuator_motor = MOTOR_BACKWARDS
        self.hopper_vibration_motor = MOTOR_FORWARDS # Might as well keep this on until retraction is finished

        self.hopper_timer = time.time()

        self.hopper_state = HOPPER_STATE.RETRACTING

    # If the linear actuators are retracting
    elif self.hopper_state == HOPPER_STATE.RETRACTING:
      # If they have not been retracting long enough, continue retracting
      if time.time() - self.hopper_timer < HOPPER_EXTEND_AND_RETRACT_PERIOD:
        self.hopper_actuator_motor = MOTOR_BACKWARDS
        self.hopper_vibration_motor = MOTOR_FORWARDS
      
      # If it has been retracting long enough to be fully retracted, continue to RESTING
      elif time.time() - self.hopper_timer > HOPPER_EXTEND_AND_RETRACT_PERIOD:
        self.hopper_actuator_motor = MOTOR_STOP
        self.hopper_vibration_motor = MOTOR_STOP
        # TODO: Lock latch
        
        self.hopper_state = HOPPER_STATE.RESTING
        
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