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


class TIBALT_STATE(enumerate):
  """Enum to keep track of the current robot state. Does NOT include drivetrain states."""
  REST = 0
  DIGGING = 1

class EXCAV_STATE(enumerate):
  """Enum to keep track of the excavation states"""
  RESTING = 0
  EXTENDING = 1
  DIGGING = 2
  RETRACTING = 3

EXCAV_RETRACT_PERIOD = 10 # period of time (in seconds) for the linear atuator to retract to 0cm
                          # the current value is a meaningless placeholder
                          # TODO: find out how long this should take

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
    self.excavation_timer = 0
    self.excavation_spin_motor == MOTOR_STOP
    self.excavation_actuator_motor == MOTOR_STOP

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

    # EXCAVATION LOGIC

    # placeholder value for extending the linear actuator
    # this pressing this button will also begin the spinning motor
    # TODO: replace extend_button instances with the real value
    extend_button = 2
    # placeholder value for the retract button
    # TODO: replace retract_button with the real value
    retract_button = 3

    '''
    While not excavating, check if any butons are pressed that initialize digging,
    otherwise, do nothing
    '''
    if self.excavation_state == EXCAV_STATE.RESTING:
      self.excavation_spin_motor = MOTOR_STOP
      self.excavation_actuator_motor = MOTOR_STOP
      # if digging process has been initiated, continue to EXTENDING
      if joystick.buttons[extend_button] == 1:
        # in this moment, start extending, and digging
        self.excavation_state = EXCAV_STATE.EXTENDING

    '''
    while extending, keep spinning the excavator and check if the extend button
    is being held. If so, continue extending otherwise stop extending and change 
    states
    '''
    if self.excavation_state == EXCAV_STATE.EXTENDING:
      # keep spinning
      self.excavation_spin_motor = MOTOR_FORWARDS
      self.excavation_actuator_motor = MOTOR_FORWARDS
      # if the extending button is still being pressed
      if joystick.buttons[extend_button] != 1:
        self.excavation_state = EXCAV_STATE.DIGGING
    
    '''
    In the digging state, don't extend the actuator and continue spinning
    the excavator. Check if the extending button is pressed again, in which case
    return to the extending state. Check if the retract button is pressed, in
    which case move to the retracting state.
    '''
    if self.excavation_state == EXCAV_STATE.DIGGING:
      # no matter what, during this stage the motor should be spinning
      self.excavation_spin_motor = MOTOR_FORWARDS
      self.excavation_actuator_motor = MOTOR_STOP

      # if the extending button is pressed again
      if joystick.buttons[extend_button] == 1:
        self.excavation_state = EXCAV_STATE.EXTENDING

      # was the retract button pressed
      elif joystick.buttons[retract_button] == 1:
        self.excavation_timer = time.time()
        self.excavation_state = EXCAV_STATE.RETRACTING

    '''
    In the retracting state, the excavator is still spinning while the
    actuator is retracting. Stays in the retracting state for a set
    amount of time.
    '''
    if self.excavation_state == EXCAV_STATE.RETRACTING:

      if time.time() - self.excavation_timer < EXCAV_RETRACT_PERIOD:
        self.excavation_actuator_motor = MOTOR_BACKWARDS
        self.excavation_spin_motor = MOTOR_FORWARDS

      elif time.time() - self.excavation_timer > EXCAV_RETRACT_PERIOD:
        self.excavation_state = EXCAV_STATE.RESTING

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