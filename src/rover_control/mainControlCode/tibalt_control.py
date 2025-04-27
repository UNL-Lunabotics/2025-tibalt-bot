# This is the Python ROS2 library
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# This is the Python ROS library that communicates with the joystick controller
from sensor_msgs.msg import Joy

# The Roboclaw library we use to control the motors
from std_msgs.msg import Int16 # Roboclaw requires Int16 types
from std_msgs.msg import Int16MultiArray

'''CONSTANT DECLARATION'''
# Motor speed constants to help make the code more readable
# They range from 0-127, with 0 as full power backward, 64 as stop, and 127 as full power forward
MOTOR_FORWARDS = 127
MOTOR_BACKWARDS = 0
MOTOR_STOP = 64

# Servo constants to help make the code more readable
# It ranges from 0-180 degrees
SERVO_FULL_CLOSE = 0
SERVO_FULL_OPEN = 180

EXCAV_MOTOR_INCREMENT = 10
EXCAV_RETRACT_PERIOD = 10  # period of time (in seconds) for the linear atuator to retract to 0cm

'''CONTROL SCHEME'''
# EXCAV_ROTATE_B = 0 # There also isn't support for this in the code as it is automatically decided
                   # by state machine logic
EXCAV_RETRACT_B = 0
EXCAV_EXTEND_B = 0
HOPPER_RETRACT_B = 0
HOPPER_EXTEND_B = 0
# Note we don't actually need a hopper latch button, it's coded automatically

class HOPPER_STATE(enumerate):
  """Enum to keep track of the current hopper state."""
  RESTING = 0
  EXTENDING = 1
  DUMPING = 2
  RETRACTING = 3

class EXCAV_STATE(enumerate):
  """Enum to keep track of the excavation states."""
  RESTING = 0
  EXTENDING = 1
  DIGGING = 2
  RETRACTING = 3

class Tibalt(Node):
  """The node class for all the main Tibalt logic. This contains the functionality
  for drive train and autonomy."""

  def __init__(self):
    super().__init__('tibalt') # Initializes this as a node with the name 'tibalt'

    # Hopper class variables
    self.hopper_state = HOPPER_STATE.RESTING
    self.hopper_vibration_motor = MOTOR_STOP
    self.hopper_actuator_motor = MOTOR_STOP
    self.hopper_latch_servo = SERVO_FULL_CLOSE

    # Drivetrain class variables
    self.dt_left_motors = MOTOR_STOP
    self.dt_right_motors = MOTOR_STOP

    # Excavation motor variables
    self.excav_state = EXCAV_STATE.RESTING
    self.excav_spin_motor = MOTOR_STOP
    self.excav_actuator_motor = MOTOR_STOP

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
  
  def control_callback(self, joystick: Joy):
    """This contains the main driver control code."""

    # Button check for excavation
    if joystick.buttons[EXCAV_EXTEND_B] == 1:    self.excav_state = EXCAV_STATE.EXTENDING
    elif joystick.buttons[EXCAV_RETRACT_B] == 1: self.excav_state = EXCAV_STATE.RETRACTING
    else:                                        self.excav_state = EXCAV_STATE.RESTING

    # Button check for hopper
    if joystick.buttons[HOPPER_EXTEND_B] == 1:    self.hopper_state = HOPPER_STATE.EXTENDING
    elif joystick.buttons[HOPPER_RETRACT_B] == 1: self.hopper_state = HOPPER_STATE.RETRACTING
    else:                                         self.hopper_state = HOPPER_STATE.RESTING

    drivetrain_logic(self=self, joystick=joystick)
    excavation_logic(self=self)
    hopper_logic(self=self)

    # TODO: publish all motor speeds here (need to decide on order based on total amount of motors for each system)
    motor_speeds = Int16MultiArray()
    # [dtLeft, dtRight, exTurn, exActuator, hopperActuator, hopperHatch, hopperVibe] (variable names can change but this is the order for the motors)
    motor_speeds.data = []
    self.publisher.publish(motor_speeds)

def drivetrain_logic(self, joystick: Joy):
  """The logic for computing the motor speeds for the drivetrain.
  
  Parameters:
    self
    joystick, the Joy object representing joystick input from the driver
  Returns: Nothing, it sets relevant class variables instead
  """

  # Don't understand the math? Me neither, accept that it works
  x_axis: float = joystick.axes[0]  # pos<-right, neg<-left
  y_axis: float = joystick.axes[1]  # pos<-down, neg<-up

  forward: int = int(x_axis * 63)
  turn: int = int(y_axis * 63)

  DtLeft: int = 64 + forward + turn
  DtRight: int = 64 + forward - turn

  self.dt_left_motors = DtLeft
  self.dt_right_motors = DtRight

def excavation_logic(self):
  """The logic for computing the motor speeds for the excavation system.
  It contains the state machine that will do the necessary calculations.
  
  Parameters:
    self
    joystick, the Joy object representing joystick input from the driver
  Returns: Nothing, it sets relevant class variables instead
  """
  # These are helper functions to help mediate motor speeds
  def incrementMotor(motorSpeed, max):
    if (motorSpeed + EXCAV_MOTOR_INCREMENT > max):
      return max
    else:
      return motorSpeed + EXCAV_MOTOR_INCREMENT
    
  def decrementMotor(motorSpeed, min):
    if (motorSpeed - EXCAV_MOTOR_INCREMENT < min):
      return min
    else:
      return motorSpeed - EXCAV_MOTOR_INCREMENT
    
  # TODO what to do about digging? should we move while digging or no?

  # In the resting state, both motors are not moving. Check if the extend
  # button is pressed
  if self.excavation_state == EXCAV_STATE.RESTING:
    self.excavation_spin_motor = MOTOR_STOP
    self.excavation_actuator_motor = MOTOR_STOP

  # In the extending state, the actuator is extending while the excavator is spinning.
  # The motors are in this state as long as the extending button remains pressed.
  # check if the button is released
  elif self.excavation_state == EXCAV_STATE.EXTENDING:

    # Increment values while under 127
    self.excavation_spin_motor = incrementMotor(self.excavation_spin_motor, MOTOR_FORWARDS)
    self.excavation_actuator_motor = incrementMotor(self.excavation_actuator_motor, MOTOR_FORWARDS)
  
    # # if the extending button is released move to digging
    # if joystick.buttons[EXCAV_EXTEND_B] == 0:
    #   self.excavation_state = EXCAV_STATE.DIGGING
  
  # # In the digging state, the actuator is still while the excavator is still
  # # spinning. Check if the extend button or the retract button is pressed.
  # # Move to the extend or retract state accordingly
  # if self.excavation_state == EXCAV_STATE.DIGGING:

  #   #Increment values while under 127(MOTOR_FORWARDS)
  #   self.excavation_spin_motor = incrementMotor(self.excavation_spin_motor, MOTOR_FORWARDS)
  #   self.excavation_actuator_motor = decrementMotor(self.excavation_actuator_motor, MOTOR_STOP)

  #   # can return to the extending state
  #   if joystick.buttons[EXCAV_EXTEND_B] == 1:
  #     self.excavation_state = EXCAV_STATE.EXTENDING

  #   elif joystick.buttons[EXCAV_RETRACT_B] == 1:
  #     # records the time the retract button was pressed
  #     self.excavation_timer = time.time()
  #     self.excavation_state = EXCAV_STATE.RETRACTING

  # In the retracting state, the excavator is still spinning while the
  # actuator is retracting. Stays in the retracting state for a set
  # amount of time.
  elif self.excavation_state == EXCAV_STATE.RETRACTING:
    # retract state motor status
    self.excavation_spin_motor = decrementMotor(self.excavation_spin_motor, MOTOR_FORWARDS)

    # incase the actuator is still moving forward
    if (self.excavation_actuator_motor > MOTOR_STOP):
      #sets it to the increment value so that it will be at MOTOR_STOP once decrementMotor() is called
      self.excavation_actuator_motor = EXCAV_MOTOR_INCREMENT

    self.excavation_actuator_motor = decrementMotor(self.excavation_actuator_motor, MOTOR_BACKWARDS)

def hopper_logic(self):
  """The logic for computing the motor speeds for the hopper system.
  It contains the state machine that will do the necessary calculations.
  
  Parameters:
    self
    joystick, the Joy object representing joystick input from the driver
  Returns: Nothing, it sets relevant class variables instead
  """

  # TODO How to do vibration motor and latch servo with this model?

  # If the extend button is currently being pressed
  if self.hopper_state == HOPPER_STATE.EXTENDING:
    self.hopper_vibration_motor = MOTOR_STOP
    self.hopper_actuator_motor = MOTOR_FORWARDS
    self.hopper_latch_servo = SERVO_FULL_OPEN

  # If the retracting button is curreltny being pressed
  elif self.hopper_state == HOPPER_STATE.RETRACTING:
    self.hopper_vibration_motor = MOTOR_STOP
    self.hopper_actuator_motor = MOTOR_BACKWARDS
    self.hopper_latch_servo = SERVO_FULL_OPEN
  
  # If no hopper buttons are being pressed, do nothing
  elif self.hopper_state == HOPPER_STATE.RESTING:
    self.hopper_actuator_motor = MOTOR_STOP
    self.hopper_vibration_motor = MOTOR_STOP

  # # If the hopper is not active, all motors should be stopped and we should be monitoring
  # # for input to change the hopper state
  # if self.hopper_state == HOPPER_STATE.RESTING:
    
  #   # The driver has initiated hopper dumping, begin extending the hopper
  #   # and then continue to EXTENDING
  #   if joystick.buttons[HOPPER_EXTEND_B] == 1:
  #     self.hopper_vibration_motor = MOTOR_STOP
  #     self.hopper_actuator_motor = MOTOR_FORWARDS
  #     self.hopper_latch_servo = SERVO_FULL_OPEN

  #     self.hopper_state = HOPPER_STATE.EXTENDING
  #   else: # Do nothing
  #     self.hopper_actuator_motor = MOTOR_STOP
  #     self.hopper_vibration_motor = MOTOR_STOP

  # # If the hopper actuators are currently extending, we need to continue extending
  # # until the full extension time period is over, continue to DUMPING
  # elif self.hopper_state == HOPPER_STATE.EXTENDING:
  #   # If it hasn't been extending long enough, continue extending
  #   if time.time() - self.hopper_timer < HOPPER_EXTEND_AND_RETRACT_PERIOD:
  #     self.hopper_vibration_motor = MOTOR_STOP
  #     self.hopper_actuator_motor = MOTOR_FORWARDS
    
  #   # If it has been extending long enough, continue to DUMPING phase
  #   elif time.time() - self.hopper_timer > HOPPER_EXTEND_AND_RETRACT_PERIOD:
  #     self.hopper_vibration_motor = MOTOR_STOP
  #     self.hopper_actuator_motor = MOTOR_STOP

  #     self.hopper_timer = time.time()

  #     self.hopper_state = HOPPER_STATE.DUMPING

  # # If the hopper is fully extended and the dumping phase is not over yet, continue
  # # dumping until the full dumping time period is over, continue to RETRACTING
  # elif self.hopper_state == HOPPER_STATE.DUMPING:
  #   # If it hasn't been paused long enough, continue vibrating
  #   if time.time() - self.hopper_timer < HOPPER_PAUSE_PERIOD:
  #     self.hopper_vibration_motor = MOTOR_FORWARDS
  #     self.hopper_actuator_motor = MOTOR_STOP
    
  #   # If it has been paused long enough, continue to RETRACTING
  #   elif time.time() - self.hopper_timer > HOPPER_PAUSE_PERIOD:
  #     self.hopper_vibration_motor = MOTOR_STOP
  #     self.hopper_actuator_motor = MOTOR_BACKWARDS

  #     self.hopper_state = HOPPER_STATE.RETRACTING

  # # If the linear actuators are retracting back into a resting state, then continue to RESTING
  # elif self.hopper_state == HOPPER_STATE.RETRACTING:
  #   # If they have not been retracting long enough, continue retracting
  #   if time.time() - self.hopper_timer < HOPPER_EXTEND_AND_RETRACT_PERIOD:
  #     self.hopper_vibration_motor = MOTOR_STOP
  #     self.hopper_actuator_motor = MOTOR_BACKWARDS
    
  #   # If it has been retracting long enough to be fully retracted, continue to RESTING
  #   elif time.time() - self.hopper_timer > HOPPER_EXTEND_AND_RETRACT_PERIOD:
  #     self.hopper_vibration_motor = MOTOR_STOP
  #     self.hopper_actuator_motor = MOTOR_STOP
  #     self.hopper_latch_servo = SERVO_FULL_CLOSE

  #     self.hopper_state = HOPPER_STATE.RESTING

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