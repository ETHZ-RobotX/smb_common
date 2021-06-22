#!/usr/bin/env python3  
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from actionlib_msgs.msg import GoalID

msg_print_counter = 0
eStopHasBeenCalled = False   

def joyCallback(joyMsg):
  global msg_print_counter
  global eStopHasBeenCalled

  #if eStopHasBeenCalled == True:
  #  msg_print_counter = msg_print_counter + 1
  #  if msg_print_counter == 200:
  #    print("[joy_e_stop_node] Soft e-stop is still active. Press green A-button to deactivate")
  #    msg_print_counter = 0

  if joyMsg.buttons[2] == 1 and eStopHasBeenCalled == False:
    rospy.logwarn("[joy_e_stop_node] Soft e-stop is ACTIVATED! Robot is frozen. (Press green A-button to deactivate)")
    eStopHasBeenCalled = True
    msg_print_counter = 0
    eStopMsg = Bool()
    eStopMsg.data = True
    eStopPublisher.publish(eStopMsg)
    emptyNavigationMsg = GoalID()
    cancelNavigationPlanPublisher.publish(emptyNavigationMsg)
  elif joyMsg.buttons[1] == 1 and eStopHasBeenCalled == True:
    rospy.logwarn("[joy_e_stop_node] Soft e-stop is DEACTIVATED!")
    eStopHasBeenCalled = False
    eStopMsg = Bool()
    eStopMsg.data = False
    eStopPublisher.publish(eStopMsg)
    emptyNavigationMsg = GoalID()
    cancelNavigationPlanPublisher.publish(emptyNavigationMsg)

if __name__ == '__main__':
  rospy.init_node('joy_e_stop')

  rospy.Subscriber("joy", Joy, joyCallback)
  eStopPublisher = rospy.Publisher("e_stop", Bool, queue_size=10)
  cancelNavigationPlanPublisher = rospy.Publisher("/move_base/cancel", GoalID, queue_size=10)

  rospy.spin()

  # Should we keep the repeating reminder printouts when controller is in e-stopped state?
  # Should we also cancel the planner when un-e-stopping in order to prevent potentially unexpected motion?
  # Is it okay or bad practice to pull variables into function scope with "global" keyword?
