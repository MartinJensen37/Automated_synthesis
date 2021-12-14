
from std_msgs.msg import String 
import rospy 

def main():
    rospy.init_node("apply_vacum_node") 
    publisher1 = rospy.Publisher('/ur_hardware_interface/script_command', String, quesize=2)
    try:
        #first 1 is the pin number and second number is the state of the pin 
        message = "sec MyProgram():" + "\n" + "set_standard_digital_output(1,0)" + "\n" + "end " + "\n"
        publisher1.publish(message)  
    except rospy.ServiceException:
        print ("Something went wrong: %s")