########################### IMPORTED MODULES AND MESSAGES ###########################################

# Standard python libraries
from ctypes import string_at
import time
import sys, os
import argparse
from argparse import RawTextHelpFormatter
from dataclasses import dataclass
from typing import Dict

# ROS standard libraries and msgs
import rospy
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import String

# ROS UR specific modules
from ur_msgs.msg import IOStates
from ur_msgs.srv import SetIO
from ur_dashboard_msgs.srv import Load, IsProgramSaved

# MQTT paho specific modules
import paho.mqtt.client as mqtt

########################## GLOBAL VARIABLES AND CONSTANTS ###########################################

# create commandline arguments for recipes 
parser = argparse.ArgumentParser(description="ROS program used to interface an MQTT driven dosing pipette with a UR5 robot.", formatter_class=RawTextHelpFormatter)
parser.add_argument('-r', '--recipe', help="Choose the recipe for the robot to create: \n 1: mixture with 2 components \n 2: mixture with 3 components \n 3: mixture with 4 components", type=int)
parser.add_argument('-d', '--reset', help="Reset all the relevant digital output pins on the robot.", action='store_true')
 
args = parser.parse_args()

# Define constants for I/O used on the UR5 robot
LARGE_TT_1 = 2
LARGE_TT_2 = 3  
SMALL_TT_1 = 4
SMALL_TT_2 = 5
SMALL_TT_3 = 6
SMALL_TT_4 = 7
CLEANING_STATION = 8
PROG_FINISH = 9

EMPTY_TOPIC = "pump/empty"
FILL_TOPIC = "pump/fill"
STATUS_TOPIC = "pump/status"
RECIPE_TOPIC = "recipe"
RESET_TOPIC = "reset"

########################## CLASSES AND FUNCTION DEFINITIONS #########################################

@dataclass
class recipePicker():
    recipe_nr: int
    status_pin: int
    progress_count:int
    
    def get_units(self):
        units = {'1': [200, 300], '2': [1000, 500, 700], '3': [500, 700, 800, 1000]}
        return units[str(self.recipe_nr)]
    
    def empty_fill(self):
        if self.status_pin in range(SMALL_TT_1, SMALL_TT_4+1):
            return [FILL_TOPIC]
        elif self.status_pin in range(LARGE_TT_1, LARGE_TT_2+1):
            return [EMPTY_TOPIC]
        elif self.status_pin == CLEANING_STATION:
            return [FILL_TOPIC, EMPTY_TOPIC]
        else:
            print("The status pin is either out of scope or the program is finished. status_pin: ", self.status_pin)
            return ["none"]

    def progress(self):
        recipe_units = self.get_units()
        current_unit = recipe_units[self.progress_count]
        topics = self.empty_fill()

        if len(topics) > 1:
            return {'topic1': topics[0], 'topic2': topics[1], 'units': current_unit}
        else: 
            return {'topic1': topics[0], 'topic2': None, 'units': current_unit}

class serviceController():

    def __init__(self) -> None:
        rospy.init_node("service_control_node")
        rospy.Subscriber('/ur_hardware_interface/io_states', IOStates, callback=self.check_io_pin, queue_size=None)
        self.pins = [LARGE_TT_1, LARGE_TT_2, SMALL_TT_1, SMALL_TT_2, SMALL_TT_3, SMALL_TT_4, CLEANING_STATION]
        self.prior_pin = 0
        self.recipe = None

        # Initialize and establish connection to MQTT broker 
        broker_address="192.168.1.100"
        self.mqtt_client = mqtt.Client("Smart pipette client") #create new instance
        self.mqtt_client.on_message = self.mqtt_cb #attach function to callback
        result = self.mqtt_client.connect(broker_address, 1883) #connect to broker
        self.mqtt_client.subscribe([(STATUS_TOPIC, 0), (RECIPE_TOPIC, 0), (RESET_TOPIC, 0)]) # Subscribe to topics
        self.mqtt_client.loop_start()
        self.mqtt_msg = ''

        # reset and progress counters used to keep track of where in the process the robot is
        self.res_flag = 0
        self.progress_count = 0
        self.wait_for_message = 0
        self.send_once = 0
        self.increment_once = 0


    def play(self): # plays the current script on the robot
        service = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)
        signal = TriggerRequest()
        result = service(signal)
        return result


    def pause(self): # pauses the current script on the robot
        service = rospy.ServiceProxy('/ur_hardware_interface/dashboard/pause', Trigger)
        signal = TriggerRequest()
        result = service(signal)
        return result


    def stop(self): # stops the current script on the robot
        service = rospy.ServiceProxy('/ur_hardware_interface/dashboard/stop', Trigger)
        signal = TriggerRequest()
        result = service(signal)
        return result


    def load_program(self, filename): # loads a script with a given filename on the UR robot
        service = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)
        select_program = filename
        result = service(select_program)
        return result


    def close_popup(self): # closes any non-safety pop-up windows for the program to continue
        service = rospy.ServiceProxy('/ur_hardware_interface/dashboard/close_popup', Trigger)
        signal = TriggerRequest()
        result = service(signal)
        return result


    def reset_pin(self, pin): # resets the relevant I/O pins on the robot 
        print("This is the pin number that is being reset: ", pin)
        reset = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        reset(fun = 1, pin = pin, state = False)


    def check_io_pin(self, message): # callback function of the rostopic subscriber
        if self.recipe == None: return # guard clause that will exit if the recipe has not yet been defined

        if message.digital_out_states[PROG_FINISH].state == True: # check if program has finished and reset the recipe flag
            self.res_flag = 0
            self.progress_count = 0
            self.recipe = ''
            self.reset_pin(PROG_FINISH)
            print("Program finished! Pick a new program.")

        for pin in self.pins: # look through all the pin states to find which state the robot is in and act accordingly
            pin_state = message.digital_out_states[pin].state
            if  pin_state == True and self.prior_pin != pin and self.wait_for_message == 1: # If is the current pin is high/true send a command to the pipette(empty or fill)
                temp = recipePicker(self.recipe, pin, self.progress_count)
                recipe_dict = temp.progress()
                print("This is the MQTT message before sending: ", self.mqtt_msg)
                time.sleep(0.2)

                if self.send_once == 0:
                    self.mqtt_client.publish(recipe_dict['topic1'], recipe_dict['units'])
                    self.send_once = 1
                
                if self.mqtt_msg == "1":
                    print("Finished extraction/depositing of solution!")
                    self.reset_pin(pin)
                    self.close_popup()
                    self.mqtt_msg = ''
                    self.wait_for_message = 0
                    self.prior_pin = pin
                    self.send_once = 0
                    self.increment_once = 0
                
                if self.increment_once == 0 and message.digital_out_states[LARGE_TT_1].state == True or message.digital_out_states[LARGE_TT_2].state == True:
                    self.progress_count = self.progress_count + 1
                    print("incrementing the progress. This is the current count: ", self.progress_count)
                    self.increment_once = 1

    def mqtt_cb(self, mqttself, client, message): # callback function of the MQTT subscriber
        if message.topic == "pump/status": # checks if the pipette is ready or busy
            self.mqtt_msg = message.payload.decode("utf-8")
            self.wait_for_message = 1
            return

        if message.topic == "reset": # resets the I/O using the reset method of the serviceController class
            for pin in self.pins:
                self.reset_pin(pin)
            print("All pins have been reset now!")

        saved = rospy.ServiceProxy('/ur_hardware_interface/dashboard/program_saved', IsProgramSaved)
        if saved().program_saved == False: # guard clause to protect program from not being saved
            print("The program is not saved!") 
            return

        if message.payload.decode("utf-8") == "1" and self.res_flag == 0: # checks which recipe to execute and if a program is currently running
            self.recipe = 1
            #self.load_program('bubble_project/bubble_project_kelvin_tool.urp')
            self.play()
            self.res_flag = 1

        elif message.payload.decode("utf-8") == "2" and self.res_flag == 0:
            self.recipe = 2
            #self.load_program('bubble_project/bubble_project_put_L_tt.urp')
            #self.play()
            #self.res_flag = 1

        elif message.payload.decode("utf-8") == "3" and self.res_flag == 0:
            self.recipe = 3
            #self.load_program('bubble_project/bubble_project_kelvin_tool.urp')
            #self.play()
            #self.res_flag = 1
  
####################################### MAIN LOOP ###################################################

def main():
    serviceController() # initialize the class

    rospy.spin() # spin to keep the program alive while no rostopic messages arrive

if  __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt or rospy.ROSInterruptException: # check is the program has been interrupted if it has then exit
        print("interrupted")
        try:
            sys.exit(0) 
        except SystemExit:
            os._exit(0)


