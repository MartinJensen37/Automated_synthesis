# Node-RED Readme/Information

## Node-RED UI Flow Structure

This is the structure of the UI with all the subflows split into groups of different color. The main idea of this code is to be able to test various components of the solution individually, such as I/O control, MQTT message etc.

![image](https://user-images.githubusercontent.com/11269762/145584254-32782d5c-66f9-4ae5-b9c4-53a786a8cf12.png)

## Node-RED UI

The UI consists of a set of buttons(in blue) that can be used to control the UR5 robot as well as the smart pipette. Status messages can also be seen in the gray boxes. Some are related to the basic information provided by the RTDE interface from the UR robot and others are status messeges provided by the MQTT broker locally running on the computer.

![image](https://user-images.githubusercontent.com/11269762/146002784-dc643a3f-222b-47e5-b16d-c85eb67b93cd.png)
