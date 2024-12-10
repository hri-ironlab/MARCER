# MARCER System Overview
This project provides reviewers with a simulation of our system MARCER. Users can run our system in Unity while interacting with a simulated robot running in ROS. If the reviewer has a HoloLens, they may also run our Unity program using the holographic remoting application on the HoloLens device.

## Requirements ##
To run our program reviewers will need to have the following. 
ROS Noetic
Unity 2020.3.38f1 or greater
git downloaded on windows

## Setting up Unity ##
### Step 1 ###
Open Unity Hub then click "Open" in the top right

### Step 2 ###
Navigate to the unity folder and press open

### Step 3 ###
Click the project that was added to Unity Hub and let it download

### Step 4 ###
If the project does not automatically open with the demo scene, navigate to Assets -> ProgramAR -> Scenes and click demo_scenes

## Setting up ROS ##
### Step 1 ###
In your ROS environment create a workspace 

```
mkdir catkin_ws
cd catkin_ws/
mkdir src
```

### Step 2 ###
Then copy the files from the folder ros, into the src directory. 
Then add your OpenAI API key to the file language_model_node.py in the language_model folder

### Step 3 ###
Once that is done, build your project using 
```
catkin build
```
or 

```
catkin_make
```
### Step 4 ###
Once the build is finished, source the directory

```
source devel/setup.bash
```
### Step 5 ###
Finally to run the ROS program, execute the command

```
roslaunch action_dispatcher demo.launch
```

In another terminal run the command

```
roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=127.0.0.1 tcp_port:=10000
```

### Step 6 ###
In Unity press play and the system should be running.

Note: If the ROS IP in the top left of the game view is red rather than a blue/green color, you may need to stop Unity, relaunch the ROS files from the previous Step 5, then replay Unity. Once the Unity system is running, you can hold down the right click button on the mouse and move around the scene using the wasd keys. To click on buttons or move zones around the scene, use the left clicker on your mouse. 

### Step 7 ###
In two more seperate terminals run the commands:

```
rosrun rule_monitor rule_monitor_node.py 
```
and

```
rosrun user_input speech_to_text.py
```
The speech to text script needs a microphone to run.

### Step 8 ###
Begin interacting with the system by saying, "hey fetch" followed by a command.


## Using the HoloLens ##
If you would like to use your HoloLens with the demo simulation, first download and print off the QR code provided with the files, put it on a table, then follow these steps. 

### Step 1 ###
Turn on your HoloLens and put it on. Then download the application called Holographic Remoting. Once downloaded, navigate to the application and run it. 

### Step 2 ### 
Make sure the HoloLens is on the same network as your computer running Unity. Then write down the IP Address shown in the Holographic Remoting app. 

### Step 3 ###
Go back to the Unity application then navigate to the tabs at the top and click Mixed Reality -> Remoting -> Holographic Remoting for Play Mode. Then type in the IP Address into the "Remote Host Name". Finally, click Enable Holographic Remoting for Play Mode. 

### Step 4 ###
Go to the Hierarchy on the left side of the application and click "QRManager". Then, click the three checkboxes associated with each attached component. 

### Step 5 ###
Press play on the Unity application then look down at the QR code. Wait for the HoloLens to display the demo and move freely around the scene to interact with our system. 