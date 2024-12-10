# MARCER System Overview
This project provides a simulation of our system, MARCER, as well as a description of each component of our system architecture. Users can run our system in Unity while interacting with a simulated robot running in ROS. If the reviewer has a HoloLens, they may also run our Unity program using the holographic remoting application on the HoloLens device.

## System Architecture
![Example workflow showing interaction between user and robot](/Figures/system_diagram.png)

### Speech Processor
The user first begins a command with the keywords "Hey Fetch," followed by their command to the robot. The speech processor will capture the user's voice using the [PyAudio](https://pypi.org/project/PyAudio/) package and convert it to text using the [Whisper Speech Recognition package](https://cloud.google.com/speech-to-text). The text is then sent to the Rule Generator over the /user_input/speech topic.

### Rule Generator
Before processing commands, the ContextBuilder queries the robot's scene graph to gather environmental context, including available objects, surface locations, spatial relationships (like which objects are supported by or contained within surfaces), and the robot's currently held objects. Then, a three-stage process is utilized for generating the Rule with the LLM. First, we analyze the user's speech command to determine appropriate triggers (time-based, zone-based, or combined), then generate a Python function implementation with relevant manipulation constraints (such as requiring objects to be picked before placing), and finally creating a human-readable description of the rule's behavior. Following this process, the rule generator waits for the user to approve, provide feedback for regeneration, or reject the generated rule. Throughout this process, the PromptBuilder class constructs the LLM prompts by incorporating relevant examples and constraints based on the specific trigger type identified. Successfully approved rules are then sent to the Rule Monitor using a custom Rule.msg that contains the rule's python function, description, and operation (Add, Remove).

### Rule Monitor
The Rule Monitor serves as the manager for TAP rules, which are implemented as Python methods. When adding a new rule, the Rule Monitor first initializes any required timers. Then, it adds the function to a local dictionary using Python’s exec() function. We add this function call to a call_functions dictionary, where it can be accessed by its rule name. The Rule Monitor can modify these stored rules to enable, disable, remove, or reorder them. At runtime, the Rule Monitor continuously executes each TAP rule in call_functions. When a rule’s check_condition() returns true, the system uses each action call (e.g., pick(), place(), or pour()) to add the action to a list. Upon reaching the done() function, the action list is sent to the Action Dispatcher via a service call for robot execution.

### Action Dispatcher
The Action Dispatcher manages the execution of robot manipulation actions by coordinating planning and execution through a state machine. It receives action requests (like pick and place operations) and processes them sequentially. For each action, it first enters a planning state where it computes the manipulation solution by querying the planning service, allowing up to two retry attempts if planning fails. Once a valid plan is obtained, the system extracts and processes the solution into a minimal representation that includes joint trajectories, object attachments/detachments, and relevant scene information (like surface locations and object poses). This is necessary so that the solution can be displayed in the HoloLens interface. The Action Dispatcher then sends this plan for execution and monitors its progress. If execution fails, it enters a recovery state where it attempts to generate and execute recovery behaviors - for example, if a place action fails while an object is still attached, it will attempt to place again, otherwise it will return to a ready pose. Throughout this process, the system publishes state updates and feedback, allowing other components to track the progress of manipulation tasks. The state machine continues until all actions are successfully completed, preempted, or determined to be unrecoverable.

### Manipulation Node
The Manipulation Node implements robot manipulation tasks through built on the [MoveIt Task Constructor](https://github.com/moveit/moveit_task_constructor). Each manipulation primitive (pick, place, move, etc.) inherits from the TaskBase class, which provides the functions for planning and executing a task. Each task then registers themselves to the Task Factory by mapping their task type to a task creation function in a static registry. When the Manipulation Node receives a planning request, it uses the TaskFactory to instantiate the appropriate task type. Then, the task executes the planning method, which generates a motion plan for the task. If planning is successful, the task extracts the solution message containing the motion plan and publishes it to be displayed in the interface and executed by the robot. Once the Manipulation node receives a 'DONE' task type, it resets and clears all existing task plans. 

### Scene Graph
The Scene Graph maintains a directed graph representation of objects and their spatial relationships in the robot's environment. The SceneGraphNode class initializes the graph by querying the planning scene to get information about surfaces (like tables and shelves) and objects, including their positions, orientations, dimensions, and shapes. The system continuously updates relationships between objects through two main sources: object detections from a perception system and virtual zones created by the user. For surfaces, it stores additional metadata like subframe names and poses that represent valid placement locations. The Scene Graph provides a query service that can retrieve various spatial relationships - such as what objects are supported by a surface, what surface an object is on, or what objects are inside a zone. To determine these relationships, the system performs geometric calculations, checking if objects are within the boundaries of surfaces or zones by transforming their coordinates into the appropriate reference frames. The system visualizes the graph using NetworkX, where surfaces are shown in brown, zones in blue, and objects in green, with edges representing relationships like "supports" or "has_inside". This graph visualization updates in real-time as the robot's environment changes.

### Language Model Node
The Language Model Node serves as a ROS interface to OpenAI's language model (gpt-4o). The node initializes with configuration parameters including an OpenAI API key and model selection, setting up CUDA support when available for GPU acceleration. When it receives a service query, the node processes the ROS message into a chat format, using primary and secondary roles with content items. It then sends this formatted prompt with specific sampling parameters using the OpenAI API, and picks the response with the highest calculated mean log probability across tokens. The node then sends the output back as a service response.

### Object Tracker Node
The object tracker is implemented in the Perception class. Currently, the perception node sends object types and poses using [vision_msgs](https://docs.ros.org/en/noetic/api/vision_msgs/html/index-msg.html) so that future iterations of this system might incorporate vision models to track objects. However, in its current form, the object tracking utilizes a Vicon motion tracking system as a stand in. 

## Requirements
To run our program reviewers will need to have the following. 
ROS Noetic
Unity 2020.3.38f1 or greater
git downloaded on windows

## Setting up Unity
### Step 1
Open Unity Hub then click "Open" in the top right

### Step 2
Navigate to the unity folder and press open

### Step 3
Click the project that was added to Unity Hub and let it download

### Step 4
If the project does not automatically open with the demo scene, navigate to Assets -> ProgramAR -> Scenes and click demo_scenes

## Setting up ROS
### Step 1
In your ROS environment create a workspace 

```
mkdir catkin_ws
cd catkin_ws/
mkdir src
```

### Step 2
Then copy the files from the folder ros, into the src directory. 
Then add your OpenAI API key to the file language_model_node.py in the language_model folder

Clone moveit_task_constructor
``` 
git clone --no-checkout https://github.com/moveit/moveit_task_constructor.git
cd moveit_task_constructor
git checkout d95a2fc
```

Then install dependencies 

```
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Step 3
Once that is done, build your project using 
```
catkin build
```
or 

```
catkin_make
```
### Step 4
Once the build is finished, source the directory

```
source devel/setup.bash
```
### Step 5
Finally to run the ROS program, execute the command

```
roslaunch action_dispatcher demo.launch
```

In another terminal run the command

```
roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=127.0.0.1 tcp_port:=10000
```

### Step 6
In Unity press play and the system should be running.

Note: If the ROS IP in the top left of the game view is red rather than a blue/green color, you may need to stop Unity, relaunch the ROS files from the previous Step 5, then replay Unity. Once the Unity system is running, you can hold down the right click button on the mouse and move around the scene using the wasd keys. To click on buttons or move zones around the scene, use the left clicker on your mouse. 

### Step 7
In two more seperate terminals run the commands:

```
rosrun rule_monitor rule_monitor_node.py 
```
and

```
rosrun user_input speech_to_text.py
```
The speech to text script needs a microphone to run.

### Step 8
Begin interacting with the system by saying, "hey fetch" followed by a command.


## Using the HoloLens
If you would like to use your HoloLens with the demo simulation, first download and print off the QR code provided with the files, put it on a table, then follow these steps. 

### Step 1
Turn on your HoloLens and put it on. Then download the application called Holographic Remoting. Once downloaded, navigate to the application and run it. 

### Step 2
Make sure the HoloLens is on the same network as your computer running Unity. Then write down the IP Address shown in the Holographic Remoting app. 

### Step 3
Go back to the Unity application then navigate to the tabs at the top and click Mixed Reality -> Remoting -> Holographic Remoting for Play Mode. Then type in the IP Address into the "Remote Host Name". Finally, click Enable Holographic Remoting for Play Mode. 

### Step 4
Go to the Hierarchy on the left side of the application and click "QRManager". Then, click the three checkboxes associated with each attached component. 

### Step 5
Press play on the Unity application then look down at the QR code. Wait for the HoloLens to display the demo and move freely around the scene to interact with our system. 
