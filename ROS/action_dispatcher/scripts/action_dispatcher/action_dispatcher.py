import rospy
from geometry_msgs.msg import Pose
from actionlib import SimpleActionClient, SimpleActionServer
from actionlib_msgs.msg import GoalStatus
from manipulation.msg import ManipulationPlanRequest, ManipulationPlanResponse
from manipulation.srv import GetManipulationPlan, GetManipulationPlanResponse
from moveit_msgs.msg import MoveItErrorCodes, CollisionObject
from moveit_task_constructor_msgs.msg import ExecuteTaskSolutionAction, ExecuteTaskSolutionGoal
from moveit_msgs.msg import MoveItErrorCodes
from moveit_task_constructor_msgs.msg import ExecuteTaskSolutionAction, ExecuteTaskSolutionGoal, Solution
from action_dispatcher.msg import (DispatchActionsAction, DispatchActionsFeedback, DispatchActionsGoal, DispatchActionsResult)
from action_dispatcher.msg import ActionDispatcherState, DisplaySolution, MinimalSubTrajectory, JointTrajectoryPositions, SceneSetup
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from action_dispatcher.srv import SendSolution, SendSolutionRequest, SendSolutionResponse, SendActionState, SendActionStateRequest, SendActionStateResponse, SendManipulationPlanFeedbackRequest
from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest

class ActionDispatcher:
    def __init__(self):
        # Initialize action clients and servers
        self.get_manipulation_plan_client = None
        self.execute_manipulation_plan_client = None
        self.action_dispatcher_server = None
        self.action_dispatcher_state_publisher = None
        self.publish_display_solution = None
        self.send_solution = None
        self.send_action_state = None
        self.send_manipulation_plan_feedback = None
        self.get_planning_scene = None

        self.state = ActionDispatcherState.IDLE
        self.in_recovery_state = False
        self.action_solution = ExecuteTaskSolutionGoal()
        self.solution = Solution()
        self.is_executing = False
        self.task_id = 1
        self.done_status = None
        self.done_result = None
        self.feedback = None
        self.state_sent = ActionDispatcherState.IDLE

    def dispatch_actions_callback(self, goal):
        result = MoveItErrorCodes.SUCCESS
        self.in_recovery_state = False

        for action in goal.actions:
            self.state = ActionDispatcherState.PLANNING
            while result == MoveItErrorCodes.SUCCESS and self.state != ActionDispatcherState.IDLE:
                result = self.run_state_machine(action)

            if result in [MoveItErrorCodes.PREEMPTED, MoveItErrorCodes.PLANNING_FAILED]:
                break
        self.publish_state()
        self.set_final_result(result)

    def run_state_machine(self, action):
        self.send_manipulation_plan_feedback.call(SendManipulationPlanFeedbackRequest(action))
        self.publish_state()
        if self.action_dispatcher_server.is_preempt_requested():
            self.execute_manipulation_plan_client.cancel_goal()
            return MoveItErrorCodes.PREEMPTED
        
        if self.state == ActionDispatcherState.PLANNING:
            return self.handle_planning(action)
        
        elif self.state == ActionDispatcherState.SEND_PLAN:
            return self.handle_send_plan()

        elif self.state == ActionDispatcherState.MONITOR_EXECUTION:
            return self.handle_monitor_execution()
        
        elif self.state == ActionDispatcherState.RECOVERY:
            return self.handle_recovery(action)

        self.state = ActionDispatcherState.IDLE

        return MoveItErrorCodes.SUCCESS

    def publish_action_dispatcher_state(self):
        self.action_dispatcher_state_publisher.publish(self.state)

    def handle_planning(self, action):
        print(action)
        print("-----------------------------")
        result = self.compute_action_solution(action)
        if(result == MoveItErrorCodes.SUCCESS):
            self.extract_and_publish_display_solution()
        else:
            self.state = ActionDispatcherState.IDLE
            return MoveItErrorCodes.SUCCESS
        ######################################### SLEEP TIMER #########################################
        rospy.sleep(3) 
        self.state = ActionDispatcherState.SEND_PLAN
        return result

    def extract_and_publish_display_solution(self):
        display_solution = DisplaySolution()
        scene_setup = SceneSetup()

        # Set the task ID and start scene collision objects
        display_solution.task_id = self.task_id

        object_starts_attached = len(self.solution.start_scene.robot_state.attached_collision_objects) > 0

        # If object is attached to the robot add which one
        if object_starts_attached:
            current_attached_object_start_pose = PoseStamped()
            current_attached_object_start_pose.header.frame_id = self.solution.start_scene.robot_state.attached_collision_objects[0].object.id
            current_attached_object_start_pose.pose = self.solution.start_scene.robot_state.attached_collision_objects[0].object.pose
            scene_setup.current_attached_object_start_pose = current_attached_object_start_pose

        future_attached_object_start_pose = PoseStamped()

        for sub_trajectory in self.solution.sub_trajectory:
            minimal_sub_trajectory = MinimalSubTrajectory()

            # If scene_diff has collision object added, then an object was placed at some point, save name and pose
            if len(sub_trajectory.scene_diff.world.collision_objects) == 1:
                future_placed_object_end_pose = PoseStamped()
                future_placed_object_end_pose.header.frame_id = sub_trajectory.scene_diff.world.collision_objects[0].id
                future_placed_object_end_pose.pose = sub_trajectory.scene_diff.world.collision_objects[0].pose
                scene_setup.future_placed_object_end_pose = future_placed_object_end_pose

            object_is_attached = len(sub_trajectory.scene_diff.robot_state.attached_collision_objects) > 0

            # If scene diff has object attached 
            if object_is_attached and not object_starts_attached:
                future_attached_object_start_pose.header.frame_id = sub_trajectory.scene_diff.robot_state.attached_collision_objects[0].object.id
                minimal_sub_trajectory.object_state_change = MinimalSubTrajectory.OBJECT_ATTACHED
            elif not object_is_attached and object_starts_attached:
                minimal_sub_trajectory.object_state_change = MinimalSubTrajectory.OBJECT_DETACHED

            minimal_sub_trajectory.joint_names = sub_trajectory.trajectory.joint_trajectory.joint_names
            i = 0
            for point in sub_trajectory.trajectory.joint_trajectory.points:
                if i % 10 == 1:
                    minimal_sub_trajectory.positions.append(JointTrajectoryPoint(positions=point.positions))
                i += 1

            # Append the minimal sub-trajectory to the display solution
            display_solution.sub_trajectories.append(minimal_sub_trajectory)

    
        # If surfaces are referenced in task add them.
        for collision_object in self.solution.start_scene.world.collision_objects:
            if collision_object.type.key == "surface":
                surface_start_pose = PoseStamped()
                surface_start_pose.header.frame_id = collision_object.id
                surface_start_pose.pose = collision_object.pose
                scene_setup.involved_locations.append(surface_start_pose)
            if collision_object.id == future_attached_object_start_pose.header.frame_id:
                future_attached_object_start_pose.pose = collision_object.pose
        
        scene_setup.future_attached_object_start_pose = future_attached_object_start_pose
        display_solution.start_scene = scene_setup

        # Publish the display solution
        #self.publish_display_solution.publish(display_solution)
        solution_to_send = SendSolutionRequest()
        solution_to_send.solution = display_solution
        self.send_solution.call(solution_to_send)
        # Increment the task ID
        self.task_id = self.task_id + 1

    def is_object_attached(self):
        # Define the request and specify what parts of the scene we need (we want robot state including attached objects)
        scene_request = GetPlanningSceneRequest()
        scene_request.components.components = scene_request.components.ROBOT_STATE_ATTACHED_OBJECTS
        
        try:
            # Call the service
            planning_scene = self.get_planning_scene(scene_request)
            
            # Check if the object is attached to the robot
            attached_objects = planning_scene.scene.robot_state.attached_collision_objects
            if len(attached_objects) > 0:
                rospy.loginfo(f"Replaying place action for object")
                return True
            return False

        except rospy.ServiceException as e:
            rospy.logerr("Service call to get_planning_scene failed: %s" % e)
            return False

    def handle_send_plan(self):
        self.execute_manipulation_plan_client.send_goal(self.action_solution, done_cb=self.execution_done_callback, feedback_cb=self.execution_feedback_callback)
        self.reset_monitor_variables()
        self.state = ActionDispatcherState.MONITOR_EXECUTION
        return MoveItErrorCodes.SUCCESS

    def reset_monitor_variables(self):
        self.done_status = None
        self.done_result = None
        self.feedback = None

    def publish_state(self):
        if self.state_sent != self.state:
            to_send = SendActionStateRequest()
            to_send.state.state = self.state
            self.send_action_state(to_send)
            self.state_sent = self.state

    def handle_monitor_execution(self): 
        if self.done_status is not None: 
            if self.done_status == GoalStatus.SUCCEEDED:
                self.in_recovery_state = False
                self.state = ActionDispatcherState.IDLE
                return MoveItErrorCodes.SUCCESS
            elif self.done_status == GoalStatus.ABORTED and not self.in_recovery_state:
                self.in_recovery_state = True
                rospy.loginfo("[action_dispatcher_node]: Execution failed, attempting recovery")
                self.state = ActionDispatcherState.RECOVERY
                return MoveItErrorCodes.SUCCESS
            elif self.done_status == GoalStatus.PREEMPTED:
                self.state = ActionDispatcherState.IDLE
                return MoveItErrorCodes.PREEMPTED
            elif self.done_status == GoalStatus.PENDING or self.done_status == GoalStatus.ACTIVE:
                return MoveItErrorCodes.SUCCESS
            else: 
                rospy.loginfo("[action_dispatcher_node]: Recovery failed, exiting")
                self.state = ActionDispatcherState.IDLE
            return MoveItErrorCodes.PLANNING_FAILED
        else:
            return MoveItErrorCodes.SUCCESS
        
    def handle_recovery(self, action):
        replan = self.get_recovery_plan(action)
        if(replan == MoveItErrorCodes.SUCCESS):
            self.extract_and_publish_display_solution()
        if replan:
            if self.compute_action_solution(replan) == MoveItErrorCodes.FAILURE:
                rospy.loginfo("[action_dispatcher_node]: Failed to get recovery plan, exiting")
                return MoveItErrorCodes.PLANNING_FAILED
        else:
            rospy.loginfo("[action_dispatcher_node]: Recovery failed, exiting")
            return MoveItErrorCodes.PLANNING_FAILED
        
        self.state = ActionDispatcherState.SEND_PLAN
        return MoveItErrorCodes.SUCCESS

    def handle_solution(self, solution):
        self.solution = solution

    def get_recovery_plan(self, action):
        if action.task_type == ManipulationPlanRequest.PLACE:
            if self.is_object_attached():
                return ManipulationPlanRequest(task_type=ManipulationPlanRequest.PLACE, target_object_name=action.target_object_name, task_name="Recovery Place")
            else:
                return ManipulationPlanRequest(task_type=ManipulationPlanRequest.READY_POSE, target_object_name="", task_name="Ready Pose")
        elif action.task_type == ManipulationPlanRequest.PICK:
            return action
        return None

    def compute_action_solution(self, task, max_tries=2):
        for _ in range(max_tries):
            response = self.get_manipulation_plan_client(task)
            if response.manipulation_plan_response.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("[action_dispatcher_node]: Plan computed")
                self.action_solution.solution = response.manipulation_plan_response.solution
                return MoveItErrorCodes.SUCCESS
            rospy.loginfo("[action_dispatcher_node]: Failed to get plan, trying again...")
        
        rospy.loginfo("[action_dispatcher_node]: Failed to get plan after {} tries, exiting".format(max_tries))
        return MoveItErrorCodes.PLANNING_FAILED
    
    def execution_feedback_callback(self, feedback):
        self.feedback = feedback
    
    def execution_done_callback(self, status, result):
        self.done_status = status
        self.done_result= result

    def publish_feedback(self, action):
        feedback = DispatchActionsFeedback()
        feedback.current_action = action
        self.action_dispatcher_server.publish_feedback(feedback)

    def publish_action_dispatcher_state(self):
        self.action_dispatcher_state_publisher.publish(self.state)

    def set_final_result(self, result):
        response = DispatchActionsResult()
        response.error_code.val = result
        if result == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("[action_dispatcher_node]: Action plan completed successfully")
            self.action_dispatcher_server.set_succeeded(response)
        elif result == MoveItErrorCodes.PREEMPTED:
            rospy.loginfo("[action_dispatcher_node]: Action plan preempted")
            self.action_dispatcher_server.set_preempted(response)
        else:
            rospy.loginfo("[action_dispatcher_node]: Action plan failed")
            self.action_dispatcher_server.set_aborted(response)
