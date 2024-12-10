#!/usr/bin/env python3

import copy
import rospy
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, TriggerResponse
from scene_graph.srv import QuerySceneGraph, QuerySceneGraphRequest, QuerySceneGraphResponse
from rule_monitor.msg import Rule, TriggerType, ActiveRules
from rule_monitor.srv import UpdateRule, UpdateRuleRequest, UpdateRuleResponse
import types 
from geometry_msgs.msg import Pose
from manipulation.msg import ManipulationPlanRequest
from action_dispatcher.msg import DispatchActionsGoal
import actionlib
import atexit
from rule_monitor.rule_storage_manager import RuleStorageManager
from moveit_msgs.msg import MoveItErrorCodes
from actionlib import GoalStatus
from rule_monitor.srv import GetRuleDescription, GetRuleDescriptionResponse
from rule_monitor.context_builder import Context, ContextBuilder
from rule_monitor.prompt_builder import PromptBuilder
from rule_monitor.conditions import Conditions
from rule_monitor.srv import SendActiveRulesRequest, SendActiveRulesResponse

def build_action_request(task_type, target_object_name="", description="", place_pose=Pose()) -> ManipulationPlanRequest:
    return ManipulationPlanRequest(
        task_type=task_type,
        target_object_name=target_object_name,
        task_name=description,
        place_pose=place_pose
    )

# from manipulation_plan.msg import ManipulationPlanRequest

class RuleMonitor:
    def __init__(self):
        self.scene_graph_client = None
        self.action_dispatcher_client = None
        self.active_rules_publisher = None
        self.language_model_service = None
        self.send_active_rules = None

        self.is_stopped = False
        self.is_ready_for_command = True

        self.rules = []
        self.active_timers = {}
        self.local_vars = {}
        self.call_functions = {}
        self.action_queue = DispatchActionsGoal()
        self.action_queue.actions = []
        self.executing_rule_index = 0
        self.next_rule = 0
        self.paused = False; 

        # Save Load        
        self.rule_storage_manager = RuleStorageManager("rules.json")
        atexit.register(self.save_rules)

        self.conditions = Conditions()
        self.context = Context()
        self.context_builder = ContextBuilder()
        self.prompt_builder = PromptBuilder()
        self.rule_to_delete = None
        self.object_list = self.conditions.all_scene_objects 

    def load_rules(self):
        self.load_rules()

    def run_rules(self):
        print("is_ready_for_command:", self.is_ready_for_command)
        if len(self.rules) > 0:
            if self.is_ready_for_command and not self.is_stopped and not self.paused:
                if self.rule_to_delete is not None:
                    self.remove_rule(self.rule_to_delete)
                    self.rule_to_delete = None
                    if len(self.rules) == 0:
                        return 
                    if self.next_rule != 0:
                        self.next_rule = self.next_rule - 1

                self.executing_rule_index = self.next_rule
                current_rule = self.rules[self.executing_rule_index]
                if current_rule.is_active:
                    print("calling rule: " + current_rule.name)
                    self.call_functions[current_rule.name]()
                    if current_rule.type.type == TriggerType.NONE:
                        self.rule_to_delete = current_rule.name
                    if current_rule.type.type == TriggerType.TIMER and not self.is_ready_for_command:
                        self.rule_to_delete = current_rule.name
                self.next_rule += 1
                if self.next_rule >= len(self.rules):
                    self.next_rule = 0
        else:
            self.is_ready_for_command = True

    def add_rule(self, rule):
        # self.active_rules.append(rule.name)
        # self.rules[rule.name] = rule
        # print(rule.name)
        # print('-----------------')
        # print(rule.function)
        # print('-----------------')
        # print(rule.description)
        # print('-----------------')
        if "self.check_condition" in rule.function and "type='time'" in rule.function:
            self.conditions.add_function_timer(rule.name)
        self.rules.append(rule)
        self.add_function(rule.name, rule.function)
        self.publish_active_rules()
        
    def handle_get_rule_description(self, req):
        for rule in self.rules:
            if rule.name == req.name:
                return GetRuleDescriptionResponse(description=rule.name + "\n" + rule.description)
        return ""

    def handle_pause_rule_monitor(self, req):
        self.paused = not self.paused
        return TriggerResponse(success=True, message="Rule Monitor is now paused." if self.paused else "Rule Monitor is now running.")

    def remove_rule(self, rule_name):
        for rule in self.rules:
            if rule.name == rule_name:
                self.rules.remove(rule)
                break
        
        self.remove_function(rule_name)
        self.publish_active_rules()

    def add_function(self, rule_name, rule_code):
        exec(rule_code, globals(), self.local_vars)
        self.call_functions[rule_name] = types.MethodType(self.local_vars[rule_name], self)

    def remove_function(self, rule_name):
        # Remove the function from the rules dictionary
        if rule_name in self.call_functions:
            del self.call_functions[rule_name]
        
        # Remove the function from the local_vars dictionary if needed
        if rule_name in self.local_vars:
            del self.local_vars[rule_name]

    def action_dispatcher_done_callback(self, state, result):
        if result.error_code.val == MoveItErrorCodes.SUCCESS: 
            self.is_ready_for_command = True
            rospy.loginfo("Action succeeded!")
        else:
            self.is_ready_for_command = True
            rospy.loginfo("Action dispatcher error received: " + str(state))

    def publish_active_rules(self):
        rules = ActiveRules()
        # Create a deep copy of self.rules to avoid directly modifying it
        # dont need to do this anymore since not publishing but kept it
        rules.rules = copy.deepcopy(self.rules)
        rules.executing_rule_index = self.executing_rule_index
        rules_request = SendActiveRulesRequest()
        rules_request.active_rules = rules
        self.send_active_rules.call(rules_request)
        #self.active_rules_publisher.publish(rules)

    def handle_update_rule_request(self, req):
        if req.rule.operation == Rule.ADD:
            self.add_rule(req.rule)
        elif req.rule.operation == Rule.REMOVE:
            self.remove_rule(req.rule.name)
        elif req.rule.operation == Rule.TOGGLE_IS_ACTIVE:
            self.toggle_is_active(req.rule.name)
        elif req.rule.operation == Rule.UPDATE_INDEX:
            self.update_rule_index(req.rule)
        rules = ActiveRules()
        rules.rules = self.rules
        rules.executing_rule_index = self.executing_rule_index
        update = UpdateRuleResponse()
        update.active_rules = rules
        return update

    def update_rule_index(self, rule):
        # Find the index of the rule with the matching name
        for i, r in enumerate(self.rules):
            if r.name == rule.name:
                # Ensure the new index is valid
                if 0 <= rule.new_index < len(self.rules):
                    # Remove the rule from the current position
                    temp = self.rules.pop(i)
                    # Insert the rule at the new index
                    self.rules.insert(rule.new_index, temp)
                else:
                    raise IndexError(f"Invalid new_index: {rule.new_index}. Must be within the range of the rules list.")
                break
        else:
            raise ValueError(f"Rule with name '{rule.name}' not found in the rules list.")

    def toggle_is_active(self, rule_name):
        if not self.paused:
            for rule in self.rules:
                if rule.name == rule_name:
                    rule.is_active = not rule.is_active
                    break

    def handle_delete_zone(self, msg):
        for trigger in self.trigger_list_request.triggers:
            if trigger.zone_name == msg.data or msg.data in trigger.action:
                self.delete_trigger(trigger.id)
        self.update_unity_trigger_list()

    def handle_stop_request(self, req):
        if self.is_stopped:
            self.is_stopped = False
            self.is_ready_for_command = True
            return TriggerResponse(success=True, message="Rule Monitor is now running.")
        else:
            state = self.action_dispatcher_client.get_state()
            if state in [GoalStatus.ACTIVE, GoalStatus.PENDING]:
                self.action_dispatcher_client.cancel_goal()
            self.is_stopped = True
            return TriggerResponse(success=True, message="Rule Monitor is now stopped.")
    
    ########################################## TRIGGERS ##########################################

    def check_condition(self, condition, type="", function=""):
        context = self.context_builder.get_context()
        if type == "location":
            print("Checking zone: " + condition)
            # check scene graph for object zone
            if False:
                # check if object is in zone
                return True
            else:
                # check from LLM
                return self.generate_condition_boolean(condition, context)
        elif type == "time":
            print("Checking time: " + condition)
            return self.conditions.check_function_time(condition, function)
        elif type == "location and time":
            print("Checking object location time: " + condition)
            scene_object = ""
            scene_location = ""

            for object in context.object_names:
                if condition.startswith(object):
                    scene_object = object
                    break

            for location in context.locations:
                if location in condition:
                    scene_location = location
                    break
                
            return self.conditions.check_object_time(condition, scene_object, scene_location)
        else:
            print("Checking unknown condition: " + condition)
            return self.generate_condition_boolean(condition, context)
    
    def generate_condition_boolean(self, condition, context):
        # Prompt for trigger generation
        condition_prompt = self.prompt_builder.get_check_condition_prompt(context, condition)
        response = self.language_model_service.call(condition_prompt)
        # self.generated_trigger_type = self.get_trigger_type(response.response)

        print('------------------', response.response, context.object_locations, '------------------')

        if ('true' in response.response.lower()) and ('false' not in response.response.lower()):
            return True
        else:
            return False

    ########################################## ACTIONS ##########################################
    
    def pour(self, object_name):
        self.action_queue.actions.append(build_action_request(ManipulationPlanRequest.POUR, object_name.lower(), "Pour Object"))
        print(f"Pouring in {object_name}")

    def wipe(self, location):
        self.action_queue.actions.append(build_action_request(ManipulationPlanRequest.WIPE, location.lower(), "Wipe Object"))
        print(f"Wiping {location}")
    
    def pick(self, object_name):
        self.action_queue.actions.append(build_action_request(ManipulationPlanRequest.PICK, object_name.lower(), "Pick Object"))
        print(f"Picked {object_name}")

    def place(self, location):
        self.action_queue.actions.append(build_action_request(ManipulationPlanRequest.PLACE, location.lower(), "Place Object"))
        print(f"Placed in {location}")

    def wave(self):
        self.action_queue.actions.append(build_action_request(ManipulationPlanRequest.WAVE, "", "Wave"))
        print(f"Waving")

    def done(self):
        if self.action_queue.actions == []:
            self.is_ready_for_command = True
            print(f"Rule Checked, no actions to perform")
            return
        # Create a goal to send to the action server
        if self.is_ready_for_command and not self.is_stopped and not self.paused:
            self.publish_active_rules()
            self.action_dispatcher_client.send_goal(self.action_queue, done_cb=self.action_dispatcher_done_callback)
            rospy.sleep(1)
            self.action_queue.actions = []
            self.is_ready_for_command = False
            print(f"Done")
        else:
            self.action_queue.actions = []
            
    ######################################### Save and Load #########################################
    def load_rules(self):
        loaded_rules = self.rule_storage_manager.load_rules()
        for rule in loaded_rules:
            self.add_rule(rule)

        print(f"Loaded {len(loaded_rules)} rules.")

    def save_rules(self):
        """Saves the current rules."""
        self.rule_storage_manager.save_rules(self.rules)  
