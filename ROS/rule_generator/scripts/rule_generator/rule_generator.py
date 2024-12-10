#!/usr/bin/env python3
from rule_generator.msg import UserApprovalResponse, UserFeedback
from rule_monitor.srv import UpdateRuleRequest, UpdateRuleResponse
from rule_generator.msg import RuleGeneratorState
from rule_generator.context_builder import Context, ContextBuilder
from rule_generator.prompt_builder import PromptBuilder
from rule_monitor.msg import TriggerType, Rule
from rule_generator.srv import GetUserApproval, GetUserApprovalRequest
class RuleGenerator:
    def __init__(self):
        self.display_current_command_publisher = None
        self.rule_generator_state_publisher = None
        self.update_rule_service = None
        self.language_model_service = None
        self.get_user_approval_client = None

        # Dictionary to map states to their corresponding handler methods
        self.state_handlers = {
            RuleGeneratorState.IDLE: self.idle_state,
            RuleGeneratorState.GENERATE_RULE: self.generate_rule_state,
            RuleGeneratorState.WAIT_FOR_APPROVAL: self.wait_for_approval_state,
            RuleGeneratorState.WAIT_FOR_FEEDBACK: self.wait_for_feedback_state,
            RuleGeneratorState.ADD_RULE: self.add_rule_state,
            RuleGeneratorState.DONE: self.done_state,
        }

        self.state = RuleGeneratorState.IDLE

        self.context = Context()
        self.context_builder = ContextBuilder()
        self.prompt_builder = PromptBuilder()
        self.context.action_list = [
                "self.self.pick(<object>)", 
                "self.self.place(<location>)", 
                "self.self.pour(<object>)", 
                "self.self.wipe(<location>)", 
                "self.self.wave()",
                "self.done()",
                "self.self.done()",
                "self.check_condition(<condition>)"
            ]
        
        self.speech_command = ""
        # self.speech_command = "After 4 minutes have passed, Put the groceries on the dish area"
        # self.speech_command = ""
        # # self.speech_command = "When I place the groceries on a table, Put the groceries on the dish area"
        # self.speech_command = "pick up then place the pringles back on the table"
        self.current_task = ""
        self.generated_trigger_type = TriggerType.TIMER
        self.generated_function_name = ""
        self.generated_function = ""
        self.generated_function_description = ""
        self.rule_feedback = ""

    def handle_model_states(self, msg):
        self.model_states = msg
        
    def handle_speech_input(self, msg):
        if self.state == RuleGeneratorState.IDLE or self.state == RuleGeneratorState.WAIT_FOR_FEEDBACK:
            self.speech_command = msg.data
    
    def run_state_machine(self):
        handler = self.state_handlers.get(self.state)
        self.rule_generator_state_publisher.publish(self.state)
        if handler:
            handler()
        else:
            print(f"Unknown state: {self.state}")

    def get_trigger_type(self, response):
        if "zone and time" in response.lower():
            return TriggerType.ZONE
        elif "zone" in response.lower():
            return TriggerType.ZONE
        elif "time" in response.lower():
            return TriggerType.TIMER
        else:
            return TriggerType.NONE
    
    def parse_function_response(self, response):
        """
        parse the output of the language model to get the function
        input: output of the language model
        output: function
        """
        function = response.split("```python")[1].split("```")[0]
        function_name =  function.split()[1].split('(')[0]   
        return function_name, function

    def idle_state(self):
        if self.speech_command:
            # Send feedback to user
            self.current_task = self.speech_command
            self.speech_command = ""
            self.state = RuleGeneratorState.GENERATE_RULE
        return 
    
    def generate_rule_state(self):
        current_commands = UserFeedback()
        current_commands.speech_command = self.current_task
        current_commands.speech_command_feedback = self.rule_feedback

        self.display_current_command_publisher.publish(current_commands)
        print("Planning task: ", self.current_task)

        context = self.context_builder.get_context()
        context.task = self.current_task
        context.task_feedback = self.rule_feedback

        # Prompt for trigger generation
        trigger_prompt = self.prompt_builder.get_trigger_prompt(context)
        response = self.language_model_service.call(trigger_prompt)
        self.generated_trigger_type = self.get_trigger_type(response.response)
        # print(self.generated_trigger_type)
        # print("-----------------")

        # Prompt for function generation
        function_prompt = self.prompt_builder.get_function_prompt(context, response.response)
        response = self.language_model_service.call(function_prompt)
        self.generated_function_name, self.generated_function = self.parse_function_response(response.response)
        print(self.generated_function)
        print("-----------------")

        description_prompt = self.prompt_builder.get_function_description_prompt(self.generated_function)
        response = self.language_model_service.call(description_prompt)
        self.generated_function_description = response.response

        self.state = RuleGeneratorState.WAIT_FOR_APPROVAL
        # self.state = RuleGeneratorState.ADD_RULE
        return 
    
    def wait_for_approval_state(self):
        req = GetUserApprovalRequest()
        req.plan_description = self.generated_function_description
        req.generated_function = self.generated_function
        response = self.get_user_approval_client(req)
        if response.response.response == UserApprovalResponse.APPROVED:
            current_commands = UserFeedback()
            current_commands.speech_command = self.current_task
            current_commands.speech_command_feedback = self.rule_feedback
            self.display_current_command_publisher.publish(current_commands)
            self.state = RuleGeneratorState.ADD_RULE
        elif response.response.response == UserApprovalResponse.FEEDBACK:
            self.state = RuleGeneratorState.WAIT_FOR_FEEDBACK
        elif response.response.response == UserApprovalResponse.DELETE:
            self.state = RuleGeneratorState.DONE
        return 
    
    def wait_for_feedback_state(self):
        if(self.speech_command != ""):
            self.rule_feedback += " " + self.speech_command
            self.speech_command = ""
            self.state = RuleGeneratorState.GENERATE_RULE
        return 
    
    def add_rule_state(self):
        # Send rule to rule monitor
        req = UpdateRuleRequest()

        req.rule.name = self.generated_function_name
        req.rule.function = self.generated_function
        req.rule.description = self.generated_function_description
        req.rule.type.type = self.generated_trigger_type
        req.rule.operation = Rule.ADD
        req.rule.is_active = True
        self.update_rule_service.call(req)
        self.state = RuleGeneratorState.DONE

    def done_state(self):
        self.speech_command = ""
        self.current_task = ""
        self.generated_trigger_type = ""
        self.generated_function = ""
        self.generated_function_description = ""
        self.rule_feedback = ""
        self.state = RuleGeneratorState.IDLE
        return