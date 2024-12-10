#!/usr/bin/env python3
from ipdb import set_trace
from language_model.msg import ContentItem, ContentItems, Prompt
from language_model.srv import (
    QueryLanguageModel,
    QueryLanguageModelRequest,
    QueryLanguageModelResponse,
)
import rule_generator.dataset


class PromptBuilder:
    def __init__(self):
        self.examples = rule_generator.dataset.get_examples()

    def get_trigger_prompt(self, context):
        system_prompt = "You are a helpful assistant. You will be given a task and you have to identify the trigger for the task."

        # Prepare the content prompts
        secondary_content_prompts = [
            "Here are a few examples of tasks with their respective triggers -\n",
            ("These are a few things to consider - \n "
            "1. If a task requires waiting for some time, then it has a time trigger. Answer with 'time trigger <time>'. Where the <time> token should be replaced by a measure of time in minutes.\n "
            "2. If a task involves objects being in a specific location before execution, then it has a zone trigger. Answer with 'zone trigger <zone>'. Where the <zone> token should be replaced by a surface.\n "
            "3. If a task involves a 'zone trigger' amd a 'time trigger'. Answer with 'zone and time trigger, <zone>, <time>'. Where the <zone> token should be replaced by a surface and <time> token should be replaced by a measure of time in minutes.\n "
            "4. If a task has neither of the triggers then it does not have a trigger. Answer with 'no trigger'."),
            f"These are the objects present near you - {context.object_names}\n These are the locations present near you - {context.locations}\n",
            "Given the following task what trigger does it have?\n",
            context.task
        ]

        secondary_content = ContentItems()

        if len(self.examples) > 0:
            secondary_content.content_items.append(ContentItem(type="text", text=secondary_content_prompts[0]))

        for example in self.examples:
            secondary_content.content_items.append(ContentItem(type="text", text=example["tasks"]))
            secondary_content.content_items.append(ContentItem(type="text", text=example["triggers"]))

        for text in secondary_content_prompts[1:]:
            secondary_content.content_items.append(ContentItem(type="text", text=text))

        # Construct the query
        query = QueryLanguageModelRequest()
        query.prompt = Prompt()
        query.prompt.primary_role = "system"
        query.prompt.primary_role_content = system_prompt
        query.prompt.secondary_role = "user"
        query.prompt.secondary_role_content = secondary_content

        return query

    def get_action_plan_prompt(self, context):
        primary_content = (
            "You are an action plan generator. You will be given a task to perform and you have to answer with a sequence of actions to complete the task. You can use actions from the action_list to generate the action plan.\n"
            f"action_list = {context.actions}\n"
            f"The <object> token should be replaced by an object in object_list that the action refers to. You see the objects from object_list in the same room as you.\n"
            f"object_list = {context.object_names}\n"
            f"The <location> token should be replaced by a location in location_list that the action refers to. You see the location from location_list in the same room as you.\n"
            f"location_list = {context.locations}\n"
            "The <condition> token should be replaced by a condition to check."
        )

        # Prepare the content prompts
        secondary_content_prompts = [
            "Here are a few examples of tasks with their respective action plans -\n",
            ("These are a few constraints to consider - \n"
            "1. If an object is picked up, it needs to be put down before picking another object.\n"
            "2. An object needs to be picked up before it can be placed on anything.\n"
            "3. An object needs to be picked up before it can be used.\n"
            "4. If an object is picked up, it needs to be put down before waving."),
            "Given the following task what is the action plan for it?\n",
            f"Here is some additional information: {context.task_feedback}\n",
            context.task,
        ]

        secondary_content = ContentItems()

        if len(self.examples) > 0:
            secondary_content.content_items.append(ContentItem(type="text", text=secondary_content_prompts[0]))
        
        for example in self.examples:
            secondary_content.content_items.append(ContentItem(type="text", text=example["tasks"]))
            secondary_content.content_items.append(f"object_list = {example['object_names']}")
            secondary_content.content_items.append(f"location_list = {example['locations']}")
            secondary_content.content_items.append(f"object_locations = {example['object_locations']}")
            eg_ap = "\n".join([f"Step {i+1}: {act}" for i, act in enumerate(example["action_plans"])])
            secondary_content.content_items.append("Action plan =\n" + eg_ap)

        secondary_content.content_items.append(ContentItem(type="text", text=secondary_content_prompts[1]))
        secondary_content.content_items.append(ContentItem(type="text", text=secondary_content_prompts[2]))
        secondary_content.content_items.append(ContentItem(type="text", text=context.task))
        if context.task_feedback != "":
            secondary_content.content_items.append(ContentItem(type="text", text=secondary_content_prompts[3]))
            
        secondary_content.content_items.append(ContentItem(type="text", text=f"object_list = {context.object_names}"))
        secondary_content.content_items.append(ContentItem(type="text", text=f"location_list = {context.locations}"))
        secondary_content.content_items.append(ContentItem(type="text", text=f"{context.object_locations}"))
        secondary_content.content_items.append(ContentItem(type="text", text="Action plan =\n"))

        # Construct the query
        query = QueryLanguageModelRequest()
        query.prompt = Prompt()
        query.prompt.primary_role = "system"
        query.prompt.primary_role_content = primary_content
        query.prompt.secondary_role = "user"
        query.prompt.secondary_role_content = secondary_content
        return query

    def get_function_prompt(self, context, trigger):
        if "zone and time" in trigger.lower():
            filtered_examples = [
                example
                for example in self.examples
                if "zone and time" in example["triggers"].lower()
            ]
        elif "zone" in trigger.lower():
            filtered_examples = [
                example
                for example in self.examples
                if "zone" in example["triggers"].lower()
            ]
        elif "time" in trigger.lower():
            filtered_examples = [
                example
                for example in self.examples
                if "time" in example["triggers"].lower()
            ]
        else:
            filtered_examples = [
                example
                for example in self.examples
                if "no" in example["triggers"].lower()
            ]

        primary_content = (
            "You are an action plan generator. You will be given a task to perform and you have to "
            "answer with a python function to complete the task. Your function can internally call the "
            "python functions from the function_list to generate the python function.\n"
            f"function_list = {context.actions}\n"
            f"The <object> token should be replaced by an object in object_list that the action refers to. "
            "You see the objects from object_list in the same room as you.\n"
            f"object_list = {context.object_names}\n"
            f"The <surface> token should be replaced by a surface in location_list that the action refers to. "
            "You see the location from location_list in the same room as you.\n"
            f"location_list = {context.locations}\n"
        )

        # Prepare the content prompts
        secondary_content_prompts = [
            "Here are a few examples of tasks with their respective python functions -\n",
            ("These are a few constraints to consider - \n "
             "1. If an object is picked up, it needs to be put down before picking another object.\n "
             "2. An object needs to be picked up before it can be placed on anything.\n "
             "3. An object needs to be picked up before it can be used.\n "
             "4. If an object is picked up, it needs to be put down before waving."),
            "Given the following task what is the python function for it?\n",
            f"Here is some additional information: {context.task_feedback}\n",
        ]

        secondary_content = ContentItems()

        if len(filtered_examples) > 0:
            secondary_content.content_items.append(ContentItem(type="text", text=secondary_content_prompts[0]))

        for example in filtered_examples:
            secondary_content.content_items.append(ContentItem(type="text", text=example["tasks"]))
            secondary_content.content_items.append(ContentItem(type="text", text=example["triggers"]))
            secondary_content.content_items.append(ContentItem(type="text", text=f"object_list = {example['object_names']}"))
            secondary_content.content_items.append(ContentItem(type="text", text=f"location_list = {example['locations']}"))
            secondary_content.content_items.append(ContentItem(type="text", text=f"object_location_list = {example['object_locations']}"))
            secondary_content.content_items.append(ContentItem(type="text", text=f"Python function =\n{example['functions']}"))

        secondary_content.content_items.append(ContentItem(type="text", text=secondary_content_prompts[1]))
        secondary_content.content_items.append(ContentItem(type="text", text=secondary_content_prompts[2]))
        secondary_content.content_items.append(ContentItem(type="text", text=context.task))
        if context.task_feedback != "":
            secondary_content.content_items.append(ContentItem(type="text", text=secondary_content_prompts[3]))

        secondary_content.content_items.append(ContentItem(type="text", text=trigger))
        secondary_content.content_items.append(ContentItem(type="text", text=f"object_list = {context.object_names}"))
        secondary_content.content_items.append(ContentItem(type="text", text=f"location_list = {context.locations}"))
        secondary_content.content_items.append(ContentItem(type="text", text=f"object_location_list = {context.object_locations}"))
        secondary_content.content_items.append(ContentItem(type="text", text="Python function =\n"))

        # Construct the query
        query = QueryLanguageModelRequest()
        query.prompt = Prompt()
        query.prompt.primary_role = "system"
        query.prompt.primary_role_content = primary_content
        query.prompt.secondary_role = "user"
        query.prompt.secondary_role_content = secondary_content

        return query

    def get_function_description_prompt(self, function):
        primary_content = "You will be given a python function. You have to respond with a summary of the function."

        # Prepare the content prompts
        secondary_content_prompts = [
            "Summarize the following Python function in simple but specific terms for a non-programmer. Focus only on what it will do when it runs, using no more than five clear and concise sentences.",
            function
        ]

        # Construct the content prompt
        secondary_content = ContentItems()
        for text in secondary_content_prompts:
            secondary_content.content_items.append(ContentItem(type="text", text=text))

        # Construct the query
        query = QueryLanguageModelRequest()
        query.prompt = Prompt()
        query.prompt.primary_role = "system"
        query.prompt.primary_role_content = primary_content
        query.prompt.secondary_role = "user"
        query.prompt.secondary_role_content = secondary_content

        return query