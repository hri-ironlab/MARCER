import openai


# Hyperparameters to initialize VLM
class Hyperparameters:
    def __init__(self, n=1, temperature=1, sampling_params=None):
        self.sampling_params = sampling_params or {
            "max_tokens": 1024,
            "temperature": temperature,
            "n": n,
            "logprobs": True,
            "presence_penalty": 0,
            "frequency_penalty": 0,
        }

test_prompt = [
    {
        "role": "system",
        "content": "You are a helpful assistant. You will be given a task and you have to identify the trigger for the task.",
    },
    {
        "role": "user",
        "content": [
            {
                "type": "text",
                "text": "Here are a few examples of tasks with their respective triggers -\n",
            },
            {"type": "text", "text": "Pour the water from the bottle into the glass"},
            {"type": "text", "text": "No trigger"},
            {"type": "text", "text": "Wave hi!"},
            {"type": "text", "text": "No trigger"},
            {"type": "text", "text": "Do a wave!"},
            {"type": "text", "text": "No trigger"},
            {"type": "text", "text": "Move the mustard to the table"},
            {"type": "text", "text": "No trigger"},
            {"type": "text", "text": "Move the cheezeit to the top shelf"},
            {"type": "text", "text": "No trigger"},
            {"type": "text", "text": "Move the pringles to the right table"},
            {"type": "text", "text": "No trigger"},
            {"type": "text", "text": "Move the cheezeit to the bottom shelf"},
            {"type": "text", "text": "No trigger"},
            {"type": "text", "text": "move the food to the left table"},
            {"type": "text", "text": "No trigger"},
            {"type": "text", "text": "move the snacks to the right table"},
            {"type": "text", "text": "No trigger"},
            {"type": "text", "text": "Pick the cup"},
            {"type": "text", "text": "No trigger"},
            {"type": "text", "text": "Place the cheezeit on the left table"},
            {"type": "text", "text": "No trigger"},
            {"type": "text", "text": "wipe the table"},
            {"type": "text", "text": "No trigger"},
            {"type": "text", "text": "Move the water bottle to the right table"},
            {"type": "text", "text": "No trigger"},
            {"type": "text", "text": "Move the mug to the left table"},
            {"type": "text", "text": "No trigger"},
            {
                "type": "text",
                "text": "Move the groceries from the table to the bottom shelf then wipe the left table",
            },
            {"type": "text", "text": "No trigger"},
            {
                "type": "text",
                "text": "When the glass is in zone 2, Pour the water from the bottle into the glass",
            },
            {"type": "text", "text": "Zone trigger, zone 2"},
            {
                "type": "text",
                "text": "Move the mustard to the table when it is on the shelf",
            },
            {"type": "text", "text": "Zone trigger, shelf"},
            {
                "type": "text",
                "text": "When the cheezeit is in zone 1, Move the cheezeit to the top shelf",
            },
            {"type": "text", "text": "Zone trigger, zone 1"},
            {
                "type": "text",
                "text": "When the pringles is on the left table, Move the pringles to the right table",
            },
            {"type": "text", "text": "Zone trigger, left table"},
            {
                "type": "text",
                "text": "Move the cheezeit to the bottom shelf if it is placed in zone 1",
            },
            {"type": "text", "text": "Zone trigger, zone 1"},
            {
                "type": "text",
                "text": "move the food to the left table, if there is food in zone 3",
            },
            {"type": "text", "text": "Zone trigger, zone 3"},
            {
                "type": "text",
                "text": "When I put snacks on the left table, move the snacks to the right table",
            },
            {"type": "text", "text": "Zone trigger, left table"},
            {"type": "text", "text": "If I put the cup in zone 2, Pick the cup"},
            {"type": "text", "text": "Zone trigger, zone 2"},
            {
                "type": "text",
                "text": "Place the cheezeit on the left table, if I put it in zone 4",
            },
            {"type": "text", "text": "Zone trigger, left table"},
            {
                "type": "text",
                "text": "When the water bottle is on the left table, Move the water bottle to the right table",
            },
            {"type": "text", "text": "Zone trigger, left table"},
            {
                "type": "text",
                "text": "Move the mug to the left table, when it is on the right table",
            },
            {"type": "text", "text": "Zone trigger, right table"},
            {
                "type": "text",
                "text": "When I put groceries on the left table, Move the groceries to the bottom shelf then wipe the left table",
            },
            {"type": "text", "text": "Zone trigger, table"},
            {
                "type": "text",
                "text": "After 2 minutes, Pour the water from the bottle into the glass",
            },
            {"type": "text", "text": "Time trigger, 2 minutes"},
            {"type": "text", "text": "After 2 minutes, Wave hi!"},
            {"type": "text", "text": "Time trigger, 2 minutes"},
            {"type": "text", "text": "Do a wave after 10 minutes!"},
            {"type": "text", "text": "Time trigger, 10 minutes"},
            {"type": "text", "text": "In 1 minute, Move the mustard to the table"},
            {"type": "text", "text": "Time trigger, 1 minutes"},
            {
                "type": "text",
                "text": "Move the cheezeit to the top shelf after 5 minutes",
            },
            {"type": "text", "text": "Time trigger, 5 minutes"},
            {
                "type": "text",
                "text": "After 5 minutes, Move the pringles to the right table",
            },
            {"type": "text", "text": "Time trigger, 5 minutes"},
            {
                "type": "text",
                "text": "Move the cheezeit to the bottom shelf, in half a minute",
            },
            {"type": "text", "text": "Time trigger, 0.5 minutes"},
            {
                "type": "text",
                "text": "After 3 minutes, move the food to the left table",
            },
            {"type": "text", "text": "Time trigger, 3 minutes"},
            {
                "type": "text",
                "text": "In ten minutes, move the snacks to the right table",
            },
            {"type": "text", "text": "Time trigger, 10 minutes"},
            {"type": "text", "text": "Pick the cup, after 5 minutes have passed."},
            {"type": "text", "text": "Time trigger, 5 minutes"},
            {
                "type": "text",
                "text": "Place the cheezeit on the left table after one minute has passed",
            },
            {"type": "text", "text": "Time trigger, 1 minutes"},
            {"type": "text", "text": "In 15 minutes wipe the table"},
            {"type": "text", "text": "Time trigger, 15 minutes"},
            {
                "type": "text",
                "text": "After four minutes Move the water bottle to the right table",
            },
            {"type": "text", "text": "Time trigger, 4 minutes"},
            {"type": "text", "text": "Move the mug to the left table after 3 minutes"},
            {"type": "text", "text": "Time trigger, 3 minutes"},
            {
                "type": "text",
                "text": "After three minutes have passed Move the groceries from the table to the bottom shelf then wipe the left table",
            },
            {"type": "text", "text": "Time trigger, 3 minutes"},
            {
                "type": "text",
                "text": "These are a few things to consider - \n 1. If a task requires waiting for some time, then it has a time trigger. Answer with 'time trigger <time>'. Where the <time> token should be replaced by a measure of time in minutes.\n 2. If a task involves objects being in a specific location before execution, then it has a zone trigger. Answer with 'zone trigger <zone>'. Where the <zone> token should be replaced by a surface.\n 3. If a task has neither of the triggers then it does not have a trigger. Answer with 'no trigger'.",
            },
            {
                "type": "text",
                "text": "These are the objects present near you - ['Mustard', 'Pringles', 'Cheezeit', 'Bottle']\n These are the locations present near you - ['table', 'dish area']\n",
            },
            {
                "type": "text",
                "text": "Given the following task what trigger does it have?\n",
            },
            {
                "type": "text",
                "text": "When I place the groceries on a table, Put the groceries in zone 2",
            },
        ],
    },
]