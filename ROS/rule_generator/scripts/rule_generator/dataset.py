#!/usr/bin/env python3
def get_examples():
    examples = []
    for eg_id in example_tasks:
        example = {'tasks': example_tasks[eg_id],
                   'triggers': example_triggers[eg_id],
                   'object_names': example_object_names[eg_id],
                   'locations': example_locations[eg_id],
                   'object_locations': example_object_locations[eg_id],
                   'action_plans': example_action_plans[eg_id],
                   'functions': example_functions[eg_id]}
        examples.append(example)
    return examples

example_tasks = {1: "Pour the water from the bottle into the glass",
                 2: "Wave hi!",
                 3: "Do a wave!",
                 4: "Move the mustard to the table",
                 5: "Move the cheezit to the top shelf",
                 6: "Move the pringles to the right table",
                 7: "Move the cheezit to the bottom shelf",
                 8: "move the food to the left table",
                 9: "move the snacks to the right table",
                 10: "Pick the cup",
                 11: "Place the cheezit on the left table",
                 12: "wipe the table",
                 13: "Move the water bottle to the right table",
                 14: "Move the mug to the left table",
                 15: "Move the groceries from the table to the bottom shelf then wipe the left table",

                 16: "When the glass is in zone 2, Pour the water from the bottle into the glass",
                 17: "Move the mustard to the table when it is on the shelf",
                 18: "When the cheezit is in zone 1, Move the cheezit to the top shelf",
                 19: "When the pringles is on the left table, Move the pringles to the right table",
                 20: "Move the cheezit to the bottom shelf if it is placed in zone 1",
                 21: "move the food to the left table, if there is food in zone 3",
                 22: "When I put snacks on the left table, move the snacks to the right table",
                 23: "If I put the cup in zone 2, Pick the cup",
                 24: "Place the cheezit on the left table, if I put it in zone 4",
                 25: "When the water bottle is on the left table, Move the water bottle to the right table",
                 26: "Move the mug to the left table, when it is on the right table",
                 27: "When I put groceries on the left table, Move the groceries to the bottom shelf then wipe the left table",

                 28: "After 2 minutes, Pour the water from the bottle into the glass",
                 29: "After 2 minutes, Wave hi!",
                 30: "Do a wave after 10 minutes!",
                 31: "In 1 minute, Move the mustard to the table",
                 32: "Move the cheezit to the top shelf after 5 minutes",
                 33: "After 5 minutes, Move the pringles to the right table",
                 34: "Move the cheezit to the bottom shelf, in half a minute",
                 35: "After 3 minutes, move the food to the left table",
                 36: "In ten minutes, move the snacks to the right table",
                 37: "Pick the cup, after 5 minutes have passed.",
                 38: "Place the cheezit on the left table after one minute has passed",
                 39: "In 15 minutes wipe the table",
                 40: "After four minutes Move the water bottle to the right table",
                 41: "Move the mug to the left table after 3 minutes",
                 42: "After three minutes have passed Move the groceries from the table to the bottom shelf then wipe the left table",

                 43: "When the glass is in zone 2, after 2 minutes, Pour the water from the bottle into the glass",
                 44: "When it is on the shelf for 1 minute, Move the mustard to the table",
                 45: "When the cheezit is in zone 1, move the cheezit to the top shelf after 5 minutes",
                 46: "When the pringles has been on the left table for 5 minutes, move the pringles to the right table",
                 47: "Move the cheezit to the bottom shelf, if it is placed in zone 1 for half a minute",
                 48: "If there is food in zone 3 after 3 minutes, move the food to the left table",
                 49: "When I put snacks on the left table wait for ten minutes, then move the snacks to the right table",
                 50: "If I put the cup in zone 2, pick the cup after 5 minutes have passed.",
                 51: "Place the cheezit on the left table if I put it in zone 4 and one minute has passed",
                 52: "When the water bottle is on the left table for four minutes move the water bottle to the right table",
                 53: "Move the mug to the left table when it is on the right table for 3 minutes",
                 54: "When I put groceries on the left table, wait for three minutes to pass, then move the groceries from the table to the bottom shelf then wipe the left table"
                 }

example_triggers = {1: "No trigger",
                    2: "No trigger",
                    3: "No trigger",
                    4: "No trigger",
                    5: "No trigger",
                    6: "No trigger",
                    7: "No trigger",
                    8: "No trigger",
                    9: "No trigger",
                    10: "No trigger",
                    11: "No trigger",
                    12: "No trigger",
                    13: "No trigger",
                    14: "No trigger",
                    15: "No trigger",

                    16: "Zone trigger, zone 2",
                    17: "Zone trigger, shelf",
                    18: "Zone trigger, zone 1",
                    19: "Zone trigger, left table",
                    20: "Zone trigger, zone 1",
                    21: "Zone trigger, zone 3",
                    22: "Zone trigger, left table",
                    23: "Zone trigger, zone 2",
                    24: "Zone trigger, left table",
                    25: "Zone trigger, left table",
                    26: "Zone trigger, right table",
                    27: "Zone trigger, table",

                    28: "Time trigger, 2 minutes",
                    29: "Time trigger, 2 minutes",
                    30: "Time trigger, 10 minutes",
                    31: "Time trigger, 1 minutes",
                    32: "Time trigger, 5 minutes",
                    33: "Time trigger, 5 minutes",
                    34: "Time trigger, 0.5 minutes",
                    35: "Time trigger, 3 minutes",
                    36: "Time trigger, 10 minutes",
                    37: "Time trigger, 5 minutes",
                    38: "Time trigger, 1 minutes",
                    39: "Time trigger, 15 minutes",
                    40: "Time trigger, 4 minutes",
                    41: "Time trigger, 3 minutes",
                    42: "Time trigger, 3 minutes",

                    43: "Zone and time trigger, zone 2, 2 minutes",
                    44: "Zone and time trigger, shelf, 1 minutes",
                    45: "Zone and time trigger, zone 1, 5 minutes",
                    46: "Zone and time trigger, left table, 5 minutes",
                    47: "Zone and time trigger, zone 1, 0.5 minutes",
                    48: "Zone and time trigger, zone 3, 3 minutes",
                    49: "Zone and time trigger, left table, 10 minutes",
                    50: "Zone and time trigger, zone 2, 5 minutes",
                    51: "Zone and time trigger, left table, 1 minutes",
                    52: "Zone and time trigger, left table, 4 minutes",
                    53: "Zone and time trigger, right table, 3 minutes",
                    54: "Zone and time trigger, table, 3 minutes"
                    }

example_object_names = {1: ["glass", "pringles", "mustard", "bottle", "cheezit"],
                             2: ["glass", "pringles", "mustard", "bottle", "cheezit"],
                             3: ["glass", "pringles", "mustard", "bottle", "cheezit"],
                             4: ["glass", "pringles", "mustard", "bottle", "cheezit"],
                             5: ["glass", "pringles", "mustard", "bottle", "cheezit"],
                             6: ["glass", "pringles", "mustard", "bottle", "cheezit"],
                             7: ["glass", "pringles", "mustard", "bottle", "cheezit"],
                             8: ["glass", "pringles", "mustard", "cheezit"],
                             9: ["glass", "pringles", "mustard", "cheezit"],
                             10: ["pringles", "mustard", "glass", "bottle"],
                             11: ["pringles", "mustard", "bottle", "cheezit"],
                             12: ["glass", "pringles", "mustard", "bottle", "sponge"],
                             13: ["glass", "pringles", "sponge", "bottle", "cheezit"],
                             14: ["cup", "pringles", "cheezit"],
                             15: ["cup", "pringles", "cheezit", "sponge"],

                             16: ["glass", "pringles", "mustard", "bottle", "cheezit"],
                             17: ["glass", "pringles", "mustard", "bottle", "cheezit"],
                             18: ["glass", "pringles", "mustard", "bottle", "cheezit"],
                             19: ["glass", "pringles", "mustard", "bottle", "cheezit"],
                             20: ["glass", "pringles", "mustard", "bottle", "cheezit"],
                             21: ["glass", "pringles", "mustard", "cheezit"],
                             22: ["glass", "pringles", "mustard", "cheezit"],
                             23: ["pringles", "mustard", "glass", "bottle"],
                             24: ["pringles", "mustard", "bottle", "cheezit"],
                             25: ["glass", "pringles", "sponge", "bottle", "cheezit"],
                             26: ["cup", "pringles", "cheezit"],
                             27: ["cup", "pringles", "cheezit", "sponge"],

                             28: ["glass", "pringles", "mustard", "bottle", "cheezit"],
                             29: ["glass", "pringles", "mustard", "bottle", "cheezit"],
                             30: ["glass", "pringles", "mustard", "bottle", "cheezit"],
                             31: ["glass", "pringles", "mustard", "bottle", "cheezit"],
                             32: ["glass", "pringles", "mustard", "bottle", "cheezit"],
                             33: ["glass", "pringles", "mustard", "bottle", "cheezit"],
                             34: ["glass", "pringles", "mustard", "bottle", "cheezit"],
                             35: ["glass", "pringles", "mustard", "cheezit"],
                             36: ["glass", "pringles", "mustard", "cheezit"],
                             37: ["pringles", "mustard", "glass", "bottle"],
                             38: ["pringles", "mustard", "bottle", "cheezit"],
                             39: ["glass", "pringles", "mustard", "bottle", "sponge"],
                             40: ["glass", "pringles", "sponge", "bottle", "cheezit"],
                             41: ["cup", "pringles", "cheezit"],
                             42: ["cup", "pringles", "cheezit", "sponge"],

                             43: ["glass", "pringles", "mustard", "bottle", "cheezit"],
                             44: ["glass", "pringles", "mustard", "bottle", "cheezit"],
                             45: ["glass", "pringles", "mustard", "bottle", "cheezit"],
                             46: ["glass", "pringles", "mustard", "bottle", "cheezit"],
                             47: ["glass", "pringles", "mustard", "bottle", "cheezit"],
                             48: ["glass", "pringles", "mustard", "cheezit"],
                             49: ["glass", "pringles", "mustard", "cheezit"],
                             50: ["pringles", "mustard", "glass", "bottle"],
                             51: ["pringles", "mustard", "bottle", "cheezit"],
                             52: ["glass", "pringles", "sponge", "bottle", "cheezit"],
                             53: ["cup", "pringles", "cheezit"],
                             54: ["cup", "pringles", "cheezit", "sponge"]
                        }

example_locations = {1: ["table on the right", "table on the left"],
                         2: ["table on the right", "table on the left"],
                         3: ["table on the right", "table on the left"],
                         4: ["table on the right", "table on the left", "dish area"],
                         5: ["table on the right", "table on the left", "bottom shelf", "top shelf"],
                         6: ["bottom shelf", "top shelf", "table on the right", "table on the left"],
                         7: ["bottom shelf", "top shelf"],
                         8: ["table on the left", "table on the right"],
                         9: ["bottom shelf", "top shelf", "table on the left", "table on the right"],
                         10: ["dish area", "table"],
                         11: ["dish area", "table on the left"],
                         12: ["table on the left", "Zone 1"],
                         13: ["dish area", "table on the right", "table on the left"],
                         14: ["dish area", "table on the right", "table on the left"],
                         15: ["table on the right", "table on the left", "bottom shelf", "top shelf"],

                         16: ["table on the right", "table on the left", "zone 2"],
                         17: ["table on the right", "table on the left", "shelf"],
                         18: ["table on the right", "table on the left", "bottom shelf", "top shelf", "zone 1"],
                         19: ["bottom shelf", "top shelf", "table on the right", "table on the left"],
                         20: ["zone 1", "bottom shelf", "top shelf", "table on the left"],
                         21: ["table on the left", "zone 3", "table on the right"],
                         22: ["bottom shelf", "top shelf", "table on the left", "table on the right"],
                         23: ["dish area", "table", "zone 1", "zone 2"],
                         24: ["dish area", "table on the left", "zone 2", "zone 4"],
                         25: ["dish area", "table on the right", "table on the left"],
                         26: ["dish area", "table on the right", "table on the left"],
                         27: ["table on the right", "table on the left", "bottom shelf", "top shelf"],

                         28: ["table on the right", "table on the left"],
                         29: ["table on the right", "table on the left"],
                         30: ["table on the right", "table on the left"],
                         31: ["table on the right", "table on the left", "dish area"],
                         32: ["table on the right", "table on the left", "bottom shelf", "top shelf"],
                         33: ["bottom shelf", "top shelf", "table on the right", "table on the left"],
                         34: ["bottom shelf", "top shelf"],
                         35: ["table on the left", "table on the right"],
                         36: ["bottom shelf", "top shelf", "table on the left", "table on the right"],
                         37: ["dish area", "table"],
                         38: ["dish area", "table on the left"],
                         39: ["table on the left", "Zone 1"],
                         40: ["dish area", "table on the right", "table on the left"],
                         41: ["dish area", "table on the right", "table on the left"],
                         42: ["table on the right", "table on the left", "bottom shelf", "top shelf"],

                         43: ["table on the right", "table on the left", "zone 2"],
                         44: ["table on the right", "table on the left", "shelf"],
                         45: ["table on the right", "table on the left", "bottom shelf", "top shelf", "zone 1"],
                         46: ["bottom shelf", "top shelf", "table on the right", "table on the left"],
                         47: ["zone 1", "bottom shelf", "top shelf", "table on the left"],
                         48: ["table on the left", "zone 3", "table on the right"],
                         49: ["bottom shelf", "top shelf", "table on the left", "table on the right"],
                         50: ["dish area", "table", "zone 1", "zone 2"],
                         51: ["dish area", "table on the left", "zone 2", "zone 4"],
                         52: ["dish area", "table on the right", "table on the left"],
                         53: ["dish area", "table on the right", "table on the left"],
                         54: ["table on the right", "table on the left", "bottom shelf", "top shelf"]
                     }

example_object_locations = {
    1: ['glass is on the table on the right', 'pringles is on the table on the right',
        'mustard is on the table on the right', 'bottle is on the table on the right',
        'cheezit is on the table on the right'],
    2: ['glass is on the table on the right', 'pringles is on the table on the right',
        'mustard is on the table on the right', 'bottle is on the table on the right',
        'cheezit is on the table on the right'],
    3: ['glass is on the table on the right', 'pringles is on the table on the right',
        'mustard is on the table on the right', 'bottle is on the table on the right',
        'cheezit is on the table on the right'],
    4: ['glass is on the table on the right', 'pringles is on the table on the right', 'mustard is on the dish area',
        'bottle is on the table on the right', 'cheezit is on the table on the right'],
    5: ['glass is on the table on the right', 'pringles is on the table on the right',
        'mustard is on the table on the right', 'bottle is on the table on the right',
        'cheezit is on the table on the right'],
    6: ['glass is on the table on the left', 'pringles is on the table on the left',
        'mustard is on the table on the left', 'bottle is on the table on the left',
        'cheezit is on the table on the left'],
    7: ['glass is on the top shelf', 'pringles is on the top shelf', 'mustard is on the top shelf',
        'bottle is on the top shelf', 'cheezit is on the top shelf'],
    8: ['glass is on the table on the left', 'pringles is on the table on the right',
        'mustard is on the table on the right', 'cheezit is on the table on the right'],
    9: ['glass is on the table on the left', 'pringles is on the table on the left',
        'mustard is on the table on the left', 'cheezit is on the table on the left'],
    10: ['glass is on the table', 'pringles is on the table', 'mustard is on the table', 'cheezit is on the table'],
    11: ['bottle is on the table on the left', 'pringles is on the table on the left',
         'mustard is on the table on the left', 'cheezit is on the dish area'],
    12: ['bottle is on Zone 1', 'pringles is on Zone 1', 'glass is on Zone 1', 'mustard is on Zone 1',
         'sponge is on the table on the left'],
    13: ['bottle is on table on the left', 'pringles is on table on the left', 'glass is on table on the right',
         'cheezit is on table on the right', 'sponge is on the table on the left'],
    14: ['cup is on the dish area', 'pringles is on the table on the right', 'cheezit is on the table on the right'],
    15: ['cup is on the bottom shelf', 'pringles is on the table on the right', 'cheezit is on the table on the right',
         'sponge is on the table on the left'],

    16: ['glass is on zone 2', 'pringles is on the table on the right', 'mustard is on the table on the right',
         'bottle is on the table on the right', 'cheezit is on the table on the right'],
    17: ['glass is on the table on the right', 'pringles is on the table on the right', 'mustard is on the shelf',
         'bottle is on the table on the right', 'cheezit is on the table on the right'],
    18: ['glass is on the table on the right', 'pringles is on the table on the right',
         'mustard is on the table on the right', 'bottle is on the table on the right',
         'cheezit is on the table on the right', 'cheezit is on zone 1'],
    19: ['glass is on the table on the left', 'pringles is on the table on the left',
         'mustard is on the table on the left', 'bottle is on the table on the left',
         'cheezit is on the table on the left'],
    20: ['glass is on the top shelf', 'pringles is on the top shelf', 'mustard is on the top shelf',
         'bottle is on the top shelf', 'cheezit is on the top shelf', 'bottle is on zone 1', 'cheezit is on zone 1'],
    21: ['glass is on the table on the left', 'pringles is on the table on the right',
         'mustard is on the table on the right', 'cheezit is on the table on the right', 'mustard is on zone 3',
         'cheezit is on zone 3'],
    22: ['glass is on the table on the left', 'pringles is on the table on the left',
         'mustard is on the table on the left', 'cheezit is on the table on the left'],
    23: ['glass is on the table', 'glass is on zone 2', 'pringles is on the table', 'mustard is on the table',
         'cheezit is on the table'],
    24: ['bottle is on the table on the left', 'pringles is on the table on the left',
         'mustard is on the table on the left', 'cheezit is on zone 4'],
    25: ['bottle is on table on the left', 'pringles is on table on the left', 'glass is on table on the right',
         'cheezit is on table on the right', 'sponge is on the table on the left'],
    26: ['pringles is on table on the left', 'cup is on table on the right', 'cheezit is on table on the right'],
    27: ['cup is on the dish area', 'pringles is on the table on the left', 'cheezit is on the table on the left',
         'sponge is on the table on the left'],

    28: ['glass is on the table on the right', 'pringles is on the table on the right',
         'mustard is on the table on the right', 'bottle is on the table on the right',
         'cheezit is on the table on the right'],
    29: ['glass is on the table on the right', 'pringles is on the table on the right',
         'mustard is on the table on the right', 'bottle is on the table on the right',
         'cheezit is on the table on the right'],
    30: ['glass is on the table on the right', 'pringles is on the table on the right',
         'mustard is on the table on the right', 'bottle is on the table on the right',
         'cheezit is on the table on the right'],
    31: ['glass is on the table on the right', 'pringles is on the table on the right', 'mustard is on the dish area',
         'bottle is on the table on the right', 'cheezit is on the table on the right'],
    32: ['glass is on the table on the right', 'pringles is on the table on the right',
         'mustard is on the table on the right', 'bottle is on the table on the right',
         'cheezit is on the table on the right'],
    33: ['glass is on the table on the left', 'pringles is on the table on the left',
         'mustard is on the table on the left', 'bottle is on the table on the left',
         'cheezit is on the table on the left'],
    34: ['glass is on the top shelf', 'pringles is on the top shelf', 'mustard is on the top shelf',
         'bottle is on the top shelf', 'cheezit is on the top shelf'],
    35: ['glass is on the table on the left', 'pringles is on the table on the right',
         'mustard is on the table on the right', 'cheezit is on the table on the right'],
    36: ['glass is on the table on the left', 'pringles is on the table on the left',
         'mustard is on the table on the left', 'cheezit is on the table on the left'],
    37: ['glass is on the table', 'pringles is on the table', 'mustard is on the table', 'cheezit is on the table'],
    38: ['bottle is on the table on the left', 'pringles is on the table on the left',
         'mustard is on the table on the left', 'cheezit is on the dish area'],
    39: ['bottle is on Zone 1', 'pringles is on Zone 1', 'glass is on Zone 1', 'mustard is on Zone 1',
         'sponge is on the table on the left'],
    40: ['bottle is on table on the left', 'pringles is on table on the left', 'glass is on table on the right',
         'cheezit is on table on the right', 'sponge is on the table on the left'],
    41: ['cup is on the dish area', 'pringles is on the table on the right', 'cheezit is on the table on the right'],
    42: ['cup is on the bottom shelf', 'pringles is on the table on the right', 'cheezit is on the table on the right',
         'sponge is on the table on the left'],

    43: ['glass is on zone 2', 'pringles is on the table on the right', 'mustard is on the table on the right',
         'bottle is on the table on the right', 'cheezit is on the table on the right'],
    44: ['glass is on the table on the right', 'pringles is on the table on the right', 'mustard is on the shelf',
         'bottle is on the table on the right', 'cheezit is on the table on the right'],
    45: ['glass is on the table on the right', 'pringles is on the table on the right',
         'mustard is on the table on the right', 'bottle is on the table on the right',
         'cheezit is on the table on the right', 'cheezit is on zone 1'],
    46: ['glass is on the table on the left', 'pringles is on the table on the left',
         'mustard is on the table on the left', 'bottle is on the table on the left',
         'cheezit is on the table on the left'],
    47: ['glass is on the top shelf', 'pringles is on the top shelf', 'mustard is on the top shelf',
         'bottle is on the top shelf', 'cheezit is on the top shelf', 'bottle is on zone 1', 'cheezit is on zone 1'],
    48: ['glass is on the table on the left', 'pringles is on the table on the right',
         'mustard is on the table on the right', 'cheezit is on the table on the right', 'mustard is on zone 3',
         'cheezit is on zone 3'],
    49: ['glass is on the table on the left', 'pringles is on the table on the left',
         'mustard is on the table on the left', 'cheezit is on the table on the left'],
    50: ['glass is on the table', 'glass is on zone 2', 'pringles is on the table', 'mustard is on the table',
         'cheezit is on the table'],
    51: ['bottle is on the table on the left', 'pringles is on the table on the left',
         'mustard is on the table on the left', 'cheezit is on zone 4'],
    52: ['bottle is on table on the left', 'pringles is on table on the left', 'glass is on table on the right',
         'cheezit is on table on the right', 'sponge is on the table on the left'],
    53: ['pringles is on table on the left', 'cup is on table on the right', 'cheezit is on table on the right'],
    54: ['cup is on the dish area', 'pringles is on the table on the left', 'cheezit is on the table on the left',
         'sponge is on the table on the left']
}

example_action_plans = {1: ["pick bottle", "pour in glass", "place on table on the right", "done"],
               2: ["wave at me", "done"],
               3: ["wave at me", "done"],
               4: ["pick mustard", "place on table on the right", "done"],
               5: ["pick cheezit", "place on top shelf", "done"],
               6: ["pick pringles", "place on table on the right", "done"],
               7: ["pick cheezit", "place on bottom shelf", "done"],
               8: ["pick pringles", "place on table on the left", "pick mustard", "place on table on the left",
                   "pick cheezit", "place on table on the left", "done"],
               9: ["pick pringles", "place on table on the right", "pick cheezit", "place on table on the right",
                   "done"],
               10: ["pick glass", "done"],
               11: ["pick cheezit", "place on table on the left", "done"],
               12: ["pick sponge", "wipe table on the left", "done"],
               13: ["pick bottle", "place on table on the right", "done"],
               14: ["pick cup", "place on table on the left", "done"],
               15: ["pick pringles", "place on bottom shelf", "pick cheezit", "place on bottom shelf", "pick sponge",
                    "wipe table on the left", "done"],

               16: ["pick bottle", "pour in glass", "place on table on the right", "done"],
               17: ["pick mustard", "place on table on the right", "done"],
               18: ["pick cheezit", "place on top shelf", "done"],
               19: ["pick pringles", "place on table on the right", "done"],
               20: ["pick cheezit", "place on bottom shelf", "done"],
               21: ["pick mustard", "place on table on the left", "pick cheezit", "place on table on the left",
                    "done"],
               22: ["pick pringles", "place on table on the right", "pick cheezit", "place on table on the right",
                    "done"],
               23: ["pick glass", "done"],
               24: ["pick cheezit", "place on table on the left", "done"],
               25: ["pick bottle", "place on table on the right", "done"],
               26: ["pick cup", "place on table on the left", "done"],
               27: ["pick pringles", "place on bottom shelf", "pick cheezit", "place on bottom shelf", "pick sponge",
                    "wipe table on the left", "done"],

               28: ["pick bottle", "pour in glass", "place on table on the right", "done"],
               29: ["wave at me", "done"],
               30: ["wave at me", "done"],
               31: ["pick mustard", "place on table on the right", "done"],
               32: ["pick cheezit", "place on top shelf", "done"],
               33: ["pick pringles", "place on table on the right", "done"],
               34: ["pick cheezit", "place on bottom shelf", "done"],
               35: ["pick pringles", "place on table on the left", "pick mustard", "place on table on the left",
                    "pick cheezit", "place on table on the left", "done"],
               36: ["pick pringles", "place on table on the right", "pick cheezit", "place on table on the right",
                    "done"],
               37: ["pick glass", "done"],
               38: ["pick cheezit", "place on table on the left", "done"],
               39: ["pick sponge", "wipe table on the left", "done"],
               40: ["pick bottle", "place on table on the right", "done"],
               41: ["pick cup", "place on table on the left", "done"],
               42: ["pick pringles", "place on bottom shelf", "pick cheezit", "place on bottom shelf", "pick sponge",
                    "wipe table on the left", "done"],

               43: ["pick bottle", "pour in glass", "place on table on the right", "done"],
               44: ["pick mustard", "place on table on the right", "done"],
               45: ["pick cheezit", "place on top shelf", "done"],
               46: ["pick pringles", "place on table on the right", "done"],
               47: ["pick cheezit", "place on bottom shelf", "done"],
               48: ["pick mustard", "place on table on the left", "pick cheezit", "place on table on the left",
                    "done"],
               49: ["pick pringles", "place on table on the right", "pick cheezit", "place on table on the right",
                    "done"],
               50: ["pick glass", "done"],
               51: ["pick cheezit", "place on table on the left", "done"],
               52: ["pick bottle", "place on table on the right", "done"],
               53: ["pick cup", "place on table on the left", "done"],
               54: ["pick pringles", "place on bottom shelf", "pick cheezit", "place on bottom shelf", "pick sponge",
                    "wipe table on the left", "done"]}

example_functions = {1: ("```python\n"
                         "def pour_water_to_glass(self):\n"
                         "\tself.pick('bottle')\n"
                         "\tself.pour('glass')\n"
                         "\tself.place('table on the right')\n"
                         "\tself.done()\n"
                         "\treturn\n"
                         "```\n"),
                     2: ("```python\n"
                         "def do_wave(self):\n"
                         "\tself.wave()\n"
                         "\tself.done()\n"
                         "\treturn\n"
                         "```\n"),
                     3: ("```python\n"
                         "def do_wave(self):\n"
                         "\tself.wave()\n"
                         "\tself.done()\n"
                         "\treturn\n"
                         "```\n"),
                     4: ("```python\n"
                         "def mustard_to_table(self):\n"
                         "\tself.pick('mustard')\n"
                         "\tself.place('table on the right')\n"
                         "\tself.done()\n"
                         "\treturn\n"
                         "```\n"),
                     5: ("```python\n"
                         "def cheezit_to_top_shelf(self):\n"
                         "\tself.pick('cheezit')\n"
                         "\tself.place('top shelf')\n"
                         "\tself.done()\n"
                         "\treturn\n"
                         "```\n"),
                     6: ("```python\n"
                         "def pringles_to_right_table(self):\n"
                         "\tself.pick('pringles')\n"
                         "\tself.place('table on the right')\n"
                         "\tself.done()\n"
                         "\treturn\n"
                         "```\n"),
                     7: ("```python\n"
                         "def cheezit_to_bottom_shelf(self):\n"
                         "\tself.pick('cheezit')\n"
                         "\tself.place('bottom shelf')\n"
                         "\tself.done()\n"
                         "\treturn\n"
                         "```\n"),
                     8: ("```python\n"
                         "def food_to_left_table(self):\n"
                         "\tfor object in self.object_list:\n"
                         "\t\t# check if object is food\n"
                         "\t\tif self.check_condition(f'{object} is food', type='property', function='food_to_left_table'):\n"
                         "\t\t\tself.pick(object)\n"
                         "\t\t\tself.place('table on the left')\n"
                         "\tself.done()\n"
                         "\treturn\n"
                         "```\n"),
                     9: ("```python\n"
                         "def snacks_to_right_table(self):\n"
                         "\tfor object in self.object_list:\n"
                         "\t\t# check if object is snack\n"
                         "\t\tif self.check_condition(f'{object} is snack', type='property', function='snacks_to_right_table'):\n"
                         "\t\t\tself.pick(object)\n"
                         "\t\t\tself.place('table on the right')\n"
                         "\tself.done()\n"
                         "\treturn\n"
                         "```\n"),
                     10: ("```python\n"
                          "def pick_cup(self):\n"
                          "\tself.pick('glass')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     11: ("```python\n"
                          "def cheezit_to_left_table(self):\n"
                          "\tself.pick('cheezit')\n"
                          "\tself.place('table on the left')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     12: ("```python\n"
                          "def wipe_table(self):\n"
                          "\tself.pick('sponge')\n"
                          "\tself.wipe('table on the left')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     13: ("```python\n"
                          "def bottle_to_right_table(self):\n"
                          "\tself.pick('bottle')\n"
                          "\tself.place('table on the right')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     14: ("```python\n"
                          "def mug_to_left_table(self):\n"
                          "\tself.pick('cup')\n"
                          "\tself.place('table on the left')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     15: ("```python\n"
                          "def groceries_to_left_table_and_wipe_left_table(self):\n"
                          "\tfor object in self.object_list:\n"
                          "\t\t# check if object is grocery\n"
                          "\t\tif self.check_condition(f'{object} is grocery', type='property', function='groceries_to_left_table_and_wipe_left_table'):\n"
                          "\t\t\tself.pick(object)\n"
                          "\t\t\tself.place('bottom shelf')\n"
                          "\tself.pick('sponge')\n"
                          "\tself.wipe('table on the left')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),

                     16: ("```python\n"
                          "def pour_water_to_glass(self):\n"
                          "\t# check for zone trigger on zone 2"
                          "\tif self.check_condition('glass is in zone 2', type='location', function='pour_water_to_glass'):\n"
                          "\t\tself.pick('bottle')\n"
                          "\t\tself.pour('glass')\n"
                          "\t\tself.place('table on the right')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     17: ("```python\n"
                          "def mustard_to_table(self):\n"
                          "\t# check for zone trigger on shelf"
                          "\tif self.check_condition('mustard is on the shelf', type='location', function='mustard_to_table'):\n"
                          "\t\tself.pick('mustard')\n"
                          "\t\tself.place('table on the right')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     18: ("```python\n"
                          "def cheezit_to_top_shelf(self):\n"
                          "\t# check for zone trigger on zone 1"
                          "\tif self.check_condition('cheezit is in zone 1', type='location', function='cheezit_to_top_shelf'):\n"
                          "\t\tself.pick('cheezit')\n"
                          "\t\tself.place('top shelf')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     19: ("```python\n"
                          "def pringles_to_right_table(self):\n"
                          "\t# check for zone trigger on left table"
                          "\tif self.check_condition('pringles is on left table', type='location', function='pringles_to_right_table'):\n"
                          "\t\tself.pick('pringles')\n"
                          "\t\tself.place('table on the right')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     20: ("```python\n"
                          "def cheezit_to_bottom_shelf(self):\n"
                          "\t# check for zone trigger on zone 1"
                          "\tif self.check_condition('cheezit is in zone 1', type='location', function='cheezit_to_bottom_shelf'):\n"
                          "\t\tself.pick('cheezit')\n"
                          "\t\tself.place('bottom shelf')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     21: ("```python\n"
                          "def food_to_left_table(self):\n"
                          "\tfor object in self.object_list:\n"
                          "\t\t# check for zone trigger on zone 3 and if object is food\n"
                          "\t\tif self.check_condition(f'{object} is in zone 3', type='location', function='food_to_left_table') and self.check_condition(f'{object} is food', type='property', function='food_to_left_table'):\n"
                          "\t\t\tself.pick(object)\n"
                          "\t\t\tself.place('table on the left')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     22: ("```python\n"
                          "def snacks_to_right_table(self):\n"
                          "\tfor object in self.object_list:\n"
                          "\t\t# check for zone trigger on left table and if object is snack\n"
                          "\t\tif self.check_condition(f'{object} is on the left table', type='location', function='snacks_to_right_table') and self.check_condition(f'{object} is snack', type='property', function='snacks_to_right_table'):\n"
                          "\t\t\tself.pick(object)\n"
                          "\t\t\tself.place('table on the right')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     23: ("```python\n"
                          "def pick_cup(self):\n"
                          "\t# check for zone trigger on zone 2"
                          "\tif self.check_condition('cup is in zone 2', type='location', function='pick_cup'):\n"
                          "\t\tself.pick('glass')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     24: ("```python\n"
                          "def cheezit_to_left_table(self):\n"
                          "\t# check for zone trigger on zone 4"
                          "\tif self.check_condition('cheezit is in zone 4', type='location', function='cheezit_to_left_table'):\n"
                          "\t\tself.pick('cheezit')\n"
                          "\t\tself.place('table on the left')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     25: ("```python\n"
                          "def bottle_to_right_table(self):\n"
                          "\t# check for zone trigger on left table"
                          "\tif self.check_condition('water bottle is on the left table', type='location', function='bottle_to_right_table'):\n"
                          "\t\tself.pick('bottle')\n"
                          "\t\tself.place('table on the right')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     26: ("```python\n"
                          "def mug_to_left_table(self):\n"
                          "\t# check for zone trigger on right table"
                          "\tif self.check_condition('mug is on the right table', type='location', function='mug_to_left_table'):\n"
                          "\t\tself.pick('cup')\n"
                          "\t\tself.place('table on the left')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     27: ("```python\n"
                          "def groceries_to_left_table_and_wipe_left_table(self):\n"
                          "\tfor object in self.object_list:\n"
                          "\t\t# check for zone trigger on left table and if object is grocery\n"
                          "\t\tif self.check_condition(f'{object} is on the left table', type='location', function='groceries_to_left_table_and_wipe_left_table') and self.check_condition(f'{object} is grocery', type='property', function='groceries_to_left_table_and_wipe_left_table'):\n"
                          "\t\t\tself.pick(object)\n"
                          "\t\t\tself.place('bottom shelf')\n"
                          "\tself.pick('sponge')\n"
                          "\tself.wipe('table on the left')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),

                     28: ("```python\n"
                          "def timed_pour_water_to_glass(self):\n"
                          "\t# check if 2 minutes have passed for time trigger\n"
                          "\tif not self.check_condition(f'120 seconds have passed', type='time', function='timed_pour_water_to_glass'):\n"
                          "\t\treturn\n"
                          "\tself.pick('bottle')\n"
                          "\tself.pour('glass')\n"
                          "\tself.place('table on the right')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     29: ("```python\n"
                          "def timed_do_wave(self):\n"
                          "\t# check if 2 minutes have passed for time trigger\n"
                          "\tif not self.check_condition(f'120 seconds have passed', type='time', function='timed_do_wave'):\n"
                          "\t\treturn\n"
                          "\tself.wave()\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     30: ("```python\n"
                          "def timed_do_wave(self):\n"
                          "\t# check if 10 minutes have passed for time trigger\n"
                          "\tif not self.check_condition(f'600 seconds have passed', type='time', function='timed_do_wave'):\n"
                          "\t\treturn\n"
                          "\tself.wave()\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     31: ("```python\n"
                          "def timed_mustard_to_table(self):\n"
                          "\t# check if 1 minute has passed for time trigger\n"
                          "\tif not self.check_condition(f'60 seconds have passed', type='time', function='timed_mustard_to_table'):\n"
                          "\t\treturn\n"\
                          "\tself.pick('mustard')\n"
                          "\tself.place('table on the right')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     32: ("```python\n"
                          "def timed_cheezit_to_top_shelf(self):\n"
                          "\t# check if 5 minutes have passed for time trigger\n"
                          "\tif not self.check_condition(f'300 seconds have passed', type='time', function='timed_cheezit_to_top_shelf'):\n"
                          "\t\treturn\n"
                          "\tself.pick('cheezit')\n"
                          "\tself.place('top shelf')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     33: ("```python\n"
                          "def timed_pringles_to_right_table(self):\n"
                          "\t# check if 5 minutes have passed for time trigger\n"
                          "\tif not self.check_condition(f'300 seconds have passed', type='time', function='timed_pringles_to_right_table'):\n"
                          "\t\treturn\n"
                          "\tself.pick('pringles')\n"
                          "\tself.place('table on the right')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     34: ("```python\n"
                          "def timed_cheezit_to_bottom_shelf(self):\n"
                          "\t# check if half minute has passed for time trigger\n"
                          "\tif not self.check_condition(f'30 seconds have passed', type='time', function='timed_cheezit_to_bottom_shelf'):\n"
                          "\t\treturn\n"
                          "\tself.pick('cheezit')\n"
                          "\tself.place('bottom shelf')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     35: ("```python\n"
                          "def timed_food_to_left_table(self):\n"
                          "\t# check if 3 minutes have passed for time trigger\n"
                          "\tif not self.check_condition(f'180 seconds have passed', type='time', function='timed_food_to_left_table'):\n"
                          "\t\treturn\n"
                          "\tfor object in self.object_list:\n"
                          "\t\t# check if object is food\n"
                          "\t\tif self.check_condition(f'{object} is food', type='property', function='timed_food_to_left_table'):\n"
                          "\t\t\tself.pick(object)\n"
                          "\t\t\tself.place('table on the left')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     36: ("```python\n"
                          "def timed_snacks_to_right_table(self):\n"
                          "\t# check if 10 minutes have passed for time trigger\n"
                          "\tif not self.check_condition(f'600 seconds have passed', type='time', function='timed_snacks_to_right_table'):\n"
                          "\t\treturn\n"
                          "\tfor object in self.object_list:\n"
                          "\t\t# check if object is snack\n"
                          "\t\tif self.check_condition(f'{object} is snack', type='property', function='timed_snacks_to_right_table'):\n"
                          "\t\t\tself.pick(object)\n"
                          "\t\t\tself.place('table on the right')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     37: ("```python\n"
                          "def timed_pick_cup(self):\n"
                          "\t# check if 5 minutes have passed for time trigger\n"
                          "\tif not self.check_condition(f'300 seconds have passed', type='time', function='timed_pick_cup'):\n"
                          "\t\treturn\n"
                          "\tself.pick('glass')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     38: ("```python\n"
                          "def timed_cheezit_to_left_table(self):\n"
                          "\t# check if 1 minute has passed for time trigger\n"
                          "\tif not self.check_condition(f'60 seconds have passed', type='time', function='timed_cheezit_to_left_table'):\n"
                          "\t\treturn\n"
                          "\tself.pick('cheezit')\n"
                          "\tself.place('table on the left')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     39: ("```python\n"
                          "def timed_wipe_table(self):\n"
                          "\t# check if 15 minutes have passed for time trigger\n"
                          "\tif not self.check_condition(f'900 seconds have passed', type='time', function='timed_wipe_table'):\n"
                          "\t\treturn\n"
                          "\tself.pick('sponge')\n"
                          "\tself.wipe('table on the left')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     40: ("```python\n"
                          "def timed_bottle_to_right_table(self):\n"
                          "\t# check if 4 minutes have passed for time trigger\n"
                          "\tif not self.check_condition(f'240 seconds have passed', type='time', function='timed_bottle_to_right_table'):\n"
                          "\t\treturn\n"
                          "\tself.pick('bottle')\n"
                          "\tself.place('table on the right')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     41: ("```python\n"
                          "def timed_mug_to_left_table(self):\n"
                          "\t# check if 3 minutes have passed for time trigger\n"
                          "\tif not self.check_condition(f'180 seconds have passed', type='time', function='timed_mug_to_left_table'):\n"
                          "\t\treturn\n"
                          "\tself.pick('cup')\n"
                          "\tself.place('table on the left')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     42: ("```python\n"
                          "def timed_groceries_to_left_table_and_wipe_left_table(self):\n"
                          "\t# check if 3 minutes have passed for time trigger\n"
                          "\tif not self.check_condition(f'180 seconds have passed', type='time', function='timed_groceries_to_left_table_and_wipe_left_table'):\n"
                          "\t\treturn\n"
                          "\tfor object in self.object_list:\n"
                          "\t\t# check if object is grocery\n"
                          "\t\tif self.check_condition(f'{object} is grocery', type='property', function='timed_groceries_to_left_table_and_wipe_left_table'):\n"
                          "\t\t\tself.pick(object)\n"
                          "\t\t\tself.place('bottom shelf')\n"
                          "\tself.pick('sponge')\n"
                          "\tself.wipe('table on the left')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),

                     43: ("```python\n"
                          "def pour_water_to_glass(self):\n"
                          "\t# check for glass on zone 2 for 2 minutes"
                          "\tif self.check_condition('glass is in zone 2 for 120 seconds', type='location and time', function='pour_water_to_glass'):\n"
                          "\t\tself.pick('bottle')\n"
                          "\t\tself.pour('glass')\n"
                          "\t\tself.place('table on the right')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     44: ("```python\n"
                          "def mustard_to_table(self):\n"
                          "\t# check for mustard on shelf for 1 minute"
                          "\tif self.check_condition('mustard is on the shelf for 60 seconds', type='location and time', function='mustard_to_table'):\n"
                          "\t\tself.pick('mustard')\n"
                          "\t\tself.place('table on the right')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     45: ("```python\n"
                          "def cheezit_to_top_shelf(self):\n"
                          "\t# check for cheezit on zone 1 for 5 minutes"
                          "\tif self.check_condition('cheezit is in zone 1 for 300 seconds', type='location and time', function='cheezit_to_top_shelf'):\n"
                          "\t\tself.pick('cheezit')\n"
                          "\t\tself.place('top shelf')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     46: ("```python\n"
                          "def pringles_to_right_table(self):\n"
                          "\t# check for pringles on left table for 5 minutes"
                          "\tif self.check_condition('pringles is on left table for 300 seconds', type='location and time', function='pringles_to_right_table'):\n"
                          "\t\tself.pick('pringles')\n"
                          "\t\tself.place('table on the right')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     47: ("```python\n"
                          "def cheezit_to_bottom_shelf(self):\n"
                          "\t# check for cheezit on zone 1 for half minute"
                          "\tif self.check_condition('cheezit is in zone 1 for 30 seconds', type='location and time', function='cheezit_to_bottom_shelf'):\n"
                          "\t\tself.pick('cheezit')\n"
                          "\t\tself.place('bottom shelf')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     48: ("```python\n"
                          "def food_to_left_table(self):\n"
                          "\tfor object in self.object_list:\n"
                          "\t\t# check for object on zone 3 for 3 minutes and if object is food\n"
                          "\t\tif self.check_condition(f'{object} is in zone 3 for 180 seconds', type='location and time', function='food_to_left_table') and self.check_condition(f'{object} is food', type='property', function='food_to_left_table'):\n"
                          "\t\t\tself.pick(object)\n"
                          "\t\t\tself.place('table on the left')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     49: ("```python\n"
                          "def snacks_to_right_table(self):\n"
                          "\tfor object in self.object_list:\n"
                          "\t\t# check for object on left table for 10 minutes and if object is snack\n"
                          "\t\tif self.check_condition(f'{object} is on the left table for 600 seconds', type='location and time', function='snacks_to_right_table') and self.check_condition(f'{object} is snack', type='property', function='snacks_to_right_table'):\n"
                          "\t\t\tself.pick(object)\n"
                          "\t\t\tself.place('table on the right')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     50: ("```python\n"
                          "def pick_cup(self):\n"
                          "\t# check for cup on zone 2 for 5 minutes"
                          "\tif self.check_condition('cup is in zone 2 for 300 seconds', type='location and time', function='pick_cup'):\n"
                          "\t\tself.pick('glass')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     51: ("```python\n"
                          "def cheezit_to_left_table(self):\n"
                          "\t# check for cheezit on zone 4 for 1 minute"
                          "\tif self.check_condition('cheezit is in zone 4 for 60 seconds', type='location and time', function='cheezit_to_left_table'):\n"
                          "\t\tself.pick('cheezit')\n"
                          "\t\tself.place('table on the left')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"), 
                     52: ("```python\n"
                          "def bottle_to_right_table(self):\n"
                          "\t# check for water bottle on left table for 4 minutes"
                          "\tif self.check_condition('water bottle is on the left table for 240 seconds', type='location and time', function='bottle_to_right_table'):\n"
                          "\t\tself.pick('bottle')\n"
                          "\t\tself.place('table on the right')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     53: ("```python\n"
                          "def mug_to_left_table(self):\n"
                          "\t# check for mug on right table for 3 minutes"
                          "\tif self.check_condition('mug is on the right table for 180 seconds', type='location and time', function='mug_to_left_table'):\n"
                          "\t\tself.pick('cup')\n"
                          "\t\tself.place('table on the left')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n"),
                     54: ("```python\n"
                          "def groceries_to_left_table_and_wipe_left_table(self):\n"
                          "\tfor object in self.object_list:\n"
                          "\t\t# check for object on left table for 3 minutes and if object is grocery\n"
                          "\t\tif self.check_condition(f'{object} is on the left table for 180 seconds', type='location', function='groceries_to_left_table_and_wipe_left_table') and self.check_condition(f'{object} is grocery', type='property', function='groceries_to_left_table_and_wipe_left_table'):\n"
                          "\t\t\tself.pick(object)\n"
                          "\t\t\tself.place('bottom shelf')\n"
                          "\tself.pick('sponge')\n"
                          "\tself.wipe('table on the left')\n"
                          "\tself.done()\n"
                         "\treturn\n"
                          "```\n")
                     }