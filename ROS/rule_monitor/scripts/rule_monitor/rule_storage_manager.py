import os
import json
import warnings
from rule_monitor.msg import Rule,TriggerType

class RuleStorageManager:
    def __init__(self, filename="rules.json", directory="src/rule_monitor/saved_data"):
        self.directory = os.path.join(os.getcwd(), directory)  # Directory for rules
        self.file_path = os.path.join(self.directory, filename)  # File path for saving rules
        
        # Ensure the rules directory exists
        if not os.path.exists(self.directory):
            os.makedirs(self.directory)

    def load_rules(self):
        """Loads the rules from a JSON file."""
        try:
            with open(self.file_path, "r") as file:
                loaded_rules = json.load(file)
                print("Rules successfully loaded from file.")
                
                # Convert the loaded rules back to a list of Rule objects
                rules = []
                if loaded_rules:  # Ensure loaded_rules is not None or empty
                    for rule_data in loaded_rules:
                        rule = Rule()  # Create a Rule object
                        rule.name = rule_data['name']
                        if rule_data['type'] == TriggerType.NONE:
                            rule.type.type = TriggerType.NONE
                        elif rule_data['type'] == TriggerType.ZONE:
                            rule.type.type = TriggerType.ZONE
                        elif rule_data['type'] == TriggerType.TIMER:
                            rule.type.type = TriggerType.TIMER
                        rule.function = rule_data['function']
                        rule.description = rule_data['description']
                        rule.is_active = rule_data['is_active']
                        rule.new_index = -1  # Assuming you want to reset new_index here
                        rules.append(rule)  # Add each Rule object to the list
                return rules
        except FileNotFoundError:
            warnings.warn(f"No rules file found at {self.file_path}. Starting with an empty rule set.", UserWarning)
            return []
        except json.JSONDecodeError:
            warnings.warn(f"Failed to decode the rules file. The file may be corrupted: {self.file_path}.", UserWarning)
            return []

    def save_rules(self, rules):
        """Saves the current rules to a JSON file."""
        rules_to_save = []
        for rule in rules:
            rules_to_save.append({
                'name': rule.name,
                'type': rule.type.type,
                'function': rule.function,
                'description': rule.description,
                'is_active': rule.is_active
            })

        with open(self.file_path, "w") as file:
            json.dump(rules_to_save, file)
        print(f"Rules successfully saved {len(rules)} rules to {self.file_path}.")