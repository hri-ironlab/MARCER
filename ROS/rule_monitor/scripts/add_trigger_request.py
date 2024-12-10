import rospy
from std_msgs.msg import Int32
from rule_monitor.msg import Rule, TriggerType
from rule_monitor.srv import UpdateRule, UpdateRuleRequest, UpdateRuleResponse
from std_srvs.srv import SetBool, SetBoolRequest

def update_trigger_list(req):
    rospy.loginfo(f"Received trigger list update request: {req}")
    return True

def print_triggered_action(msg):
    rospy.loginfo(f"Triggered action: {msg.data}")
def extract_function_name(function_def):
    # Split the function definition string to extract the function name
    # Assumes the function starts with 'def'
    function = function_def.split("```python")[1].split("```")[0]
    parts = function_def.split()[1].split('(')[0]   
    function_name = parts[1].split('(')[0]   
    print(function)
    print(function_name)
    return function_name

if __name__ == "__main__":
    rospy.init_node("add_trigger_request")



    # Example usage
    function_def = """def pick_and_place_pringles_back():
        pick('Pringles')
        place('table')
        return"""

    function_name = extract_function_name(function_def)
    exit()
    apply_trigger_client = rospy.ServiceProxy(
        "rule_monitor/update_rule", UpdateRule
    )
    run_stop_service = rospy.ServiceProxy(
        "rule_monitor/run_stop", SetBool
    )

    # update_timer_list = rospy.Subscriber("unity/update_trigger_list", UpdateTriggerList, update_trigger_list)

    triggered_action_publisher = rospy.Subscriber(
        "rule_monitor/triggered_action", Trigger, print_triggered_action
    )

    rospy.sleep(1)
    rospy.loginfo("adding object zone")
    req = ApplyTriggerRequest() 
    req.trigger.id = 1
    req.trigger.operation = Trigger.ADD
    req.trigger.trigger_type = Trigger.OBJECT_ZONE
    req.trigger.zone_name = "Zone 1"
    req.trigger.action = "move objects from Zone 1 to the left table"
    success = apply_trigger_client.call(req)
    rospy.sleep(1)

    # rospy.loginfo("adding timer")
    # req = ApplyTriggerRequest() 
    # req.trigger.id = 2
    # req.trigger.operation = Trigger.ADD
    # req.trigger.trigger_type = Trigger.TIMER
    # req.trigger.wait_time = 3
    # req.trigger.zone_name = "Zone 2"
    # req.trigger.action = "Trigger the timer node"
    # success = apply_trigger_client.call(req)

    # rospy.loginfo("adding timer")
    # req = ApplyTriggerRequest() 
    # req.trigger.id = 2
    # req.trigger.operation = Trigger.ADD
    # req.trigger.trigger_type = Trigger.TIMER
    req.trigger.wait_time = 100
    # req.trigger.zone_name = "Zone 2"
    # req.trigger.action = "Trigger the timer node"
    # success = apply_trigger_client.call(req)

    # rospy.loginfo("adding timer")
    # req = ApplyTriggerRequest() 
    # req.trigger.id = 2
    # req.trigger.operation = Trigger.ADD
    # req.trigger.trigger_type = Trigger.TIMER
    # req.trigger.wait_time = 4
    # req.trigger.zone_name = "Zone 2"
    # req.trigger.action = "Trigger the timer node"
    # success = apply_trigger_client.call(req)

    rospy.sleep(1)
    req = SetBoolRequest()
    req.data = True
    success = run_stop_service.call(req)
    rospy.loginfo(f"Run stop service call was successful: {success.success}")

    rospy.spin()