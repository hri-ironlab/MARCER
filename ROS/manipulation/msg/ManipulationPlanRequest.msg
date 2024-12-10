# By default, the task type is PICK_AND_PLACE
int32 DONE=0
int32 PICK=1
int32 PLACE=2
int32 MOVE_ARM=3
int32 OPEN_HAND=4
int32 CLOSE_HAND=5
int32 WAVE=6
int32 DANCE=7
int32 WIPE=8
int32 POUR=9
int32 READY_POSE=10
int32 task_type

# Name to label the task
string task_name

# Surface, object to pick, object to pour into
string target_object_name

# Place Position
geometry_msgs/Pose place_pose