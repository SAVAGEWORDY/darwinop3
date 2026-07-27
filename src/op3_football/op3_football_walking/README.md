# op3_football_walking

Fork of ROBOTIS `op3_walking_module` for the OP3 football project.

Currently keeps the same ROS topic API (`/robotis/walking/*`) and module name `walking_module`
so it can replace the stock library in a forked manager later.

Modify gait / balance / timing here. Stock `op3_manager` still loads the original module until we wire this fork in.
