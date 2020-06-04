# -*- coding: UTF-8 -*-
#!/usr/bin/env python

import roslaunch
import rospy

def launch(file_name):
    # Launch a launch file (do a navigaiton)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [file_name])
    launch.start()
    #rospy.loginfo("started")

    # opt 1. wait for a required node to shutdown
    # one of the node is set required="true" in the launch file
    # so it will automatically shutdown after that node is shutdown
    try:
        launch.spin()
    except:
        launch.shutdown()
    finally:
        # After Ctrl+C, stop all nodes from running
        launch.shutdown()

    # opt 2. wait for a duration
    # rospy.sleep(20.0)  # wait for x seconds
    # launch.shutdown()

# for test only
if __name__ == "__main__":
    launch('/home/wuch/tracking_ws/src/Tracking/dwa_stage/launch/maze.launch')