#! /usr/bin/env python
import math,rospy
import rospy
import geometry_msgs.msg
from gripper_control import GripperController
from move_group_interface import MoveGroupPythonInteface
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.msg import ModelStates
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

def print_information(mgpi):

    mgpi.print_useful_info()  # Print useful information
    mgpi.print_robot_state()  # Print the current state of the robot

def go_to_home_position(mgpi):

    # predefined state "home" is defined in universal_robot/ur5_e_moveit_config/config/ur5e.srdf
    mgpi.move_to_joint_state(mgpi.home_joint_angles)

def move_arm(mgpi,x,y,z):

    print("---------------------------------")
    print("move to:",x,y,z)

    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z
    target_pose.orientation.x = 0.5
    target_pose.orientation.y = 0.5
    target_pose.orientation.z = -0.5
    target_pose.orientation.w = 0.5
    mgpi.move_eef_to_pose(target_pose)


def Hold_block(mgpi,obj_name):

    mgpi.close_gripper(obj_name)

def Realease_block(mgpi,obj_name):

    mgpi.open_gripper(obj_name)


def print_position():

        rospy.sleep(1) #sleep for 1s

        position_sub = rospy.wait_for_message('/gazebo/model_states', ModelStates,timeout=None) # Recive message one time

        # define the global variables
        global goal_x, goal_y, block_1_x, block_1_y, block_1_z,block_2_x, block_2_y, block_2_z, block_3_x, block_3_y, block_3_z, block_4_x, block_4_y, block_4_z, block_5_x, block_5_y, block_5_z

        # print out the position of block and goal
        print("The goal position in x-axis", position_sub.pose[2].position.x ,"The goal position in y-axis:" ,position_sub.pose[2].position.y, "The goal position in z-axis:", position_sub.pose[2].position.z)
        goal_x = position_sub.pose[2].position.x
        goal_y = position_sub.pose[2].position.y
        print("The block_1 position in x-axis", position_sub.pose[3].position.x ,"The block_1 position in y-axis:" ,position_sub.pose[3].position.y, "The block_1 position in z-axis:", position_sub.pose[3].position.z)
        block_1_x = position_sub.pose[3].position.x
        block_1_y = position_sub.pose[3].position.y
        block_1_z = position_sub.pose[3].position.z
        print("The block_2 position in x-axis", position_sub.pose[4].position.x ,"The block_2 position in y-axis:" ,position_sub.pose[4].position.y, "The block_2 position in z-axis:", position_sub.pose[4].position.z)
        block_2_x = position_sub.pose[4].position.x
        block_2_y = position_sub.pose[4].position.y
        block_2_z = position_sub.pose[4].position.z
        print("The block_3 position in x-axis", position_sub.pose[5].position.x ,"The block_3 position in y-axis:" ,position_sub.pose[5].position.y, "The block_3 position in z-axis:", position_sub.pose[5].position.z)
        block_3_x = position_sub.pose[5].position.x
        block_3_y = position_sub.pose[5].position.y
        block_3_z = position_sub.pose[5].position.z
        print("The block_4 position in x-axis", position_sub.pose[6].position.x ,"The block_4 position in y-axis:" ,position_sub.pose[6].position.y, "The block_4 position in z-axis:", position_sub.pose[6].position.z)
        block_4_x = position_sub.pose[6].position.x
        block_4_y = position_sub.pose[6].position.y
        block_4_z = position_sub.pose[6].position.z
        print("The block_5 position in x-axis", position_sub.pose[7].position.x ,"The block_5 position in y-axis:" ,position_sub.pose[7].position.y, "The block_5 position in z-axis:", position_sub.pose[7].position.z)
        block_5_x = position_sub.pose[7].position.x
        block_5_y = position_sub.pose[7].position.y
        block_5_z = position_sub.pose[7].position.z


def main():
    try:
        raw_input("Press Enter to build a tower")    # waits for Enter
        print("  1. Creating and initializing the interface to the robot")
        print("------------------------------------------------------------")
        mgpi = MoveGroupPythonInteface()             # Create and Initialize the interface
        print("Simulation environment done.")
        raw_input("Press Enter to build a tower")    # waits for Enter
        print("  2. Print Basic Information")
        print("------------------------------------------------------------")
        print_information(mgpi)
        print("  3. Getting the block and goal position")
        print("------------------------------------------------------------")
        print_position()
        print("Location loading done.")
        go_to_home_position(mgpi)                    # start at home position

        # Start
        # First block--------------------------------------1
        move_arm(mgpi, block_1_x, block_1_y, 0.4)    #Go to the position of the block_1
        move_arm(mgpi, block_1_x, block_1_y, 0.081)  # This height need to change to hardware block height 0.061
        # Open gripper
        Hold_block(mgpi, "block_1")

        move_arm(mgpi, block_1_x, block_1_y, 0.4)
        move_arm(mgpi, goal_x, goal_y, 0.4)         #Go to the Goal point
        move_arm(mgpi, goal_x, goal_y, 0.082)       # This height need to change to hardware block height 0.064
        Realease_block(mgpi, "block_1")             # close gripper

        move_arm(mgpi, goal_x, goal_y, 0.4)

        print_position() # update new loaction to minimise the position change by collision

        # Next block---------------------------------------2
        move_arm(mgpi, block_2_x, 0.45, 0.4)
        move_arm(mgpi, block_2_x, block_2_y, 0.4)
        move_arm(mgpi, block_2_x, block_2_y, 0.081)  # This height need to change to hardware block height 0.061
        # Open gripper
        Hold_block(mgpi, "block_2")

        move_arm(mgpi, block_2_x, block_2_y, 0.4)
        move_arm(mgpi, goal_x, goal_y, 0.4)
        move_arm(mgpi, goal_x, goal_y, 0.167)        # This height need to change to hardware block height 0.125
        Realease_block(mgpi, "block_2")              # close gripper

        move_arm(mgpi, goal_x, goal_y, 0.4)

        print_position() # update new loaction to minimise the position change by collision

        # Next block---------------------------------------3
        move_arm(mgpi, block_3_x, 0.45, 0.4)
        move_arm(mgpi, block_3_x, block_3_y, 0.4)
        move_arm(mgpi, block_3_x, block_3_y, 0.081)   # This height need to change to hardware block height 0.061


         # Open gripper
        Hold_block(mgpi, "block_3")
        move_arm(mgpi, block_3_x, block_3_y, 0.4)
        move_arm(mgpi, goal_x, goal_y, 0.4)
        move_arm(mgpi, goal_x, goal_y, 0.257)          # This height need to change to hardware block height 0.188
        Realease_block(mgpi, "block_3")                # close gripper

        move_arm(mgpi, goal_x, goal_y, 0.4)

        print_position() # update new loaction to minimise the position change by collision

        # Next block---------------------------------------4
        move_arm(mgpi, block_4_x, 0.45, 0.4)
        move_arm(mgpi, block_4_x, block_4_y, 0.4)
        move_arm(mgpi, block_4_x, block_4_y, 0.081)    # This height need to change to hardware block height 0.061
        # Open gripper
        Hold_block(mgpi, "block_4")

        move_arm(mgpi, block_4_x, block_4_y, 0.4)
        move_arm(mgpi, goal_x, goal_y,0.4)
        move_arm(mgpi, goal_x, goal_y,0.345)            # This height need to change to hardware block height 0.249
        Realease_block(mgpi, "block_4")                 # close gripper

        move_arm(mgpi, goal_x, goal_y, 0.4)

        print_position() # update new loaction to minimise the position change by collision

        # Next block---------------------------------------5
        move_arm(mgpi, block_5_x, 0.45, 0.4)
        move_arm(mgpi, block_5_x, block_5_y, 0.4)
        move_arm(mgpi, block_5_x, block_5_y, 0.081)     # This height need to change to hardware block height 0.061
        # Open gripper
        Hold_block(mgpi, "block_5")

        move_arm(mgpi, block_5_x, block_5_y, 0.43)
        move_arm(mgpi, goal_x, goal_y, 0.43)             # This height need to change to hardware block height 0.31
        Realease_block(mgpi, "block_5")                  # close gripper

        print_position() # Show the final location

        print("  Tower finish !!")
        print("------------------------------------------------------------")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
