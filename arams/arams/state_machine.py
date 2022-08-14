#!/usr/bin/env python3

# ROS Python
import rclpy
import rclpy.node
from rclpy.action import ActionClient
# SMACH
import smach
import smach_ros
# Numpy
import numpy
# Messages
from std_msgs.msg import String, Bool
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from custom_msg.msg import ViewPoints
# Services
from custom_msg.srv import ViewPointsSrv, DetectionSrv
# Time
import time
# OS
import os
# from os.path import expanduser



# Postion for Explorer State: 
# [position.x, position.y, orientation.w, orientation.z]
position_list = numpy.array([
    [-1.1, 0.4, 0, 1],              # Postion A,
    [-0.6, 3.2, 0.707, 0.707],      # Postion B,
    [0.7, 2.3, 0.707, -0.707],      # ...
    [1.5, 0.5, 1, 0],
    [2.5, 2.5, 0.707, 0.707], 
    [2.1, 4.2, 1, 0],
    [-3, 2.4, 0, 1],
    [-4.4, 6, 0.707, 0.707],
    [-2.1, 5.6, 1, 0],
    [0.9, 5.6, 1, 0]
])



class Init_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "aborted"])
        # As with any constructor, place your additional setup here

    def execute(self, ud):
        print("\nState:Init_pose:execute")  
        global state_machine, inti_pose_pub 
        outcome = 'aborted'

        try:
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = state_machine.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.pose.orientation.w = 0.697
            msg.pose.pose.orientation.z = 0.717
            
            inti_pose_pub.publish(msg)

            time.sleep(5)
            outcome = 'succeeded'

        except KeyboardInterrupt:
            # manual interrupt if init position failed
            pass

        return outcome

class Explore(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "aborted", "finish"])
        # As with any constructor, place your additional setup here

    def execute(self, ud):
        print("\nState:Explore:execute")  
        global state_machine, node_handle, nav_to_pose_client #node for autonomous explorer
        global i # iteration

        if i>9:
            return 'finish' # --> next State: GetViewPoints

        goalPosition = NavigateToPose.Goal()
        goalPosition.pose.header.stamp = state_machine.get_clock().now().to_msg()
        goalPosition.pose.header.frame_id = "map"
        goalPosition.pose.pose.position.x = position_list[i][0]
        goalPosition.pose.pose.position.y = position_list[i][1]
        goalPosition.pose.pose.orientation.w = position_list[i][2]
        goalPosition.pose.pose.orientation.z = position_list[i][3]
        
        print("Received new goal => X: " + str(goalPosition.pose.pose.position.x) + " Y: " + str(goalPosition.pose.pose.position.y))

        # wait for the server being available
        while not nav_to_pose_client.wait_for_server(timeout_sec=2.0):
            print("Server still not available; waiting...")

        send_goal_future = nav_to_pose_client.send_goal_async(goalPosition)
        
        rclpy.spin_until_future_complete(node_handle, send_goal_future)

        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            print("Goal was rejected")
            return 'aborted'    # --> repeat State

        print("Goal Accepted!")

        get_result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(node_handle, get_result_future)

        status = get_result_future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            print("Reached Goal!!!")
            i=i+1
            return 'succeeded'  # --> next State: Rotate 
        else:
            print("Goal was not reached!")
            return 'aborted'    # --> repeat State

class Rotate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "aborted"])
        # As with any constructor, place your additional setup here

    def execute(self, ud):
        print("\nState:Rotate:execute")  
        global rotate_pub, rotate_flag
        rotate_flag = 'true'
        outcome = 'aborted'

        msg = Twist()
        msg.angular.z = 0.25

        print('start rotating for: 35s')

        d = 0
        t = time.time()
        
        while d < 34:               # measured for one round
            rotate_pub.publish(msg)
            time.sleep(0.3)
            d = time.time() -t
            
        msg.angular.z = 0.0
        rotate_pub.publish(msg)

        outcome = 'succeeded'
        return outcome

class GetViewPoints(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "aborted"])
        # As with any constructor, place your additional setup here        

    def execute(self, ud):
        print("\nState:GetViewPoints:execute")  
        global node_handle, get_ViewPoits_client, viewPoint_list
        outcome = 'aborted'

        future = get_ViewPoits_client.call_async(ViewPointsSrv.Request())

        rclpy.spin_until_future_complete(node_handle, future)

        try:
            result = future.result()
        except KeyboardInterrupt:
            return outcome

        viewPoint_list = result.list

        print(viewPoint_list)

        outcome = 'succeeded'
        return outcome

class GoToViewPoints(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "aborted", "finish"])
        # As with any constructor, place your additional setup here

    def execute(self, ud):
        print("\nState:GoToViewPoints:execute")  
        global state_machine, node_handle, nav_to_pose_client, id_counter, viewPoint_list, viewPoint_checked

        while id_counter < 30:
            if ((viewPoint_list.x[id_counter] or viewPoint_list.y[id_counter]) and (viewPoint_checked[id_counter] == False)):
                goalPosition = NavigateToPose.Goal()
                goalPosition.pose.header.stamp = state_machine.get_clock().now().to_msg()
                goalPosition.pose.header.frame_id = "map"
                goalPosition.pose.pose.position.x = viewPoint_list.x[id_counter]
                goalPosition.pose.pose.position.y = viewPoint_list.y[id_counter]
                goalPosition.pose.pose.orientation.w = viewPoint_list.w[id_counter]
                goalPosition.pose.pose.orientation.z = viewPoint_list.z[id_counter]
                
                print("Received new goal => X: " + str(goalPosition.pose.pose.position.x) + " Y: " + str(goalPosition.pose.pose.position.y))
                
                # wait for the server being available
                while not nav_to_pose_client.wait_for_server(timeout_sec=2.0):
                    print("Server still not available; waiting...")

                send_goal_future = nav_to_pose_client.send_goal_async(goalPosition)
                
                rclpy.spin_until_future_complete(node_handle, send_goal_future)

                goal_handle = send_goal_future.result()

                if not goal_handle.accepted:
                    print("Goal was rejected")
                    return 'aborted'

                print("Goal Accepted!")

                get_result_future = goal_handle.get_result_async()

                rclpy.spin_until_future_complete(node_handle, get_result_future)

                status = get_result_future.result().status

                if status == GoalStatus.STATUS_SUCCEEDED:
                    print("Reached Goal!!!")
                    return 'succeeded'
                else: 
                    print("Goal not reached!")
                    return 'aborted'      

            id_counter = id_counter +1
        
        id_counter = 0
        return 'finish'                 # checkt all Viewpoints

class DetectionAndPrint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["succeeded", "aborted", "failed"])
        # As with any constructor, place your additional setup here        

    def execute(self, ud):
        print("\nState:DetectionAndPrint:execute")  
        global node_handle, get_detection_client, id_counter, viewPoint_checked, failed_counter, detection
        outcome = 'aborted'
        
        flag = Bool()
        flag.data = True
        flag_pub.publish(flag)
        time.sleep(5)

        future = get_detection_client.call_async(DetectionSrv.Request())

        rclpy.spin_until_future_complete(node_handle, future)

        try:
            result = future.result()
        except KeyboardInterrupt:
            return outcome

        if not ((result.detection == "") or (result.detection == detection)):
            detection = result.detection

            txt = ('\n' + str(id_counter) + ': ' + str(detection))
            print()
            print(txt)
            print()
            with open(file, "a") as f:
                f.write(txt)

            viewPoint_checked[id_counter]=True
            id_counter=id_counter+1

            outcome = 'succeeded'
            return outcome

        elif failed_counter <3:
            # rotate a littel and give Yolo time
            msg = Twist()
            msg.angular.z = -0.1
            d = 0
            t = time.time()
            flag_pub.publish(flag)

            while d < 7:                    # 7 sec right
                rotate_pub.publish(msg)
                time.sleep(0.3)
                d = time.time() -t

            flag_pub.publish(flag)
            msg.angular.z = 0.1

            while d < 21:                   # 14 sec left
                time.sleep(0.3)
                d = time.time() -t

            flag_pub.publish(flag)
            msg.angular.z = -0.1

            while d < 28:                    # 7 sec right
                rotate_pub.publish(msg)
                time.sleep(0.3)
                d = time.time() -t

            flag_pub.publish(flag)
            msg.angular.z = 0.0

            while d < 29:                   # 1 sec stop
                rotate_pub.publish(msg)
                time.sleep(0.3)
                d = time.time() -t

            failed_counter = failed_counter +1
            outcome = 'aborted'

        else:
            txt = ('\n' + str(id_counter) + ': ---')
            print(txt)
            print()
            with open(file, "a") as f:
                f.write(txt)

            viewPoint_checked[id_counter]=False
            id_counter=id_counter+1
            failed_counter=0
            outcome = 'failed'
        
        return outcome



def main():
    # global Variables
    global i, id_counter, viewPoint_list, viewPoint_checked, failed_counter, detection, file
    global state_machine, node_handle, inti_pose_pub, rotate_pub, flag_pub
    global nav_to_pose_client
    global get_ViewPoits_client, get_detection_client

    # ROS initialization
    rclpy.init()

    i=0 # Iteration for Explore
    id_counter = 0 # Interation for GoToViewPoints

    viewPoint_list = ViewPoints()
    viewPoint_checked = [False for x in range(30)]
    failed_counter = 0
    detection = ''

    home = os.path.expanduser("~")
    # file=os.path.join(home, "Desktop", "Solution.txt")
    file = "./Solution.txt"
    with open(file, "w") as f:
        f.write("Solution of Dennis Hölter:")

    # Node, Publisher, Subscriber, Action Client and Service Clients
    state_machine = rclpy.create_node("state_machine")

    node_handle = rclpy.create_node("node_handle")

    inti_pose_pub = state_machine.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)

    rotate_pub = state_machine.create_publisher(Twist, 'cmd_vel', 1)

    flag_pub = state_machine.create_publisher(Bool, 'detect', 10)

    nav_to_pose_client = ActionClient(node_handle, NavigateToPose, 'navigate_to_pose') 

    get_ViewPoits_client = node_handle.create_client(ViewPointsSrv, 'call_ViewPointsSrv')

    get_detection_client = node_handle.create_client(DetectionSrv, 'call_detection')

    # Statemachine and optional userdata
    sm = smach.StateMachine(outcomes=['succeeded'])
    sm.userdata.some_data = []

    with sm:

        # smach.StateMachine.add("Setup", Init_pose(),
        #                   transitions={"succeeded" : "Explore", 
        #                               "aborted" : "Setup",
        #                               }
        # )
                               
        # smach.StateMachine.add("Explore", Explore(),
        #                   transitions={"succeeded" : "Rotate", 
        #                               "aborted" : "Explore",
        #                               "finish" : "GetViewPoints",
        #                               }
        # )

        # smach.StateMachine.add("Rotate", Rotate(),
        #                   transitions={"succeeded" : "Explore",
        #                               "aborted" : "Rotate",
        #                               }
        # )

        smach.StateMachine.add("GetViewPoints", GetViewPoints(),
                          transitions={"succeeded" : "GoToViewPoints",
                                      "aborted" : "GetViewPoints",
                                      }
        ) 

        smach.StateMachine.add("GoToViewPoints", GoToViewPoints(),
                          transitions={"succeeded" : "DetectionAndPrint",
                                      "aborted" : "GoToViewPoints",
                                      "finish" : "succeeded",
                                      }
        ) 

        smach.StateMachine.add("DetectionAndPrint", DetectionAndPrint(),
                          transitions={"succeeded" : "GoToViewPoints",
                                      "aborted" : "DetectionAndPrint",
                                      "failed": "GoToViewPoints",
                                      }
        ) 

    # Statemachine visualisation
    introspection_server = smach_ros.IntrospectionServer("smach_viewer", sm, "/ROOT")
    introspection_server.start()

    try:
        sm.execute()
        rclpy.spin(state_machine)
    except KeyboardInterrupt:
        pass

    inti_pose_pub.destroy()
    rotate_pub.destroy()
    nav_to_pose_client.destroy()
    get_ViewPoits_client.destroy()
    
    state_machine.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()