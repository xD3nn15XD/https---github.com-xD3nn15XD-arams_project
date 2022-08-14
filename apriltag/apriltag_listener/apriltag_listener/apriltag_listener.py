#!/usr/bin/env python3

# ROS Python
import rclpy
# messages
from geometry_msgs.msg import TransformStamped
from apriltag_msgs.msg import AprilTagDetectionArray
from custom_msg.msg import ViewPoints
from custom_msg.srv import ViewPointsSrv
# other
import tf2_ros



def apriltag_detection_callback(msg):
    """ Use a flag that stays high as long as the topic shows that a marker is detected"""
    global flag, marker
    if len(msg.detections) > 0:
        flag = True
        marker = msg
    else:
        flag = False

def timercallback():
    global flag, marker

    # check decision margin (just in case)
    i = len(marker.detections)
    while i > 0:
        margin = marker.detections[i-1].decision_margin
        if(margin > 15):
            id = marker.detections[i-1].id

            tag_frame = "marker"+str(id)

            print("Marker found: "+tag_frame)

            getViewPoint(id, tag_frame)

        i = i-1

# Inspired by NICO SCHOOFS & MATHIAS WITTIG
def getViewPoint(id, tag_frame): 
    global node, viewPoint_list, tfBuffer, counter

    try:
        br = tf2_ros.transform_broadcaster.TransformBroadcaster(node)

        # do a lookup transform between 'map' and 'marker' frame
        t0 = tfBuffer.lookup_transform("map", tag_frame, rclpy.duration.Duration())
        
        # marker ("Pseudo")
        t1 = TransformStamped()
        t1.header.stamp = node.get_clock().now().to_msg()
        t1.header.frame_id = 'map'
        t1.child_frame_id = 'pseudoMarker'
        t1.transform.translation = t0.transform.translation
        t1.transform.rotation = t0.transform.rotation        
        br.sendTransform(t1)

        # Viewpoint 1 m away towards the Marker
        t2 = TransformStamped()
        t2.header.stamp = node.get_clock().now().to_msg()
        t2.header.frame_id = 'pseudoMarker'
        t2.child_frame_id = 'viewPoint'
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 1.0
        t2.transform.rotation.x = 0.5
        t2.transform.rotation.y = 0.5
        t2.transform.rotation.z = -0.5
        t2.transform.rotation.w = 0.5
        br.sendTransform(t2)
        
        try:
            # do a lookup transform between 'map' and 'viewPoint' frame
            t3 = tfBuffer.lookup_transform("map", "viewPoint", rclpy.duration.Duration())

            if abs(t3.transform.rotation.x) < 0.1 or abs(t3.transform.rotation.y) < 0.1 :
                # save mean position
                viewPoint_list.x[id] = ((viewPoint_list.x[id] * counter[id]) + t3.transform.translation.x ) / (counter[id]+1.0)
                viewPoint_list.y[id] = ((viewPoint_list.y[id] * counter[id]) + t3.transform.translation.y ) / (counter[id]+1.0)

                viewPoint_list.w[id] = ((viewPoint_list.w[id] * counter[id]) + t3.transform.rotation.w ) / (counter[id]+1.0)
                viewPoint_list.z[id] = ((viewPoint_list.z[id] * counter[id]) + t3.transform.rotation.z ) / (counter[id]+1.0)

                counter[id] = counter[id] + 1.

            else:
                # flip z axes upwards

                # Viewpoint 1 m away towards the Marker but this time with z axis up
                t4 = TransformStamped()
                t4.header.stamp = node.get_clock().now().to_msg()
                t4.header.frame_id = 'pseudoMarker'
                t4.child_frame_id = 'viewPointCorreted'
                t4.transform.translation.x = 0.0
                t4.transform.translation.y = 0.0
                t4.transform.translation.z = 1.0
                t4.transform.rotation.x = -0.5
                t4.transform.rotation.y = 0.5
                t4.transform.rotation.z = 0.5
                t4.transform.rotation.w = 0.5
                br.sendTransform(t4)

                # do again a lookup transform between 'map' and 'viewPoint' frame
                t5 = tfBuffer.lookup_transform("map", "viewPointCorreted", rclpy.duration.Duration())

                # save mean position
                viewPoint_list.x[id] = ((viewPoint_list.x[id] * counter[id]) + t5.transform.translation.x ) / (counter[id]+1.0)
                viewPoint_list.y[id] = ((viewPoint_list.y[id] * counter[id]) + t5.transform.translation.y ) / (counter[id]+1.0)

                viewPoint_list.w[id] = ((viewPoint_list.w[id] * counter[id]) + t5.transform.rotation.w ) / (counter[id]+1.0)
                viewPoint_list.z[id] = ((viewPoint_list.z[id] * counter[id]) + t5.transform.rotation.z ) / (counter[id]+1.0)

                counter[id] = counter[id] + 1.

            print(["X: ", viewPoint_list.x[id]])
            print(["Y: ", viewPoint_list.y[id]])
            print(["RW: ", viewPoint_list.w[id]])
            print(["RZ: ", viewPoint_list.z[id]])

        except tf2_ros.TransformException:
            # catches all possible tf exceptions
            print("Lookup of ViewPoint Transform failed") 
        
    except tf2_ros.TransformException:
        # catches all possible tf exceptions
        print("Lookup of Marker Transform failed") 

        # for debugging:
    # print(["RW: ", t0.transform.rotation.w])
    # print(["RX: ", t0.transform.rotation.x])
    # print(["RY: ", t0.transform.rotation.y])
    # print(["RZ: ", t0.transform.rotation.z])

def call_ViewPointsSrv_callback(request, response):
    global viewPoint_list, counter

    print('\nIncoming request\n')

    # delete unconvincing View Points
    i = 0
    while i < len(counter):
        if counter[i]<10:
            viewPoint_list.x[i] = 0. 
            viewPoint_list.y[i] = 0. 
 
            viewPoint_list.w[i] = 0. 
            viewPoint_list.z[i] = 0. 
        i=i+1

    response.list = viewPoint_list

    return response
        
def main():
    global node, viewPoint_list, tfBuffer, flag, marker, counter

    rclpy.init()
    node = rclpy.create_node('apriltag_listener')

    # Storage for all 30 ViewPoints with 
    # id x [position.x, position.y, orientation.w, orientation.z]
    # 30 x [.x, .y, .w, .z]
    viewPoint_list = ViewPoints()
    
    flag = False

    marker = AprilTagDetectionArray()
    counter = [0.0]*30           # counter for mean position

    # Create a TF2 buffer which saves the TFs for given cache_time
    tfBuffer = tf2_ros.Buffer()

    # Listener for Transforms:
    tf2_ros.TransformListener(tfBuffer, node)
    
    # Subscriber for apriltag detections
    # apriltag_detection_callback sets flag if april tag was detected
    node.create_subscription(AprilTagDetectionArray, "apriltag/detections",apriltag_detection_callback,10)
    # timer checks if apriltag was detected and slows down the process
    node.create_timer(0.3, timercallback)

    # Service Server for getting die Viewpoint Coordinations
    srv = node.create_service(ViewPointsSrv, 'call_ViewPointsSrv', call_ViewPointsSrv_callback)

    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()