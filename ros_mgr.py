import rospy
import thread
import time
from math import sqrt
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped, Twist, PoseArray, Quaternion
from tf.transformations import quaternion_from_euler

def rosThread(*kw):
    while not rospy.is_shutdown():
        rospy.spin_once()
def targetCB(data):
    global tgt_reached
    tgt_reached = True
def wfc(): #wait for completion
    while not tgt_reached:
        time.sleep(0.1)
def moveTo(x, y, yaw, frame):
    #[x,y,z,w] = quaternion_from_euler(0, 0, yaw)
    #quaternion = Quaternion(x,y,z,w) # uses rpy convention..
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x = x
    pose.pose.position.y = y
    #pose.pose.orientation = quaternion
    linPub.publish(pose)
    global tgt_reached
    tgt_reached = False
def rotate(yaw, frame):
    [x,y,z,w] = quaternion_from_euler(0, 0, yaw)
    quaternion = Quaternion(x,y,z,w)
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.orientation = quaternion
    rotPub.publish(pose)
    global tgt_reached 
    tgt_reached = False
def setSpeed(x,y,theta):
    msg = Twist()
    msg.linear.x = x
    msg.linear.y = y
    msg.angular.z = theta
    velPub.publish(msg)
def parsePose(point):
    pose = Pose()
    pose.position.x = point[0]
    pose.position.y=point[1]
    [x,y,z,w] = quaternion_from_euler(0, 0, point[2])
    pose.orientation=Quaternion(x,y,z,w)
    return pose
def trajectory(points, frame):
    t = PoseArray()
    t.header.frame_id=frame
    t.poses = []
    for p in points:
        t.poses.append(parsePose(p))
    trajPub.publish(t)
    global tgt_reached
    tgt_reached = False
def pathplan(tgt):
    point = parsePose(tgt)
    tgt = PoseStamped()
    tgt.header.frame_id = "map"
    tgt.pose = point
    pathPub.publish(tgt)
    global tgt_reached
    tgt_reached = False
def trajectoryplan(points):
    msg = PoseArray()
    msg.header.frame_id="map"
    msg.poses = []
    for p in points:
        msg.poses.append(parsePose(p))
    ptPub.publish(msg)
    global tgt_reached
    tgt_reached = False
def restrict(res, data):
    d = len(data)
    d = int(round(sqrt(d)))
    msg = OccupancyGrid()
    msg.header.frame_id="map"
    msg.info.resolution=res
    msg.info.width = d
    msg.info.height = d
    msg.info.origin.position.x = -12.825
    msg.info.origin.position.y = -12.825
    msg.data = data
    restPub.publish(msg)
def init():
    global rotPub
    global linPub
    global linRotPub
    global trajPub
    global pathPub
    global ptPub
    global abortPub
    global velPub
    global tgt_reached
    global restPub
    tgt_reached = False
    rospy.init_node("plan_parser")
    rotPub = rospy.Publisher("/direct_move/rot_target", PoseStamped, queue_size=2)
    linPub = rospy.Publisher("/direct_move/lin_target", PoseStamped, queue_size=2)
    linRotPub = rospy.Publisher("/direct_move/lin_rot_target", PoseStamped, queue_size=2)
    trajPub = rospy.Publisher("/direct_move/trajectory", PoseArray, queue_size=2)
    velPub = rospy.Publisher("/cmd_vel", Twist, queue_size=2)
    abortPub = rospy.Publisher("/direct_move/abort", Bool, queue_size=2)
    pathPub = rospy.Publisher("/local_planner/target", PoseStamped, queue_size=1)
    ptPub = rospy.Publisher("/local_planner/trajectory", PoseArray, queue_size=1)
    rospy.Subscriber("/direct_move/reached_target", Bool, targetCB)
    restPub = rospy.Publisher("/restricted", OccupancyGrid, queue_size=1)
#    thread.start_new_thread( rosThread, (None, None))


