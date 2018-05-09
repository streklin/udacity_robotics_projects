#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from sensor_msgs.msg import JointState
from rospy_message_converter import message_converter
import yaml

# constants
PLACE_RED_X = 0
PLACE_RED_Y = 0.71
PLACE_RED_Z = 0.605

PLACE_GREEN_X = 0
PLACE_GREEN_Y = -0.71
PLACE_GREEN_Z = 0.605

#state constants
STATE_LEFT = "left" # PR2 needs to turn left
STATE_READ_LEFT = "read_left" # pr2 needs to update collision data for the left side
STATE_RIGHT = "right" # PR2 needs to turn to the right
STATE_READ_RIGHT = "read_right" # pr2 needs to update collision data for the right side
STATE_CENTER = "center" # pr2 needs to return to the center position
STATE_PERCEPTION = "perception" # pr2 should run the perception pipeline / pick and place routines

# Represent the state transitions for the PICK AND PLACE algorithms
class PR2StateMachine:
    state = ""

    def __init__(self):
        self.state = STATE_LEFT
        #self.state = STATE_PERCEPTION

    def nextState(self):
        if self.state == STATE_LEFT:
            self.state = STATE_READ_LEFT
        elif self.state == STATE_READ_LEFT:
            self.state = STATE_RIGHT
        elif self.state == STATE_RIGHT:
            self.state = STATE_READ_RIGHT
        elif self.state == STATE_READ_RIGHT:
            self.state = STATE_CENTER
        elif self.state == STATE_CENTER:
            self.state = STATE_PERCEPTION
        else:
            self.state = STATE_LEFT

        return self.state

    def getState(self):
        return self.state


# Represent the collision point cloud for the PR2, used to cache the collision point cloud
# for the left and right tables.
class PR2CollisionPointCloud:

    def __init__(self):
        self.collision_left = pcl.PointCloud()
        self.collision_right = pcl.PointCloud()
        self.collision_list = []

    def setLeftPointCloud(self, points):
        self.collision_left = points

    def setRightPointCloud(self, points):
        self.collision_right = points

    def getCollisionClouds(self):
        if len(self.collision_list) > 0:
            return self.collision_list

        left_list = pointCloudToList(self.collision_left)
        right_list = pointCloudToList(self.collision_right)
        left_list.extend(right_list)

        self.collision_list = left_list

        return self.collision_list

# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster


# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"] = arm_name.data
    yaml_dict["object_name"] = object_name.data
    # yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    # yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)


    # yaml library is complaining about output from message_converter helper functions, so I am doing
    # this conversion manually.
    yaml_dict["pick_pose"] = {
        'position': {
            'x': float(pick_pose.position.x),
            'y': float(pick_pose.position.y),
            'z': float(pick_pose.position.z)
        },
        'orientation': {
            'x': float(pick_pose.orientation.x),
            'y': float(pick_pose.orientation.y),
            'z': float(pick_pose.orientation.z)
        }
    }

    yaml_dict["place_pose"] = {
        'position': {
            'x': float(place_pose.position.x),
            'y': float(place_pose.position.y),
            'z': float(place_pose.position.z)
        },
        'orientation': {
            'x': float(place_pose.orientation.x),
            'y': float(place_pose.orientation.y),
            'z': float(place_pose.orientation.z)
        }
    }

    return yaml_dict


# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)


# downsample the point cloud into voxels to save processing time.
def voxel_grid_downsample(pointCloud):
    # voxel size
    LEAF_SIZE = 0.01

    # configure filter
    vox = pointCloud.make_voxel_grid_filter()
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

    return vox.filter()


# filter out parts of the image outside the table.
def passthrough_filter(voxels):
    # pass through settings
    filter_axis = 'z'
    axis_min = 0.6
    axis_max = 1.1

    # configure filter
    passthrough = voxels.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)

    return passthrough.filter()


# segement the point cloud data using ransac
def ransac_segmentation(voxels):
    max_distance = 0.035
    seg = voxels.make_segmenter()

    # Set the model you wish to fit
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(max_distance)

    return seg.segment()


# extract object clusters from a white point cloud
def extract_clusters(white_cloud):
    # create kd-tree
    tree = white_cloud.make_kdtree()

    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()

    # Set tolerances for distance threshold
    # as well as minimum and maximum cluster size (in points)
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(2000)

    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)

    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    return cluster_indices


# color code point cloud cluster
def create_color_cluster(cluster_indices, white_cloud):
    # Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    # Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    return cluster_cloud


# clean up noise in the camera data
def statistical_outlier_filter(point_cloud):
    # Much like the previous filters, we start by creating a filter object:
    outlier_filter = point_cloud.make_statistical_outlier_filter()

    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(50)

    # Set threshold scale factor
    x = 1.0

    # Any point with a mean distance larger than global (mean distance+x*std_dev) will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(x)

    # Finally call the filter function for magic
    cloud_filtered = outlier_filter.filter()

    return cloud_filtered


# performs object detection on clustered data
def do_object_detection(cluster_indices, cloud_objects, white_cloud):
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = cloud_objects.extract(pts_list)
        # convert the cluster from pcl to ROS using helper function
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Extract histogram features
        chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4

        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    # Publish the list of detected objects
    # This is the output you'll need to complete the upcoming project!
    detected_objects_pub.publish(detected_objects)

    return detected_objects_labels, detected_objects


# calculate the centroid of an object
def find_centroids(detected_objects):
    labels = []
    centroids = []  # to be list of tuples (x, y, z)

    for object in detected_objects:
        labels.append(object.label)
        points_arr = ros_to_pcl(object.cloud).to_array()
        centroids.append(np.mean(points_arr, axis=0)[:3])

    return labels, centroids

# search the list of identified labels for a target label
def find_object_index(target_label, label_list):
    for i in range(len(label_list)):
        if label_list[i] == target_label:
            return i

    return -1  # the object wasn't identified

# check if the joint position is within tolerance of the target position
def at_goal(position, goal):
    tolerance = .05
    result = abs(position - goal) <= abs(tolerance)
    return result

# moves the robots torso, blocks other execution while doing so
def move_torso(theta):
    print "Sending Request"

    world_joint_pub.publish(theta)

    while True:
        joint_state = rospy.wait_for_message('/pr2/joint_states', JointState)
        num_joints = len(joint_state.position)
        world_joint_position = joint_state.position[num_joints - 1]

        if at_goal(world_joint_position, theta):
            break

    print "WORK COMPLETE"

# converts a point cloud to a list, used to assist in combining multiple point
# clouds together.
def pointCloudToList(pointCloud):
    points_list = []

    for data in pointCloud:
        points_list.append([data[0], data[1], data[2]])

    return points_list

# entry point for the PR2 robot's pick and place routine.  Depends on the state machine
# to determine the next behaviour of the PR2
def pcl_callback(pcl_msg):

    # Convert ROS msg to PCL data
    point_cloud = ros_to_pcl(pcl_msg)

    current_state = state_machine.getState()

    if current_state == STATE_PERCEPTION:
        detected_objects, object_table = perception_pipeline(pcl_msg)
        pick_and_place(detected_objects, object_table)
        return

    if current_state == STATE_LEFT:
        move_torso(np.pi / 2)
        state_machine.nextState()
        return

    if current_state == STATE_READ_LEFT:
        collision_cloud.setLeftPointCloud(point_cloud)
        state_machine.nextState()
        return

    if current_state == STATE_RIGHT:
        move_torso(-np.pi / 2)
        state_machine.nextState()
        return

    if current_state == STATE_READ_RIGHT:
        collision_cloud.setRightPointCloud(point_cloud)
        state_machine.nextState()
        return

    if current_state == STATE_CENTER:
        move_torso(0)
        state_machine.nextState()
        return

def pick_and_place(detected_objects, object_table):
    # perform pick and place operations
    try:
        pr2_mover(detected_objects, object_table)
    except rospy.ROSInterruptException:
        pass

# Perception pipeline
def perception_pipeline(pcl_msg):
    print("Starting Perception Pipeline")

    # Convert ROS msg to PCL data
    point_cloud = ros_to_pcl(pcl_msg)

    # Statistical Outlier Filtering (clean up the noise)
    point_cloud = statistical_outlier_filter(point_cloud)

    # Voxel Grid Downsampling, reduces processing requirements
    point_cloud = voxel_grid_downsample(point_cloud)

    # PassThrough Filter, cleans out the extraneous data
    point_cloud = passthrough_filter(point_cloud)

    # RANSAC Plane Segmentation
    inliers, coefficient = ransac_segmentation(point_cloud)

    # Extract the table
    extracted_inliers = point_cloud.extract(inliers, negative=False)  # table voxels

    # extract the objects
    extracted_outliers = point_cloud.extract(inliers, negative=True)  # object voxels

    # Euclidean Clustering (segment the voxels into their individual objects)
    white_cloud = XYZRGB_to_XYZ(extracted_outliers)
    clusters = extract_clusters(white_cloud)

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_cloud = create_color_cluster(clusters, white_cloud)

    # Convert PCL data to ROS messages
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    object_cloud_pub.publish(ros_cluster_cloud)

    # predict what the objects are
    detected_objects_labels, detected_objects = do_object_detection(cluster_indices=clusters,
                                                                    cloud_objects=extracted_outliers,
                                                                    white_cloud=white_cloud)

    return detected_objects, extracted_inliers


# send the collision maps for all objects BUT the target object
def create_collision_list_from_objects(object_list, index):

    collision_list = []

    for i in range(len(object_list)):
        if i == index:
            continue

        point_cloud = object_list[index].cloud
        point_list = pointCloudToList(point_cloud)
        collision_list.extend(point_list)

    return collision_list

# function to load parameters and request PickPlace service
def pr2_mover(object_list, object_table):

    print("Starting Pick and Place")
    # set the test scene number
    TEST_SCENE_NUM = Int32()
    TEST_SCENE_NUM.data = 1

    # get the object centroids
    object_labels, object_centroids = find_centroids(object_list)

    # Get/Read parameters
    object_list_param = rospy.get_param('/object_list')

    # loop though the object list
    dict_list = []

    for i in range(len(object_list_param)):
        # Parse parameters into individual variables
        object_name = String()
        object_name.data = object_list_param[i]['name']
        object_group = object_list_param[i]['group']

        # Get the PointCloud for a given object and obtain it's centroid
        index = find_object_index(object_list_param[i]['name'], object_labels)

        # -1 means we never found this object
        if index == -1:
            continue

        centroid = object_centroids[index]

        arm = String()

        # Assign the arm to be used for pick_place
        if object_group == "green":
            arm.data = "right"
        else:
            arm.data = "left"

        # Create 'pick_pose' for the object
        pick_pose = Pose()

        pick_pose.position.x = centroid[0]
        pick_pose.position.y = centroid[1]
        pick_pose.position.z = centroid[2] + 0.01

        # no information in yaml file, or course info, so I am assuming zero
        pick_pose.orientation.x = 0
        pick_pose.orientation.y = 0
        pick_pose.orientation.z = 0

        # create 'place_pose' for the object
        place_pose = Pose()

        # no information in yaml file, or course info, so I am assuming zero
        place_pose.orientation.x = 0
        place_pose.orientation.y = 0
        place_pose.orientation.z = 0

        # set position based on the target box
        if object_group == "green":
            place_pose.position.x = PLACE_GREEN_X
            place_pose.position.y = PLACE_GREEN_Y
            place_pose.position.z = PLACE_GREEN_Z
        else:
            place_pose.position.x = PLACE_RED_X
            place_pose.position.y = PLACE_RED_Y
            place_pose.position.z = PLACE_RED_Z

        yaml_dict = make_yaml_dict(TEST_SCENE_NUM, arm, object_name, pick_pose, place_pose)
        dict_list.append(yaml_dict)

        # send the full collision point cloud
        collision_map = []
        collision_list = collision_cloud.getCollisionClouds()
        table_list = pointCloudToList(object_table)
        object_collision_list = create_collision_list_from_objects(object_list, index)

        collision_map.extend(table_list)
        collision_map.extend(collision_list)
        collision_map.extend(object_collision_list)

        collision_map_pcl = pcl.PointCloud()
        collision_map_pcl.from_list(collision_map)

        color_list = get_color_list(2)

        collision_map_pcl = XYZ_to_XYZRGB(collision_map_pcl, color_list[0])
        collision_map_ros = pcl_to_ros(collision_map_pcl)
        sensor_msg_pub.publish(collision_map_ros)

        # # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        # Call the pick_place service
        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)
            resp = pick_place_routine(TEST_SCENE_NUM, object_name, arm, pick_pose, place_pose)
            print ("Response: ", resp.success)
            #print ("HELLO WORLD")
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    # generate yaml file, required for project
    print ("Writing Output ...")
    send_to_yaml("picklist_2.yaml", dict_list)


if __name__ == '__main__':
    # ROS node initialization
    rospy.init_node('perception_pipeline', anonymous=True)

    # listen for input from the PR2 RGBD Camera
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # Create Publishers

    # publish segmented object voxels
    object_cloud_pub = rospy.Publisher("/object_cloud", PointCloud2, queue_size=1)

    # publish markers above identified objects
    object_markers_pub = rospy.Publisher('/object_markers', Marker, queue_size=1)

    # publish the list of detected objects
    detected_objects_pub = rospy.Publisher('/detected_objects', DetectedObjectsArray, queue_size=1)

    # publish sensor messages for collision detection
    sensor_msg_pub = rospy.Publisher('/pr2/3d_map/points', PointCloud2, queue_size=1)

    # publisher for turning the PR2 body
    world_joint_pub = rospy.Publisher('/pr2/world_joint_controller/command', Float64, queue_size=1)

    # Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Initialize the state machine
    state_machine = PR2StateMachine()

    # init collision point cloud for left / right tables
    collision_cloud = PR2CollisionPointCloud()

    print ("---PERCEPTION PIPELINE STARTED---")

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
