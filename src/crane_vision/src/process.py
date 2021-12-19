#!/usr/bin/env python
import sys
import rospy
import struct
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from stereo_msgs.msg import DisparityImage
from std_msgs.msg import Header, String
from geometry_msgs.msg import Pose
import message_filters
import cv2
import matplotlib.pyplot as plt
import numpy as np
import queue
import collections
# import threading

from crane_model_v02.msg import ObjectIdentifier, ObjectList

from camera_projector import CameraProjector
from ekf_tracker import EKFTracker

class ImageProcessor(object):

    def __init__(self):
        self._bridge = CvBridge()
        self._done = 0
        self._approach_elevation = 1.0

        self._projector = CameraProjector()
        self._detected_points = np.array([])
        self._particle_filters = None
        self._particle_states = {}
        self._particle_state_times = {}

        # Fill in
        self._completed_objects = []

        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks=50)   # or pass empty dictionary

        self._matcher = cv2.FlannBasedMatcher(index_params,search_params)
        # 100 frames ~ 2 seconds
        self._tracker_history_maxlen = 1000+10
        self._y_const = 0.15

        self._state_test_pos = []
        self._state_test_queue = collections.deque(maxlen=self._tracker_history_maxlen)

        # constants
        self._thr_HSV_low = (69, 0, 24)
        self._thr_HSV_high = (180, 255, 255)

        # subscribers
        self._sub_image_left = message_filters.Subscriber("/image/left/rgb", Image)
        self._sub_depth_left = message_filters.Subscriber("/image/left/depth", Image)
        self._sub_man_status = message_filters.Subscriber("/manager/status", String)
        self._sub_man_status.registerCallback(self.callback_publish_designated_object)

        self._sub_rgbd_left = message_filters.ApproximateTimeSynchronizer([self._sub_image_left, self._sub_depth_left], queue_size=100, slop=1)
        self._sub_rgbd_left.registerCallback(self.callback_rgbd_image)

        self._sub_pos_rc = rospy.Subscriber("/sphere/position", Pose, self.callback_test_pos)

        # Inactivated - one camera only for now
        # self._sub_image_right = message_filters.Subscriber("/image/right/rgb", Image)
        # self._sub_depth_right = message_filters.Subscriber("/image/right/depth", Image)
        # self._sub_rgbd_right = message_filters.ApproximateTimeSynchronizer([self._sub_image_right, self._sub_depth_right], queue_size=100, slop=1)

        # publishers
        self._pub_contours = rospy.Publisher("/image/left/objects", Image, queue_size=10)
        self._pub_depth_pass = rospy.Publisher("/image/left/depth_pass", Image, queue_size=10)
        self._pub_detected_objects = rospy.Publisher("/object_data/states", ObjectList, queue_size=10)
        self._pub_designated_objects = rospy.Publisher("/object_data/current_target", ObjectIdentifier, queue_size=10)


    def callback_test_pos(self, pose):
        self._state_test_pos = np.array([pose.position.x, pose.position.y, pose.position.z])

    def callback_rgbd_image(self, image, depth):
        image = self._bridge.imgmsg_to_cv2(image, "rgb8")
        depth = self._bridge.imgmsg_to_cv2(depth, "32FC1")
        # Mirror image across vertical (y) axis
        image = cv2.flip(image, 1)
        depth = cv2.flip(depth, 1)
        # Get objects and locations
        # Sort this
        image_seg, rcentroids, rindices, regions, bb_coords, bb_dims = self.segment_image_objects(image, limit_low=self._thr_HSV_low, limit_high=self._thr_HSV_high)
        centroids = self.get_object_centroids(image_seg)

        # import pdb; pdb.set_trace()

        # Translate to real coords
        centroids_world = []
        for c in centroids:
            U = np.asarray([c[0], c[1], 1]).reshape((3,1))
            # Run deprojection
            X = self._projector.deproject(U)
            X = self._projector.transform_camera_to_world_axes(X)
            X = self._projector.transform_coords_swap(X)
            centroids_world.append(np.float32(X.flatten()))
        centroids_world = np.array(centroids_world)

        # Update with depth image measurements
        # Note: have to add constant manually as changing order changes transform value
        for i in range(1, centroids_world.shape[0]+1):
            print("Depth value: ", depth[regions == i].mean())
            depth_val = self.extract_depth(depth, regions, i)
            # except IndexError
            centroids_world[i-1][0] = depth_val + self._y_const

        if 0 in self._particle_states and len(self._particle_states[0]) >= 1000:
            import pdb; pdb.set_trace()
        # First time detection - initialise object list, object states
        if not self._detected_points.size:
            # self._particle_filters = np.array([EKFTracker(3, 3) for x in centroids_world.shape[0]])
            self._particle_filters = dict(zip([x for x in range(0, centroids_world.shape[0])], [EKFTracker(3, 3) for x in range(0,centroids_world.shape[0])]))
            self._particle_states = dict(zip([x for x in range(0, centroids_world.shape[0])], [collections.deque(maxlen=self._tracker_history_maxlen) for x in range(0, centroids_world.shape[0])]))
            self._particle_state_times = dict(zip([x for x in range(0, centroids_world.shape[0])], [collections.deque(maxlen=self._tracker_history_maxlen) for x in range(0, centroids_world.shape[0])]))
            for i in self._particle_states.keys():
                self._particle_states[i].appendleft(centroids_world[i])
                self._particle_state_times[i].appendleft(rospy.get_time())
                self._state_test_queue.appendleft(self._state_test_pos)
        # Subsequent detection - Update prior detection
        else:
            # Match detected objects
            matches = self._matcher.knnMatch(centroids_world, self._detected_points, k=self._detected_points.shape[0])
            # Choose estimators to update
            for match in matches:
                target = match[0].trainIdx
                if target >= 0 and target < len(self._particle_filters):
                    estimate = self._particle_filters[target].step(centroids_world[target])
                    self._particle_states[target].appendleft(estimate)
                    self._particle_state_times[target].appendleft(rospy.get_time())
                    self._state_test_queue.appendleft(self._state_test_pos)
                    # if len(self._particle_states[target]) > self._tracker_history_maxlen:
                    #     self._particle_states[target].pop()
                else:
                    print("match " + str(match) + "out of index")
                    pass
        self._detected_points = centroids_world
        # print(centroids_world)

        image_seg = np.uint8(image_seg)
        image_seg = cv2.cvtColor(image_seg, cv2.COLOR_GRAY2RGB)
        msg = self._bridge.cv2_to_imgmsg(image_seg, "rgb8")
        self._pub_contours.publish(msg)

        # if self._detected_points.size and self._done == 0:
        #     i = self.select_object_to_target()
        #     self.publish_designated_object(i, approach_elevation=self._approach_elevation)
        #     self._done = 1


    def segment_image_objects(self, image, limit_low=(0,0,0), limit_high=(180,255,255), min_size=750):
        image_HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        image_thresholded = cv2.inRange(image_HSV, limit_low, limit_high)

        nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(image_thresholded, connectivity=8)
        sizes = stats[1:, -1];
        bb_left = stats[1:, 0];
        bb_top = stats[1:, 1];
        bb_width = stats[1:, 2];
        bb_height = stats[1:, 3];

        nb_components = nb_components - 1
        image_filtered = np.zeros((output.shape))
        centroids_filtered = []
        bb_coords_filtered = []
        bb_dims_filtered = []
        centroids_indices = []
        for i in range(0, nb_components):
            if sizes[i] >= min_size:
                image_filtered[output == i + 1] = 255
                centroids_filtered.append(centroids[i])
                bb_coords_filtered.append((bb_left[i], bb_top[i]))
                bb_dims_filtered.append((bb_width[i], bb_height[i]))
                centroids_indices.append(i+1)


        kernel = np.ones((10,10), np.uint8)
        close_kernel = np.ones((3,3), np.uint8)
        image_filtered = cv2.dilate(image_filtered, kernel)
        image_filtered = cv2.erode(image_filtered, close_kernel)
        # image_filtered = cv2.morphologyEx(image_filtered, cv2.MORPH_CLOSE, close_kernel)
        image_filtered = np.uint8(image_filtered)

        return image_filtered, centroids_filtered, centroids_indices, output, bb_coords_filtered, bb_dims_filtered

    def extract_depth(self, depth, regions, index):
        # print(depth[regions == index])
        d_value = depth[regions == index].mean()
        return self.convert_depth_liend_to_m(d_value)

    def get_object_centroids(self, mask):
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        centroids = []
        for c in contours:
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            centroids.append(np.asarray((cX, cY)))
        return centroids

    def localise_objects(self, image):
        pass

    def convert_depth_liend_to_m(self, value):
        # return value/10 + 0.125 + 6.675e-5*(value/10-1.55)
        return value/10 + 0.08 + 0.00034192*value/10

    def callback_publish_objectids(self):
        o_list = ObjectList()
        for id in keys(self._particle_states):
            point = self._particle_states[id]
            oid = ObjectIdentifier()
            oid.index = id
            oid.status = ""
            oid.coords = [point[0], point[1], point[2]]
            o_list.objects.append(oid)

        o_list.header.stamp = rospy.get_rostime()
        o_list.header.frame_id = "From real-time state data"
        self._pub_detected_objects.publish(o_list)

    def select_object_to_target(self):
        '''
        Order objects left to right (sort?), select leftmost object
        Dict: {oid: x coord}
        id, object in self._particle_states.items()
        '''
        target = -1
        try:
            if self._particle_states:
                object_x_coords = {k: v[0] for k, v in self._particle_states.items()}
                # select leftmost object by y coord
                object_x_coords = sorted(list(object_x_coords.items()), key=lambda item: item[1][1], reverse=True)
                object_numbers = [item[0] for item in object_x_coords if item not in self._completed_objects]
                target = object_numbers[0] if object_numbers else -1
                # if already picked, select corresponding next object
                self._completed_objects.append(target)
        except BaseException as err:
            print(f"Error while parsing particle_states: Unexpected {err=}, {type(err)=}")
            pass

        return target

    def callback_publish_designated_object(self, message):
        status = message.data
        if status == "ready" and self._particle_states:
            i = self.select_object_to_target()
            if i >= 0:
                self.publish_designated_object(i)
            else:
                print(f"No objects to publish")
            # add handshake here
        else:
            pass

    def publish_designated_object(self, id, approach_elevation=0.5):
        pointq = self._particle_states[id]
        point = pointq[0]
        oid = ObjectIdentifier()
        oid.index = id
        oid.status = "retrieve"
        oid.elevation = approach_elevation
        oid.coords = [point[0], point[1], point[2]]
        print("Coords: ", oid.coords)

        self._pub_designated_objects.publish(oid)

if __name__ == "__main__":
    rospy.init_node("image_processor")
    p = ImageProcessor()
    rospy.spin()
