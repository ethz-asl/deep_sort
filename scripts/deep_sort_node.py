#!/usr/bin/env python
import os
import numpy as np

import rospy
from sensor_msgs.msg import RegionOfInterest
from deep_sort_ros.srv import *

import deep_sort_ros.nn_matching as nn
from deep_sort_ros.detection import Detection
from deep_sort_ros.tracker import Tracker

max_cosine_distance = 0.2
nn_budget = 100

class DeepSortNode(object):
    def __init__(self):
        self._metric = nn.NearestNeighborDistanceMetric(
            "cosine", max_cosine_distance, nn_budget)
        self._tracker = Tracker(self._metric)
        
    def run(self):
        # inference service handler
        s = rospy.Service('generate_semantic_track_ids',
                deep_sort_result, self._handle_generate_track_id)

        while not rospy.is_shutdown():
            rospy.spin()

    def _handle_generate_track_id(self, req):
        print("in handle generate track id")
        detections = []
        print("# of boxes: ",len(req.boxes))
        for i in range(len(req.boxes)):
            box = req.boxes[i]
            confidence = req.confidences[i]
            
            descriptor = req.descriptors[i].descriptor
        
            detections.append(Detection([box.x_offset, box.y_offset, box.width, box.height],
                                             confidence,
                                             descriptor))
        # Update tracker.
        self._tracker.predict()
        track_ids = self._tracker.update(detections)

        # Create Response
        resp = deep_sort_resultResponse()
        resp.track_ids = track_ids
        return resp

def main():
    rospy.init_node('deep_sort')

    node = DeepSortNode()
    node.run()


if __name__ == '__main__':
    main()