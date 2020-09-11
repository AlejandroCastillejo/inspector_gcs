#!/usr/bin/env python

import rospy
import rostopic
import roslib
import json
import os.path

# import uav_abstraction_layer.msg
import inspector_gcs.msg
from inspector_gcs.srv import API_MissionFile, API_MissionCommand, API_GetTopic, API_GetTopicsList, StbyActionService, StopService, PausedStActionService
from inspector_gcs.srv import StbyActionServiceRequest, PausedStActionServiceRequest #import defined requests

class RostfulServices:

    def __init__(self):
        rospy.loginfo('rostful_services node running on GCS')
        # paths
        # self.desktop = os.path.expanduser("~/Desktop")
        self.mission_file = os.path.expanduser("~") + '/catkin_ws/src/inspector_gcs/json_files/mission_prueba.json'

        if not os.path.exists(self.mission_file):
            os.mknod(self.mission_file)

        # ROS Subscribers
        rospy.Subscriber('uav_list', inspector_gcs.msg.UavList, self.uav_list_cb)

        # ROS Services
        mission_file_update_srv = rospy.Service('mission_file_update', API_MissionFile, self.mission_file_update_cb)
        start_mission_srv = rospy.Service('start_mission', API_MissionCommand, self.start_mission_cb)
        stop_mission_srv = rospy.Service('stop_mission', API_MissionCommand, self.stop_mission_cb)
        resume_mission_srv = rospy.Service('resume_mission', API_MissionCommand, self.resume_mission_cb)
        abort_mission_srv = rospy.Service('abort_mission', API_MissionCommand, self.abort_mission_cb)
        get_topic_srv = rospy.Service('get_topic', API_GetTopic, self.get_topic_cb)
        get_topics_list_srv = rospy.Service('get_topics_list', API_GetTopicsList, self.get_topics_list_cb)
        

    # Services callbacks
    def get_topic_cb(self,req):
        print ('Subscribing to ', req.topic_name)
        topic_name = req.topic_name
        topic_type = rostopic.get_topic_type(req.topic_name)[0]
        print ('topic type ', topic_type)
        if topic_type:
            type_class = roslib.message.get_message_class(topic_type)
            print type_class
            message = rospy.wait_for_message(topic_name, topic_type, 5)
            return str(message.data)
        else:
            return "ERROR: topic unavailable"

    def get_topics_list_cb(self,req):
        published_topics = rospy.get_published_topics()
        topics_list = []
        for topic in published_topics:
            topics_list.append(topic[0])
        return str(topics_list)
    
    def mission_file_update_cb(self, req):
        with open(self.mission_file, 'w') as f:
            message = json.loads(req.file_content)
            json.dump(message, f)
        return "True"

    def start_mission_cb(self, req):
        print('req: ', req)
        uavs = json.loads(req.uavs)
        if uavs == "all":
            t_uavs = self.uav_list
        else:
            t_uavs = uavs.split(', ')
        print('uavs: ', t_uavs)
        uavs_ok = []
        uavs_error = []
        for uav in t_uavs:
            stby_action_client = rospy.ServiceProxy(uav + '/stby_action_service', StbyActionService)
            try:
                rospy.loginfo ("trying to call '/staby_action_service' for %s . . .", uav)
                resp = stby_action_client(StbyActionServiceRequest.START_NEW_MISSION)
                uavs_ok.append(uav)
                rospy.loginfo ("'%s/staby_action_service' called", uav)
            except rospy.ServiceException:
                rospy.logerr ("'%s/staby_action_service' not available", uav)
                uavs_error.append(uav)
            stby_action_client.close()
        return (self.service_response("start_mission service", uavs_ok, uavs_error))

    def stop_mission_cb(self, req):
        uavs = json.loads(req.uavs)
        if uavs == "all":
            t_uavs = self.uav_list
        else:
            t_uavs = uavs.split(', ')
        uavs_ok = []
        uavs_error = []
        for uav in t_uavs:
            stop_client = rospy.ServiceProxy(uav + '/stop_service', StopService)
            try:
                rospy.loginfo ("trying to call '/stop_service' for %s . . .", uav)
                resp = stop_client()
                uavs_ok.append(uav)
                rospy.loginfo ("'%s/stop_service' called", uav)
            except rospy.ServiceException:
                rospy.logerr ("'%s/stop_service' not available", uav)
                uavs_error.append(uav)
            stop_client.close()
        return (self.service_response("stop_mission service", uavs_ok, uavs_error))

    def resume_mission_cb(self, req):
        uavs = json.loads(req.uavs)
        if uavs == "all":
            t_uavs = self.uav_list
        else:
            t_uavs = uavs.split(', ')
        uavs_ok = []
        uavs_error = []
        for uav in t_uavs:
            paused_state_action_client = rospy.ServiceProxy(uav + '/paused_state_action_service', PausedStActionService)
            try:
                rospy.loginfo ("trying to call '/paused_state_action_service' for %s . . .", uav)
                resp = paused_state_action_client(PausedStActionServiceRequest.RESUME_PAUSED_MISSION)
                uavs_ok.append(uav)
                rospy.loginfo ("'%s/paused_state_action_service' called", uav)
            except rospy.ServiceException:
                rospy.logerr ("'%s/paused_state_action_service' not available", uav)
                uavs_error.append(uav)
            paused_state_action_client.close()
        return (self.service_response("resume_mission", uavs_ok, uavs_error))

    def abort_mission_cb(self, req):
        uavs = json.loads(req.uavs)
        if uavs == "all":
            t_uavs = self.uav_list
        else:
            t_uavs = uavs.split(', ')
        uavs_ok = []
        uavs_error = []
        for uav in t_uavs:
            paused_state_action_client = rospy.ServiceProxy(uav + '/paused_state_action_service', PausedStActionService)
            try:
                rospy.loginfo ("trying to call '/paused_state_action_service' for %s . . .", uav)
                resp = paused_state_action_client(PausedStActionServiceRequest.START_NEW_MISSION)
                uavs_ok.append(uav)
                rospy.loginfo ("'%s/paused_state_action_service' called", uav)
            except rospy.ServiceException:
                rospy.logerr ("'%s/paused_state_action_service' not available", uav)
                uavs_error.append(uav)
            paused_state_action_client.close()            
        return (self.service_response("abort_mission", uavs_ok, uavs_error))

    def service_response(self, _name, _uavs_ok, _uavs_error):
        response = ""
        if _uavs_ok:
            response += "Called" + _name + "for uavs: \n" + json.dumps(_uavs_ok)
        if _uavs_error:
            response += "\nService not available for uavs: \n" + json.dumps(_uavs_error)
        return response


    # Subscribers callbacks
    def uav_list_cb(self, data):
        self.uav_list = data.uavs


def main():
    rospy.init_node('RostfulServices')
    rostful_services = RostfulServices()
    rospy.loginfo('ROStful services running')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
