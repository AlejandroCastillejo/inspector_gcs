#!/usr/bin/env python

from flask import Flask, request, jsonify
from flask_cors import CORS
import json

import rospy, rostopic, roslib
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPath
# from inspector_gcs.srv import gcsCreateMission, gcsSendMission, StbyActionService
# from inspector_gcs.srv import StbyActionServiceRequest, PausedStActionServiceRequest
from inspector_gcs.srv import *
from inspector_gcs.msg import UavList

app = Flask(__name__) 
CORS(app)

@app.route("/uav_list")
def uav_list_topic():
    try:
        uav_list = rospy.wait_for_message('/uav_list', UavList, timeout=2)
        print(uav_list)
        response = uav_list.uavs
        print(response)
        return {'uavs': response}
    except Exception, e:
        print(e)
        return str(e)
        
@app.route("/get_topics_list")
def get_topics_list():
    try:
        published_topics = rospy.get_published_topics()
        topics_list = []
        for topic in published_topics:
            topics_list.append(topic[0])
        print('topics: ', topics_list)
        return jsonify(topics_list)
    except Exception, e:
        print(e)
        return str(e)

@app.route("/get_topic")
def get_topic():
    topic_name = request.args.get('topic_name')
    print ('Subscribing to ', topic_name)
    print(rostopic.get_topic_type(topic_name))
    topic_type = rostopic.get_topic_type(topic_name)[0]
    print ('topic type ', topic_type)
    try:
        type_class = roslib.message.get_message_class(topic_type)
        print type_class
        message = rospy.wait_for_message(topic_name, type_class, 5)
        print message
        return jsonify({'last_message': str(message)})
    except Exception,e:
        return str(e)


@app.route("/create_mission_service", methods=['POST'])
def create_mission_service():
    try:
        crete_mission_service = rospy.ServiceProxy('create_mission_service', gcsCreateMission)
        resp = crete_mission_service()
        paths = get_paths_from_ros(resp)
        return {"result": resp.result,
                "paths": paths}
    except Exception, e:
        print(e)
        return str(e)

@app.route("/send_mission_service", methods=['POST'])
def send_mission_service():
    try:
        send_mission_service = rospy.ServiceProxy('send_mission_service', gcsSendMission)
        resp = send_mission_service()
        return {"result": resp.result}
    except Exception, e:
        print(e)
        return str(e)

@app.route("/start_mission", methods=['POST'])
def start_mission():
    return call_action_services(request, 'start_mission service', '/stby_action_service', \
                                StbyActionService, StbyActionServiceRequest.START_NEW_MISSION)

@app.route("/stop_mission", methods=['POST'])
def stop_mission():
    return call_action_services(request, 'stop_mission service', '/stop_service', StopService)

@app.route("/resume_mission", methods=['POST'])
def resume_mission():
    return call_action_services(request, 'resume_mission service', '/paused_state_action_service', \
                                PausedStActionService, PausedStActionServiceRequest.RESUME_PAUSED_MISSION)

@app.route("/abort_mission", methods=['POST'])
def abort_mission():
    return call_action_services(request, 'abort_mission service', '/paused_state_action_service', \
                                PausedStActionService, PausedStActionServiceRequest.START_NEW_MISSION)



# This funtion performs an action by calling ros services for requested UAVs
def call_action_services(request, action_name, service_name, service_class, srv_request=None):
    try:
        uavs = request.values.get('uavs')
        print(uavs)
        if uavs == "all":
            t_uavs = uav_list_topic()['uavs']
        else:
            t_uavs = uavs.split(', ')
        print('t_uavs: ', t_uavs)
    except Exception, e:
        print(e)
        return {'result': str(e)}
    uavs_ok = []
    uavs_error = []
    for uav in t_uavs:
        action_client = rospy.ServiceProxy(uav + service_name, service_class)
        try:
            rospy.loginfo ("calling %s for %s . . .", service_name, uav)
            if srv_request == None:
                resp = action_client()
            else:
                resp = action_client(srv_request)
            uavs_ok.append(uav)
            rospy.loginfo ("'%s%s' called", uav, service_name)
        except rospy.ServiceException:
            rospy.logerr ("'%s%s' not available", uav, service_name)
            uavs_error.append(uav)
        action_client.close()
    return {'result': service_response(action_name, uavs_ok, uavs_error)}


def service_response(name, uavs_ok, uavs_error):
    response = ""
    if uavs_ok:
        response += "Called" + name + "for uavs: " + ', '.join(uavs_ok)
    if uavs_error:
        response += "  \n " + name + " not available for uavs: " + ', '.join(uavs_error)
    return response


def get_paths_from_ros(resp):
    ros_paths = resp.MissionPaths
    paths = []
    for ros_path in ros_paths:
        path = []
        for wp in ros_path.poses:
            path.append({"lat": wp.pose.position.latitude,
                        "lng": wp.pose.position.longitude})
        paths.append(path)
    return paths




## main
def main():
    while(not rospy.is_shutdown()):
        rospy.init_node('flask')
        print('init flask')
        app.debug = True
        app.run(host='127.0.0.1', port=7000)
        rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException, KeyboardInterrupt:
        pass