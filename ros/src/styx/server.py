#!/usr/bin/env python

import eventlet
eventlet.monkey_patch(socket=True, select=True, time=True)

import eventlet.wsgi
import socketio
import time
from flask import Flask, render_template

from bridge import Bridge
from conf import conf

import rospy
import thread

sio = socketio.Server()
app = Flask(__name__)
msgs = []
telemetry_current_data = None
image_current_data = None

dbw_enable = False

@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)

def send(topic, data):
    s = 1
    msgs.append((topic, data))
    #sio.emit(topic, data=json.dumps(data), skip_sid=True)

bridge = Bridge(conf, send)

def update_odometry():
    global telemetry_current_data
    if telemetry_current_data != None:
        bridge.publish_odometry(telemetry_current_data)
        telemetry_current_data = None
    else:
        pass

def thread_update_odometry():
    rospy.loginfo("[server.py] Start pose update thread")
    odometry_update_HZ = 20
    odometry_update_rate = rospy.Rate(odometry_update_HZ) 
    while not rospy.is_shutdown():
        update_odometry()
        odometry_update_rate.sleep()

def update_image():
    global image_current_data
    if image_current_data != None:
        bridge.publish_camera(image_current_data)
        image_current_data = None
    else:
        pass

def thread_update_image():
    rospy.loginfo("[server.py] Start image update thread")
    image_update_HZ = 10
    image_update_rate = rospy.Rate(image_update_HZ) 
    while not rospy.is_shutdown():
        update_image()
        image_update_rate.sleep()


@sio.on('telemetry')
def telemetry(sid, data):
    global dbw_enable
    global telemetry_current_data
    if data["dbw_enable"] != dbw_enable:
        dbw_enable = data["dbw_enable"]
        bridge.publish_dbw_status(dbw_enable)
    #bridge.publish_odometry(data)
    
    telemetry_current_data = data
    for i in range(len(msgs)):
        topic, data = msgs.pop(0)
        sio.emit(topic, data=data, skip_sid=True)

@sio.on('control')
def control(sid, data):
    bridge.publish_controls(data)

@sio.on('obstacle')
def obstacle(sid, data):
    bridge.publish_obstacles(data)

@sio.on('lidar')
def obstacle(sid, data):
    bridge.publish_lidar(data)

@sio.on('trafficlights')
def trafficlights(sid, data):
    bridge.publish_traffic(data)

@sio.on('image')
def image(sid, data):
    global image_current_data
    image_current_data = data
    #bridge.publish_camera(data)

if __name__ == '__main__':
    thread.start_new_thread( thread_update_odometry, ())
    thread.start_new_thread( thread_update_image, ())

    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)

    
