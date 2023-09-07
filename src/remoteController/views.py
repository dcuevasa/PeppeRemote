# -*- coding: utf-8 -*-
from __future__ import unicode_literals
import rospy
from django.shortcuts import render, HttpResponse
from robot_toolkit_msgs.msg import speech_msg, audio_tools_msg, depth_to_laser_msg
from robot_toolkit_msgs.srv import audio_tools_srv, navigation_tools_srv, tablet_service_srv
from geometry_msgs.msg import Twist
from django.shortcuts import render_to_response
from django.template import RequestContext
from django.http import HttpResponseRedirect
from django.core.urlresolvers import reverse

from remoteController.models import Document
from remoteController.forms import DocumentForm

speed = 0.3
linearX=0
linearY=0
angular=0
class RemoteC:
    def __init__(self):
        self.speechPublisher = rospy.Publisher('/speech',speech_msg,queue_size=10)
        self.movePublisher = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

        #Service speech call
        self.audioMessage = audio_tools_msg()
        self.audioMessage.command = "enable_tts"
        self.audioToolsService(self.audioMessage)

        #Service navigation call
        self.depthToLaserMessage = depth_to_laser_msg()
        self.depthToLaserMessage.resolution=0
        self.depthToLaserMessage.scan_time=0.0
        self.depthToLaserMessage.range_min=0.0
        self.depthToLaserMessage.range_max=0.0
        self.depthToLaserMessage.scan_height=0.0
        self.navigationMessage = navigation_tools_srv()
        self.navigationMessage.command = "enable_all"
        self.navigationMessage.tf_enable= False
        self.navigationMessage.tf_frequency= 0.0
        self.navigationMessage.odom_enable= False
        self.navigationMessage.odom_frequency= 0.0
        self.navigationMessage.laser_enable= False
        self.navigationMessage.laser_frequency= 0.0
        self.navigationMessage.cmd_vel_enable= False
        self.navigationMessage.security_timer= 0.0
        self.navigationMessage.move_base_enable=False
        self.navigationMessage.goal_enable= False
        self.navigationMessage.robot_pose_suscriber_enable= False
        self.navigationMessage.path_enable= False
        self.navigationMessage.path_frequency= 0.0
        self.navigationMessage.robot_pose_publisher_enable= False
        self.navigationMessage.robot_pose_publisher_frequency= 0.0
        self.navigationMessage.result_enable= False
        self.navigationMessage.depth_to_laser_enable= False
        self.navigationMessage.depth_to_laser_parameters= self.depthToLaserMessage
        self.navigationMessage.free_zone_enable= False
        self.navigationToolsService(self.navigationMessage)

    
    def audioToolsService(self,msg):
        """
        Enables the audio Tools service from the toolkit of the robot.
        """
        rospy.wait_for_service('/robot_toolkit/audio_tools_srv')
        try:
            audio = rospy.ServiceProxy('/robot_toolkit/audio_tools_srv', audio_tools_srv)
            audioService = audio(msg)
            print("Audio tools service connected!")
        except rospy.ServiceException as e:
            print("Service call failed")

    
    def navigationToolsService(self,msg):
        """
        Enables the navigation Tools service from the toolkit of the robot.
        """
        rospy.wait_for_service('/robot_toolkit/navigation_tools_srv')
        try:
            navigation = rospy.ServiceProxy('/robot_toolkit/navigation_tools_srv', navigation_tools_srv)
            navigationService = navigation(msg)
            print("Navigation tools service connected!")
        except rospy.ServiceException as e:
            print("Service call failed")

    def tabletService(self,url):
        """
        Changes what's being displayed in the robots tablet
        """
        print(url)
        rospy.wait_for_service('/pytoolkit/ALTabletService/show_image_srv')
        try:
            tablet = rospy.ServiceProxy('/pytoolkit/ALTabletService/show_image_srv', tablet_service_srv)
            tabletSerivce = tablet(url)
            print("Tablet service connected!")
        except rospy.ServiceException as e:
            print("Service call failed")


def aux_mov(direction):
    geometry_msg = Twist()
    geometry_msg.linear.x = 0
    geometry_msg.linear.y = 0
    if direction=='up':
        geometry_msg.linear.x = speed
    if direction=='down':
        geometry_msg.linear.x = -speed
    if direction=='left':
        geometry_msg.linear.y = speed
    if direction=='right':
        geometry_msg.linear.y = -speed
    geometry_msg.linear.z = 0
    geometry_msg.angular.x = 0
    geometry_msg.angular.y = 0
    geometry_msg.angular.z = 0
    if direction=='rotateR':
        geometry_msg.angular.z = -speed
    if direction=='rotateL':
        geometry_msg.angular.z = speed
    return geometry_msg

remote = RemoteC()
# Create your views here.
def home(request):
    return render(request,"base.html")

def move(request):
    direction = request.GET["direction"]
    geometry_msg = aux_mov(direction)
    remote.movePublisher.publish(geometry_msg)
    return HttpResponse(status=204)

def speak(request):
    print("decir")
    t2s_msg = speech_msg()
    t2s_msg.language = request.GET["language"]
    t2s_msg.text = request.GET["words"]
    remote.speechPublisher.publish(t2s_msg)
    return HttpResponse(status=204)

def display(request):
    print("attempt")
    print(request.GET["url"])
    remote.tabletService(request.GET["url"])
    return HttpResponse(status=204)
