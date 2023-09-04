# -*- coding: utf-8 -*-
from __future__ import unicode_literals
import rospy
from django.shortcuts import render, HttpResponse
from robot_toolkit_msgs.msg import speech_msg, audio_tools_msg
from robot_toolkit_msgs.srv import audio_tools_srv
from geometry_msgs.msg import Twist

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

remote = RemoteC()
# Create your views here.
def home(request):
    return render(request,"base.html")
def move(request,direction):
    geometry_msg = Twist()
    geometry_msg.linear.x = 0;
    geometry_msg.linear.y = 0;
    if direction=='up':
        geometry_msg.linear.x = speed;
    if direction=='down':
        geometry_msg.linear.x = -speed;
    if direction=='left':
        geometry_msg.linear.y = speed;
    if direction=='right':
        geometry_msg.linear.y = -speed;
    geometry_msg.linear.z = 0;
    geometry_msg.angular.x = 0;
    geometry_msg.angular.y = 0;
    #geometry_msg.angular.z = angular;
    remote.movePublisher.publish(geometry_msg)
    return HttpResponse(status=204)
def speak(request):
    t2s_msg = speech_msg()
    t2s_msg.language = request.GET["language"]
    t2s_msg.text = request.GET["words"]
    remote.speechPublisher.publish(t2s_msg)
    return HttpResponse(status=204)
