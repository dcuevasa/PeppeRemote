# -*- coding: utf-8 -*-
from __future__ import unicode_literals
import rospy
from django.shortcuts import render, HttpResponse
from robot_toolkit_msgs.msg import speech_msg

class RemoteC:
    def __init__(self):
        self.Publisher = rospy.Publisher('/test',speech_msg,queue_size=10)

remote = RemoteC()
# Create your views here.
def home(request):
    return render(request,"base.html")
def move(request,direction):
    print(direction)
    return HttpResponse(status=204)
def speak(request):
    t2s_msg = speech_msg()
    t2s_msg.language = 'English'
    t2s_msg.text = request.GET["words"]
    remote.Publisher.publish(t2s_msg)
    return HttpResponse(status=204)
