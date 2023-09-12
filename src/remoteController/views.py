# -*- coding: utf-8 -*-
from __future__ import unicode_literals
import rospy
from django.shortcuts import render, HttpResponse
from robot_toolkit_msgs.msg import audio_tools_msg, motion_tools_msg, vision_tools_msg,speech_msg, depth_to_laser_msg, camera_parameters_msg ,animation_msg, leds_parameters_msg, misc_tools_msg
from robot_toolkit_msgs.srv import audio_tools_srv, navigation_tools_srv, motion_tools_srv, vision_tools_srv, tablet_service_srv, misc_tools_srv
from geometry_msgs.msg import Twist
from django.views.decorators.csrf import csrf_exempt
from django.conf import settings
from django.core.files.base import ContentFile
import time
import os

linearX=0
linearY=0
angular=0
class RemoteC:
    def __init__(self):
        self.speechPublisher = rospy.Publisher('/speech',speech_msg,queue_size=10)
        self.movePublisher = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.animationPublisher = rospy.Publisher('/animations',animation_msg,queue_size=10)
        self.ledsPublisher = rospy.Publisher('/leds',leds_parameters_msg,queue_size=10)

        #Service speech call
        self.audioMessage = audio_tools_msg()
        self.audioMessage.command = "enable_tts"
        self.audioToolsService(self.audioMessage)

        #Service speech call
        self.miscMessage = misc_tools_msg()
        self.miscMessage.command = "enable_all"
        self.miscToolsService(self.miscMessage)

        #Service motion call
        self.motionMessage = motion_tools_msg()
        self.motionMessage.command = "enable_all"
        self.motionToolsService(self.motionMessage)

        #Service vision call
        self.cameraParametersMessage = camera_parameters_msg()
        self.cameraParametersMessage.compress = True
        self.cameraParametersMessage.compression_factor = 30
        self.cameraParametersMessage.brightness = 0
        self.cameraParametersMessage.contrast = 32
        self.cameraParametersMessage.saturation = 64
        self.cameraParametersMessage.hue = 0
        self.cameraParametersMessage.horizontal_flip = 0
        self.cameraParametersMessage.vertical_flip = 0
        self.cameraParametersMessage.auto_exposition = 1
        self.cameraParametersMessage.auto_white_balance = 1
        self.cameraParametersMessage.auto_gain = 1
        self.cameraParametersMessage.reset_camera_registers = 0
        self.cameraParametersMessage.auto_focus = 1
        self.visionMessage = vision_tools_msg()
        self.visionMessage.camera_name = "front_camera".encode('ascii')#botoom
        self.visionMessage.command = "enable"
        self.visionMessage.resolution = 0
        self.visionMessage.frame_rate = 0
        self.visionMessage.color_space = 0
        self.visionMessage.camera_parameters = self.cameraParametersMessage
        self.visionToolsService(self.visionMessage)

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

    
    def miscToolsService(self,msg):
        """
        Enables the misc Tools service from the toolkit of the robot.
        """
        rospy.wait_for_service('/robot_toolkit/misc_tools_srv')
        try:
            misc = rospy.ServiceProxy('/robot_toolkit/misc_tools_srv', misc_tools_srv)
            miscService = misc(msg)
            print("Misc tools service connected!")
        except rospy.ServiceException as e:
            print("Service call failed")
    
    
    def motionToolsService(self,msg):
        """
        Enables the motion Tools service from the toolkit of the robot.
        """
        rospy.wait_for_service('/robot_toolkit/motion_tools_srv')
        try:
            motion = rospy.ServiceProxy('/robot_toolkit/motion_tools_srv', motion_tools_srv)
            motionService = motion(msg)
            print("Motion tools service connected!")
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

    def visionToolsService(self,msg):
        """
        Enables the vision Tools service from the toolkit of the robot.
        """
        rospy.wait_for_service('/robot_toolkit/vision_tools_srv')
        try:
            vision = rospy.ServiceProxy('/robot_toolkit/vision_tools_srv', vision_tools_srv)
            visionService = vision(msg)
            print("Vision tools service connected!")
        except rospy.ServiceException as e:
            print("Vision call failed")

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


def aux_mov(direction, speedP):
    speed = float(speedP)/100
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
    speed = request.GET["speed"]
    direction = request.GET["direction"]
    geometry_msg = aux_mov(direction,speed)
    remote.movePublisher.publish(geometry_msg)
    return HttpResponse(status=204)

def speak(request):
    t2s_msg = speech_msg()
    t2s_msg.language = request.GET["language"]
    t2s_msg.text = request.GET["words"]
    remote.speechPublisher.publish(t2s_msg)
    return HttpResponse(status=204)

def display(request):
    remote.tabletService(request.GET["url"])
    return HttpResponse(status=204)

@csrf_exempt
def save(request):
    #la imagen no puede tener espacios en el nombre
    imagen = request.FILES['imagen']
    imgPath = settings.MEDIA_ROOT
    full_filename = os.path.join(settings.MEDIA_ROOT, imagen.name)
    fout = open(full_filename,'wb+')

    file_content = ContentFile(imagen.read())
    for chunk in file_content.chunks():
        fout.write(chunk)
    fout.close()
    remote.tabletService("http://192.168.0.250:8000/media/"+imagen.name)
    return HttpResponse(status=204)


def animate(request):
    anim_msg = animation_msg()
    anim_msg.family = "animations"
    anim_msg.animation_name = request.GET["animation"]
    remote.animationPublisher.publish(anim_msg)
    return HttpResponse(status=204)

def setLeds(request):
    leds_msg = leds_parameters_msg()
    leds_msg.name="FaceLeds".encode('ascii')
    leds_msg.red = int(request.GET["red"])
    leds_msg.green = int(request.GET["green"])
    leds_msg.blue = int(request.GET["blue"])
    leds_msg.time = 0
    remote.ledsPublisher.publish(leds_msg)
    time.sleep(0.5)    
    return HttpResponse(status=204)

