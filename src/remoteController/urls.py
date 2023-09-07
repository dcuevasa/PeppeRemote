from django.conf.urls import include, url
from . import views

urlpatterns = [
    url("move/",views.move, name="move"),
    url("speak/",views.speak, name="speak"),
    url("show_img/",views.display, name="show_img"),
    url("",views.home, name="home")
]