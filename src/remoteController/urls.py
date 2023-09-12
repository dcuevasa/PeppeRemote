from django.conf.urls import include, url
from django.conf import settings
from django.conf.urls.static import static
from . import views

urlpatterns = [
    url("move/",views.move, name="move"),
    url("speak/",views.speak, name="speak"),
    url("show_img/",views.display, name="show_img"),
    url("save_img/",views.save, name="save_img"),
    url("animate/",views.animate, name="animate"),
    url("setLeds/",views.setLeds, name="setLeds"),
    url("",views.home, name="home")
]