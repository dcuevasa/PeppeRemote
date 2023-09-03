from django.conf.urls import url
from . import views

urlpatterns = [
    url("arriba/",views.move,{'direction':'up'}, name="arriba"),
    url("abajo/",views.move,{'direction':'down'}, name="abajo"),
    url("izquierda/",views.move,{'direction':'left'}, name="izquierda"),
    url("derecha/",views.move,{'direction':'right'}, name="derecha"),
    url("speak/",views.speak, name="speak"),
    url("",views.home, name="home")
]
