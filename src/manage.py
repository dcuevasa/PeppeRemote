#!/usr/bin/env python
import os
import sys
import rospy
from robot_toolkit_msgs.msg import speech_msg


if __name__ == "__main__":
    rospy.init_node('webController')
    os.environ.setdefault("DJANGO_SETTINGS_MODULE", "PeppeRemote.settings")
    try:
        from django.core.management import execute_from_command_line
    except ImportError:
        # The above import may fail for some other reason. Ensure that the
        # issue is really that Django is missing to avoid masking other
        # exceptions on Python 2.
        try:
            import django
            rospy.spin()
        except ImportError:
            raise ImportError(
                "Couldn't import Django. Are you sure it's installed and "
                "available on your PYTHONPATH environment variable? Did you "
                "forget to activate a virtual environment?"
            )
        raise
    execute_from_command_line(sys.argv)
