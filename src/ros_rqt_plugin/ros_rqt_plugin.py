#!/usr/bin/env python

import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
# For add refresh infrom of button
# from python_qt_binding.QtGui import QIcon

from std_msgs.msg import Bool


class Buttons(Plugin):
    def __init__(self, context):
        super(Buttons, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('Buttons')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('ros_rqt_plugin'), 'resource', 'ros_rqt_plugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('ros_rqt_plugin_ui')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() +
                                        (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Define ROS publisher
        self.button_1_pub = rospy.Publisher('button_1_topic', Bool, queue_size=1)
        self.button_2_pub = rospy.Publisher('button_2_topic', Bool, queue_size=1)

        # Define ROS msg
        self.msg = True

        # You can set button icon like so
        # For add refresh infrom of button
        # self._widget.pushButton_1.setIcon(QIcon.fromTheme('view-refresh'))

        # Connecting widget and ui
        self._widget.pushButton_1.pressed.connect(self.button_1_callback)
        self._widget.pushButton_2.pressed.connect(self.button_2_callback)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self.button_1_pub.unregister()
        self.button_2_pub.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget
        # title bar
        # Usually used to open a modal configuration dialog

    def button_1_callback(self):
        self.button_1_pub.publish(self.msg)

    def button_2_callback(self):
        self.button_2_pub.publish(self.msg)