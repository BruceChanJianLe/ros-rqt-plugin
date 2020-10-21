#include "ros_rqt_plugin/ros_rqt_plugin.hpp"
#include <pluginlib/class_list_macros.h>
#include <QStringList>


namespace ros_rqt_plugin
{
    two_button_plugin::two_button_plugin()
    :   rqt_gui_cpp::Plugin(),
        widget_(0)
    {
        // Constructor is called first before initPlugin function, needless to say.
        // give QObjects reasonable names
        setObjectName("ros_rqt_plugin");;
    }

    void two_button_plugin::initPlugin(qt_gui_cpp::PluginContext & context)
    {
        // Access standalone command line arguments
        QStringList argv = context.argv();

        // Create QWidget
        widget_ = new QWidget();

        // Extend the widget with all attributes and children from UI file
        ui_.setupUi(widget_);

        // add widget to the user interface
        context.addWidget(widget_);

        // Define ROS publishers
        buttton_1_pub_ = getNodeHandle().advertise<std_msgs::Bool>("button_1_topic", 1);
        buttton_2_pub_ = getNodeHandle().advertise<std_msgs::Bool>("button_2_topic", 1);

        // Declare ROS msg_
        msg_.data = true;

        // Connect Qt Widgets
        connect(ui_.pushButton_1, SIGNAL(pressed()), this, SLOT(button_1_callback_()));
        connect(ui_.pushButton_2, SIGNAL(pressed()), this, SLOT(button_2_callback_()));
    }

    void two_button_plugin::shutdownPlugin()
    {
        ;
    }

    void two_button_plugin::saveSettings(
        qt_gui_cpp::Settings & plugin_settings,
        qt_gui_cpp::Settings & instance_settings
    ) const
    {
        ;
    }

    void two_button_plugin::restoreSettings(
        const qt_gui_cpp::Settings & plugin_settings,
        const qt_gui_cpp::Settings & instance_settings
    )
    {
        ;
    }

    void two_button_plugin::button_1_callback_()
    {
        // Publish msg_ as true
        buttton_1_pub_.publish(msg_);
    }

    void two_button_plugin::button_2_callback_()
    {
        // Publish msg_ as true
        buttton_2_pub_.publish(msg_);
    }

} // namespace ros_rqt_plugin

PLUGINLIB_EXPORT_CLASS(ros_rqt_plugin::two_button_plugin, rqt_gui_cpp::Plugin);
