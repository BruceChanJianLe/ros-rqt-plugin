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

} // namespace ros_rqt_plugin

PLUGINLIB_EXPORT_CLASS(ros_rqt_plugin::two_button_plugin, rqt_gui_cpp::Plugin);
