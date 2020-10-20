#ifndef ros_rqt_plugin_H_
#define ros_rqt_plugin_H_

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <ui_ros_rqt_plugin.h>
#include <QWidget>


namespace ros_rqt_plugin
{
    class two_button_plugin : public rqt_gui_cpp::Plugin
    {
        Q_OBJECT
        public:
            two_button_plugin();
            virtual void initPlugin(qt_gui_cpp::PluginContext & context);
            virtual void shutdownPlugin();
            virtual void saveSettings(
                qt_gui_cpp::Settings & plugin_settings,
                qt_gui_cpp::Settings & instance_settings
            ) const;
            virtual void restoreSettings(
                const qt_gui_cpp::Settings & plugin_settings,
                const qt_gui_cpp::Settings & instance_settings
            );

        // Comment in to signal that the plugin has a way to configure it
        //bool hasConfiguration() const;
        //void triggerConfiguration();

        private:
            Ui::two_button ui_;
            QWidget * widget_;
    };
} // ros_rqt_plugin 

#endif