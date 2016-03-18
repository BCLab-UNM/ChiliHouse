/*!
 * \brief   This is the main class that creates and connects the GUI components.
 *          The rover_gui_plugin.ui file is generated by the UI editor in QTCreator and most GUI elements are delared there
 *          and accessed via the ui object.
 *          RoverGUIPlugin is a plugin class for the rqt system. It compiles into a shared library that rqt can use. The rover GUI can be used
 *          by selecting it from within rtq or by running rtq -s rtq_rover_gui
 *          RoverGUIPlugin is event driven. The events either come from the ROS system or from QT. Event handlers process these events and update
 *          the GUI or send commands to the rovers as needed.
 *          This class also interfaces with GazeboSimManager in order to manipulate models in simulation.
 *
 * \author  Matthew Fricke
 * \date    November 11th 2015
 * \todo    Code works properly.
 * \class   RoverGUIPlugin
 */

#ifndef ROVERGUIPLUGIN_H
#define ROVERGUIPLUGIN_H

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include <rqt_gui_cpp/plugin.h>
#include <ui_rover_gui_plugin.h>
//#include <rqt_rover_gui/ui_rover_gui_plugin.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <ros/macros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <pluginlib/class_list_macros.h>
#include <QGraphicsView>
#include <QEvent>
#include <QKeyEvent>
#include <QProcess>

#include <map>
#include <set>

//ROS msg types
//#include "rover_onboard_target_detection/ATag.h"
//#include "rover_onboard_target_detection/harvest.h"

#include <QWidget>
#include <QTimer>
#include <QLabel>

#include "GazeboSimManager.h"

//AprilTag headers
#include "apriltag.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag36artoolkit.h"
#include "tag25h9.h"
#include "tag25h7.h"
#include "common/pnm.h"
#include "common/image_u8.h"
#include "common/zarray.h"
#include "common/getopt.h"

using namespace std;

namespace rqt_rover_gui {

  class RoverGUIPlugin : public rqt_gui_cpp::Plugin
  {
    Q_OBJECT
      
  public:
    RoverGUIPlugin();
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
    
    bool eventFilter(QObject *target, QEvent *event);

    // Handles output from the joystick node
    QString startROSJoyNode();
    QString stopROSJoyNode();

    void joyEventHandler(const sensor_msgs::Joy::ConstPtr& joy_msg);
    void cameraEventHandler(const sensor_msgs::ImageConstPtr& image);
    void EKFEventHandler(const ros::MessageEvent<const nav_msgs::Odometry> &event);
    void GPSEventHandler(const ros::MessageEvent<const nav_msgs::Odometry> &event);
    void encoderEventHandler(const ros::MessageEvent<const nav_msgs::Odometry> &event);
    void targetPickUpEventHandler(const ros::MessageEvent<const sensor_msgs::Image> &event);
    void targetDropOffEventHandler(const ros::MessageEvent<const sensor_msgs::Image> &event);
    void obstacleEventHandler(const ros::MessageEvent<std_msgs::UInt8 const>& event);

    void centerUSEventHandler(const sensor_msgs::Range::ConstPtr& msg);
    void leftUSEventHandler(const sensor_msgs::Range::ConstPtr& msg);
    void rightUSEventHandler(const sensor_msgs::Range::ConstPtr& msg);
    void IMUEventHandler(const sensor_msgs::Imu::ConstPtr& msg);

    void addModelToGazebo();
    QString addPowerLawTargets();
    QString addUniformTargets();
    QString addClusteredTargets();
    QString addFinalsWalls();
    QString addPrelimsWalls();


   // void targetDetectedEventHandler( rover_onboard_target_detection::ATag tagInfo ); //rover_onboard_target_detection::ATag msg );

    void setupSubscribers();
    void setupPublishers();

    // Detect rovers that are broadcasting information
    set<string> findConnectedRovers();
    
    //Image converter
	image_u8_t *copy_image_data_into_u8_container(int width, int height, uint8_t *rgb, int stride);

	//AprilTag detector
	int targetDetect(const sensor_msgs::ImageConstPtr& rawImage);

  signals:

    void joystickForwardUpdate(double);
    void joystickBackUpdate(double);
    void joystickLeftUpdate(double);
    void joystickRightUpdate(double);
    void updateObstacleCallCount(QString text);
    void updateLog(QString text);

  private slots:

    void currentRoverChangedEventHandler(QListWidgetItem *current, QListWidgetItem *previous);
    void pollRoversTimerEventHandler();
    void GPSCheckboxToggledEventHandler(bool checked);
    void EKFCheckboxToggledEventHandler(bool checked);
    void encoderCheckboxToggledEventHandler(bool checked);
    void autonomousRadioButtonEventHandler(bool marked);
    void allAutonomousRadioButtonEventHandler(bool marked);
    void joystickRadioButtonEventHandler(bool marked);
    void buildSimulationButtonEventHandler();
    void clearSimulationButtonEventHandler();
    void visualizeSimulationButtonEventHandler();
    void gazeboClientFinishedEventHandler();
    void gazeboServerFinishedEventHandler();  
    void displayLogMessage(QString msg);


  private:

    void checkAndRepositionRover(QString rover_name, float x, float y);
    void readRoverModelXML(QString path);

    map<string,ros::Publisher> control_mode_publishers;
    ros::Publisher joystick_publisher;
    map<string,ros::Publisher> targetPickUpPublisher;
    map<string,ros::Publisher> targetDropOffPublisher;

    ros::Subscriber joystick_subscriber;
    map<string,ros::Subscriber> encoder_subscribers;
    map<string,ros::Subscriber> gps_subscribers;
    map<string,ros::Subscriber> ekf_subscribers;
    ros::Subscriber us_center_subscriber;
    ros::Subscriber us_left_subscriber;
    ros::Subscriber us_right_subscriber;
    ros::Subscriber imu_subscriber;

    map<string,ros::Subscriber> obstacle_subscribers;
    map<string,ros::Subscriber> targetDropOffSubscribers;
    map<string,ros::Subscriber> targetPickUpSubscribers;
    image_transport::Subscriber camera_subscriber;

    string selected_rover_name;
    set<string> rover_names;
    ros::NodeHandle nh;
    QWidget* widget;
    Ui::RoverGUI ui;

    QProcess* joy_process;
    QTimer* timer; // for rover polling

    QString log_messages;
    GazeboSimManager sim_mgr;

    map<string,int> rover_control_state;
    bool all_autonomous;

    float arena_dim; // in meters

    map<string,int> targetsPickedUp;
    map<int,bool> targetsDroppedOff;

    bool display_sim_visualization;

    // Object clearance. These values are used to quickly determine where objects can be placed int time simulatio
    float target_cluster_size_64_clearance;
    float target_cluster_size_16_clearance;
    float target_cluster_size_4_clearance;
    float target_cluster_size_1_clearance;
    float rover_clearance;
    float collection_disk_clearance;
    float barrier_clearance;

    unsigned long obstacle_call_count;
    
    //AprilTag objects
	apriltag_family_t *tf = NULL; //tag family
	apriltag_detector_t *td = NULL; //tag detector

	//Image container
	image_u8_t *u8_image = NULL;
	
	//AprilTag assigned to collection zone
	int collectionZoneID = 256;
  };
} // end namespace

#endif // ROVERGUIPLUGIN
