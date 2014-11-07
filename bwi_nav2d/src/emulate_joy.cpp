#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <nav2d_operator/cmd.h>
#include <nav2d_navigator/SendCommand.h>
#include <nav2d_navigator/commands.h>
#include <gtkmm.h>

/******************************************************
Buttons:
 0: A
 1: B
 2: X
 3: Y
 4: LB
 5: RB
 6: BACK
 7: START
 8: Logitech*
 9: Left Stick
10: Right Stick

 ******************************************************/
using namespace std;

#define ENABLE_EMULATOR true

class Teleoperator
{
public:
    Teleoperator();

private:
    void showJoyEmulator();
    void joyCB(const sensor_msgs::Joy::ConstPtr& msg);

    void on_front_button_click();
    void on_back_button_click();
    void on_left_button_click();
    void on_right_button_click();
    void on_rotate_button_click();

    ros::NodeHandle mNode;
    ros::Publisher mCommandPublisher;
    ros::Subscriber mJoySubscriber;
    ros::ServiceClient mNavigatorClient;
    ros::ServiceClient mExploreClient;
    ros::ServiceClient mGetMapClient;

    int mAxisVelocity;
    int mAxisDirection;
    int mButtonDriveMode;
    int mButtonPauseNavigator;
    int mButtonStartExploration;
    int mButtonGetMap;
    int mButtonStop;

    bool mButtonPressed;
};

Teleoperator::Teleoperator()
{
    // Button and Axis configuration
    mAxisVelocity = 4;
    mAxisDirection = 0;

    mButtonDriveMode = 5;
    mButtonPauseNavigator = 6;
    mButtonStartExploration = 0;
    mButtonGetMap = 3;
    mButtonStop = 1;

    mCommandPublisher = mNode.advertise<nav2d_operator::cmd>("cmd", 1);
    mNavigatorClient = mNode.serviceClient<nav2d_navigator::SendCommand>(NAV_COMMAND_SERVICE);
    mExploreClient = mNode.serviceClient<nav2d_navigator::SendCommand>(NAV_EXPLORE_SERVICE);
    mGetMapClient = mNode.serviceClient<nav2d_navigator::SendCommand>(NAV_GETMAP_SERVICE);

    mButtonPressed = false;

    if(ENABLE_EMULATOR){
        showJoyEmulator();
    }
}

void Teleoperator::showJoyEmulator()
{
    int argc = 0;
    char ** argcc = NULL;

    Gtk::Main kit(argc, argcc);
    Gtk::Window window;

    Gtk::Button frontButton("↑");
    Gtk::Button leftButton("←");
    Gtk::Button rightButton("→");
    Gtk::Button backButton("↓");
    Gtk::Button rotateButton("↻");

    Gtk::VBox full;
    Gtk::HBox row1;
    Gtk::HBox row2;
    Gtk::HBox row3;

    window.set_border_width(10);
    window.set_default_size(600, 600);
    window.set_size_request(600, 600);
    window.set_resizable(false);
    window.set_title("Joy Stick Emulator");
    window.set_position(Gtk::WIN_POS_CENTER);

    frontButton.signal_clicked().connect(sigc::mem_fun(*this, &Teleoperator::on_front_button_click));
    backButton.signal_clicked().connect(sigc::mem_fun(*this, &Teleoperator::on_back_button_click));
    leftButton.signal_clicked().connect(sigc::mem_fun(*this, &Teleoperator::on_left_button_click));
    rightButton.signal_clicked().connect(sigc::mem_fun(*this, &Teleoperator::on_right_button_click));
    rotateButton.signal_clicked().connect(sigc::mem_fun(*this, &Teleoperator::on_rotate_button_click));

    row1.pack_start(frontButton, Gtk::PACK_EXPAND_WIDGET);
    row2.pack_start(leftButton, Gtk::PACK_EXPAND_WIDGET);
    row2.pack_start(rotateButton, Gtk::PACK_EXPAND_WIDGET);
    row2.pack_start(rightButton, Gtk::PACK_EXPAND_WIDGET);
    row3.pack_start(backButton, Gtk::PACK_EXPAND_WIDGET);

    full.pack_start(row1, Gtk::PACK_EXPAND_WIDGET);
    full.pack_start(row2, Gtk::PACK_EXPAND_WIDGET);
    full.pack_start(row3, Gtk::PACK_EXPAND_WIDGET);

    window.add(full);
    window.show_all_children();

    kit.run(window);
}

void Teleoperator::on_front_button_click(){
    ROS_WARN("Moving forward");
    nav2d_operator::cmd cmd;
    cmd.Turn = 0;
    cmd.Velocity = 10.0;
    cmd.Mode = 1;
    mCommandPublisher.publish(cmd);
}

void Teleoperator::on_left_button_click(){
    ROS_WARN("Moving left");
    nav2d_operator::cmd cmd;
    cmd.Turn = -0.5;
    cmd.Velocity = 10.0;
    cmd.Mode = 1;
    mCommandPublisher.publish(cmd);
}

void Teleoperator::on_right_button_click(){
    ROS_WARN("Moving right");
    nav2d_operator::cmd cmd;
    cmd.Turn = 0.5;
    cmd.Velocity = 10.0;
    cmd.Mode = 1;
    mCommandPublisher.publish(cmd);
}

void Teleoperator::on_back_button_click(){
    ROS_WARN("Moving backward");
    nav2d_operator::cmd cmd;
    cmd.Turn = 0;
    cmd.Velocity = -10.0;
    cmd.Mode = 1;
    mCommandPublisher.publish(cmd);
}

void Teleoperator::on_rotate_button_click(){
    ROS_WARN("Rotating");
    nav2d_operator::cmd cmd;
    cmd.Turn = 0;
    cmd.Velocity = 0;
    cmd.Mode = 1;
    mCommandPublisher.publish(cmd);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleoperator");
    Teleoperator tele_op;

    ros::spin();
}
