#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
#include <boost/thread/thread.hpp>

#include <string.h>
#include <vector>
#include <gtkmm.h>

#define COMMAND_TIMEOUT_SEC 0.5

const int PUBLISH_FREQ = 20;

class WindowedSimOperator
{
public:
	WindowedSimOperator(unsigned int numOfRobots);

	void createCommand(double linear, double angular);
	void sendControl();
    void showJoyEmulator();

private:
	unsigned int activeRobot;
	unsigned int mNumberOfRobots;
	double mMaxLinearVelocity;
	double mMaxAngularVelocity;

	std::vector<ros::Publisher> mCommandPublisher;
	std::vector<geometry_msgs::Twist> mCommand;

	void on_front_button_click();
    void on_back_button_click();
    void on_left_button_click();
    void on_right_button_click();
    void on_stop_button_click();
    void on_switch_button_click();

};

void WindowedSimOperator::showJoyEmulator()
{
    int argc = 0;
    char ** argcc = NULL;

    Gtk::Main kit(argc, argcc);
    Gtk::Window window;

    Gtk::Button switchButton("Switch");
    Gtk::Button frontButton("↑");
    Gtk::Button leftButton("←");
    Gtk::Button rightButton("→");
    Gtk::Button backButton("↓");
    Gtk::Button stopButton("■");

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

    switchButton.signal_clicked().connect(sigc::mem_fun(*this, &WindowedSimOperator::on_switch_button_click));
    frontButton.signal_clicked().connect(sigc::mem_fun(*this, &WindowedSimOperator::on_front_button_click));
    backButton.signal_clicked().connect(sigc::mem_fun(*this, &WindowedSimOperator::on_back_button_click));
    leftButton.signal_clicked().connect(sigc::mem_fun(*this, &WindowedSimOperator::on_left_button_click));
    rightButton.signal_clicked().connect(sigc::mem_fun(*this, &WindowedSimOperator::on_right_button_click));
    stopButton.signal_clicked().connect(sigc::mem_fun(*this, &WindowedSimOperator::on_stop_button_click));

    row1.pack_start(switchButton, Gtk::PACK_EXPAND_WIDGET);
    row1.pack_start(frontButton, Gtk::PACK_EXPAND_WIDGET);
    row2.pack_start(leftButton, Gtk::PACK_EXPAND_WIDGET);
    row2.pack_start(stopButton, Gtk::PACK_EXPAND_WIDGET);
    row2.pack_start(rightButton, Gtk::PACK_EXPAND_WIDGET);
    row3.pack_start(backButton, Gtk::PACK_EXPAND_WIDGET);

    full.pack_start(row1, Gtk::PACK_EXPAND_WIDGET);
    full.pack_start(row2, Gtk::PACK_EXPAND_WIDGET);
    full.pack_start(row3, Gtk::PACK_EXPAND_WIDGET);

    window.add(full);
    window.show_all_children();

    kit.run(window);
}

WindowedSimOperator::WindowedSimOperator(unsigned int numOfRobots)
{
	activeRobot = 0;
	mNumberOfRobots = numOfRobots;
	mMaxLinearVelocity = 1.0;
	mMaxAngularVelocity = 1.0;

	ros::NodeHandle node;

	// Publish commands for every robot in the simulation
	char topic[50];
	for(unsigned int i = 0; i < mNumberOfRobots; i++)
	{
		if(mNumberOfRobots > 1)
			sprintf(topic, "/robot_%d/cmd_vel", i);
		else
			sprintf(topic, "/cmd_vel", i);
		mCommandPublisher.push_back(node.advertise<geometry_msgs::Twist>(topic, 1));

		geometry_msgs::Twist twist;
		mCommand.push_back(twist);
	}

	boost::thread t1(boost::bind(&WindowedSimOperator::showJoyEmulator, this));
}

void WindowedSimOperator::createCommand(double linear, double angular)
{
	for(unsigned int i = 0; i < mNumberOfRobots; i++)
	{
		if(activeRobot == i)
		{
			mCommand[i].linear.x = linear * mMaxLinearVelocity;
			mCommand[i].angular.z = angular * mMaxAngularVelocity;
		}else
		{
			mCommand[i].linear.x = 0;
			mCommand[i].angular.z = 0;
		}
	}
}

void WindowedSimOperator::sendControl()
{
	for(unsigned int i = 0; i < mNumberOfRobots; i++)
	{
		mCommandPublisher[i].publish(mCommand[i]);
	}
}

void WindowedSimOperator::on_front_button_click(){
    ROS_WARN("Moving forward");
    createCommand(1, 0);
}

void WindowedSimOperator::on_left_button_click(){
    ROS_WARN("Moving left");
    createCommand(1, 1);
}

void WindowedSimOperator::on_switch_button_click(){
    ROS_WARN("Switch");
    activeRobot = ( activeRobot + 1 ) % mNumberOfRobots;
    ROS_WARN("Switching to robot %d", activeRobot);
}

void WindowedSimOperator::on_right_button_click(){
    ROS_WARN("Moving right");
    createCommand(1, -1);
}

void WindowedSimOperator::on_back_button_click(){
    ROS_WARN("Moving backward");
    createCommand(-1, 0);
}

void WindowedSimOperator::on_stop_button_click(){
    ROS_WARN("Stoping");
    createCommand(0, 0);
}

int main(int argc, char** argv)
{
	// init the ROS node
	ros::init(argc,argv,"windowed_sim_joy");
	ros::NodeHandle node;

	unsigned int num = 1;
	if(argc > 1)
	{
		num = (unsigned int)atoi(argv[1]);
		if(num > 6) num = 6;
	}

	ROS_ERROR("Using Joystick to operate %d robots in simulation!", num);
	WindowedSimOperator simOp(num);

	ros::Rate pub_rate(PUBLISH_FREQ);
	while (node.ok())
	{
		ros::spinOnce();
		simOp.sendControl();
		pub_rate.sleep();
	}

	return 0;
}

