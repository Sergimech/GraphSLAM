#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <math.h>

class Differential_Drive {
public:
	Differential_Drive() {
		sub_ = n_.subscribe("/cmd_vel", 1, &Differential_Drive::callback, this);
		pub_ = n_.advertise<geometry_msgs::PoseStamped>("/pose", 1);
		wheel_base = 0.35;
		wheel_radius = 0.05;
		x = 0.0;
		y = 0.0;
		phi = 0.0;
	}

private:
	ros::NodeHandle n_;
	ros::Subscriber sub_;
	ros::Publisher pub_;
	double wheel_base;
	double wheel_radius;
	double x;
	double y;
	double phi;
	
	inline double v_r(double velocity, double rotation) {
		return ( 2 * velocity + rotation * wheel_base ) / 2 * wheel_radius;
	}

	inline double v_l(double velocity, double rotation) {
		return ( 2 * velocity - rotation * wheel_base ) / 2 * wheel_radius;
	}

	inline double x_dot(double velocity_right_wheel, double velocity_left_wheel, double rotation) {
		return ( wheel_radius/2 ) * ( velocity_right_wheel + velocity_left_wheel ) * cos( rotation );
	}

	inline double y_dot(double velocity_right_wheel, double velocity_left_wheel, double rotation) {
		return ( wheel_radius/2 ) * ( velocity_right_wheel + velocity_left_wheel ) * sin( rotation );
	}

	inline double phi_dot(double velocity_right_wheel, double velocity_left_wheel) {
		return ( wheel_radius / wheel_base ) * ( velocity_right_wheel - velocity_left_wheel);
	}

	void callback(const geometry_msgs::Twist::ConstPtr& input) {
		double velocity = input->linear.x;
		double rotation = input->angular.z;
		double velocity_right_wheel = v_r(velocity, rotation);
		double velocity_left_wheel = v_l(velocity, rotation);

		x += x_dot(velocity_right_wheel, velocity_left_wheel, phi);
		y += y_dot(velocity_right_wheel, velocity_left_wheel, phi);
		phi += phi_dot(velocity_right_wheel, velocity_left_wheel);

		ROS_INFO("linear [%f], angular [%f]", velocity, rotation);
		ROS_INFO("x [%f], y [%f], rotation [%f]", x, y, phi);

		geometry_msgs::PoseStamped output;
		output.pose.position.x = x;
		output.pose.position.y = y;
		output.pose.orientation = tf::createQuaternionMsgFromYaw(phi);
		pub_.publish(output);
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "differential_drive");
	Differential_Drive diff_drive;
	ros::spin();
	return 0;
}
