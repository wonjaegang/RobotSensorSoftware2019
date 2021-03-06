//ros basic header
#include "ros/ros.h"
//message header
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"
#include "navigation/stopmsg.h"
//math header&constant
#include "math.h"
#define PI acos(-1)
//declare msg,pub as a static so that we can use it outside of the main function
static geometry_msgs::Twist motorvalue;
static std_msgs::Bool motorpower;
static ros::Publisher path_planning_motorpub;
static ros::Publisher path_planning_motorpowerpub;


static double turtlebot_width = 0.138, turtlebot_length = 0.178;//actually 0.178
static double lidarxy[2][360];//point cloud, NOT FILTERED
static double x = 0, y = 0, q = 0; //where am I now
static double destination_x = 5.0, destination_y = 0;//### Change your INPUT here ###
ros::Duration duration(0.05);
double go = 1;


void msgCALLBACK(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	motorpower.data = 1 * int(go);
	//######### making GRID MAP #########
	int map_size = 72;    //%4 should be 0
	double grid_size = 0.01;
	int turtlebot_width_grid = int(turtlebot_width / 2 / grid_size) * 2 + 2;
	int turtlebot_length_grid = int(turtlebot_length / 2 / grid_size) * 2 + 2;
	double map_length = double(map_size) * grid_size;
	char map[map_size][map_size];
	//initializing the map
	for (int i = 0; i < map_size; i++)
	{
		for (int k = 0; k < map_size; k++)
		{
			map[i][k] = ' ';
		}
	}
	for (int i = 0; i < 360; i++)//assumming the forward lidar angle is 0
	{
		lidarxy[0][i] = scan->ranges[i] * cos(double((i + 180) % 360) *PI / 180);
		lidarxy[1][i] = scan->ranges[i] * sin(double((i + 180) % 360)*PI / 180);
		if (fabs(lidarxy[1][i]) < map_length / 2 && fabs(lidarxy[0][i] - map_length / 4) < map_length / 2)
			//if the obstacle locates inside of the rectengular map
		{
			if (!(lidarxy[0][i] == 0 && lidarxy[1][i] == 0))//filtering 0 values
			{
				int grid_x = (map_size - 1) - int((lidarxy[0][i] + map_length / 4) / grid_size);
				int grid_y = int((map_length / 2 - lidarxy[1][i]) / grid_size);
				if(grid_x > 2 && grid_x < map_size - 2 && grid_y > 2 && grid_y < map_size - 2)
				{
					for(int n = 0; n < 25; n++)
					{
						map[grid_x + (n / 5) - 2][grid_y + (n % 5) - 2] = 'x';
					}
				}	
			}
			//grid with 'x' is an obstacle grid
		}
	}
	//###############  making virtual obstacles  ###########
	for (int i = 0; i < map_size; i++)
	{
		for (int k = 0; k < map_size; k++)
		{
			if (map[i][k] == 'x')
			{
				for (int xy = 0; xy < 2; xy++)
				{
					int blank = 0, nearest_d = 0;
					for (int n = 1; n < map_size - (k * (1 - xy) + i * xy + 1); n++)
					{
						if (map[i + n * xy][k + n * (1 - xy)] == ' ')
							blank++;
						else
						{
							nearest_d = n;
							break;
						}
					}
					if (blank >= 5 && blank <= 15)
					{
						for (int n = 1; n < nearest_d; n++)
						{
							map[i + n * xy][k + n * (1 - xy)] = '-';
						}
					}
				}
			}
		}
	}

	//######### Dynamic Window Approach #########
	double v_max = 0.1, w_max = 2.84, a_max = 0.02;//w : counterclockwise is +
	double w_head, w_speed, w_clear;
	double caution_distance = 0.7, danger_distance = 0.07;
	double g_head = 0.1, g_speed = 0.9, g_clear = 0.4;
	double pn_x, pn_y, dt = 2;
	double pn_x_r, pn_y_r;
	int dwa_size = 21;//0 value is important, so let's pick odd num
	double O_max = 0, TEMP, v_dwa, w_dwa;
	double head_check, speed_check, clear_check;

	for (int a = 0; a < dwa_size; a++)
	{
		for (int b = 0; b < dwa_size + 1; b++)
		{
			double v = double(b) * (v_max / double(dwa_size));
			double w = double(a - (dwa_size - 1) / 2) * (2 * w_max / double(dwa_size));
			pn_x = x + v * cos(q + w * dt / 2) * dt;
			pn_y = y + v * sin(q + w * dt / 2) * dt;
			pn_x_r = pn_x * cos(q) + pn_y * sin(q) - x * cos(q) - y * sin(q);
			pn_y_r = -pn_x * sin(q) + pn_y * cos(q) + x * sin(q) - y * cos(q);
			w_head = 1 - fabs(atan2(destination_y - pn_y, destination_x - pn_x) - (q + w * dt)) / PI;
			//function of the direction
			w_speed = v / v_max;
			//function of the speed
			double d = 100, TEMP_d = 100, d_x = 0, d_y = 0;			
			//calculate d with grid map
			for (int i = 0; i < map_size; i++)
			{
				for (int k = 0; k < map_size; k++)
				{
					if (map[i][k] == 'x' || map[i][k] == '-')
					{
						d_x = double((map_size*3/4 - (i + 1))*2 + 1) * grid_size/2;
                        d_y = double((map_size*1/2 - (k + 1))*2 + 1) * grid_size/2;
                        TEMP_d = sqrt(pow(d_x - pn_x_r, 2) + pow(d_y - pn_y_r, 2));
							if (TEMP_d < d)
								d = TEMP_d;
					}
				}
			}
			if (d < turtlebot_length * 1 / 3) //preventing the selection of the v,w crossing over the obstacles
				break;
			if (d < danger_distance)
				w_clear = 0;
			else if (d < caution_distance)
				w_clear = (d - danger_distance) / (caution_distance - danger_distance);
			else
				w_clear = 1;
			//function of the distance to the obstacle
			TEMP = g_head * w_head + g_speed * w_speed + g_clear * w_clear;
			map[(map_size - 1) - int((pn_x_r + map_length / 4) / grid_size)][int((map_length / 2 - pn_y_r) / grid_size)] = int(TEMP / (g_head + g_speed + g_clear) * 9 + 0.5) + 48;
			//ASCII code of integer
			if (TEMP > O_max)
			{
				O_max = TEMP;
				v_dwa = v; w_dwa = w;
				head_check = w_head;
				speed_check = w_speed;
				clear_check = w_clear;
			}
		}
	}
	motorvalue.linear.x = v_dwa * go;   //my tutlebot is reversed...
	motorvalue.angular.z = w_dwa * go;   //counterclockwise is +
	//stopping code : preventing turtlebot's oscillation at the last point
	if (fabs(destination_x - x) < 0.05 && fabs(destination_x - x) < 0.05)
	{
		go = 0;
		printf("stop");
	}
	for (int i = 0; i < map_size; i++)
	{
		for (int k = 0; k < map_size; k++)
		{
			printf(" %c", map[i][k]);
		}
		printf("\n");
	}
	printf("DWA value : %f, %f, %f\n", head_check, speed_check, clear_check);
	printf("DWA : %fm/s, %frad/s\n", v_dwa, w_dwa);
	printf("location : %f, %f theta : %fdeg\n", x, y, q / PI * 180);
	printf("==================================================================================================================\n");
	path_planning_motorpowerpub.publish(motorpower);
	path_planning_motorpub.publish(motorvalue);
	duration.sleep();
}


void odomCALLBACK(const nav_msgs::Odometry::ConstPtr& odom)
{
	float sqw = odom->pose.pose.orientation.w * odom->pose.pose.orientation.w;
	float sqx = odom->pose.pose.orientation.x * odom->pose.pose.orientation.x;
	float sqy = odom->pose.pose.orientation.y * odom->pose.pose.orientation.y;
	float sqz = odom->pose.pose.orientation.z * odom->pose.pose.orientation.z;
	float pitch = asinf(2.0f * (odom->pose.pose.orientation.y*odom->pose.pose.orientation.z + odom->pose.pose.orientation.w*odom->pose.pose.orientation.x)); // rotation about x-axis
	float roll = atan2f(2.0f * (odom->pose.pose.orientation.w*odom->pose.pose.orientation.y - odom->pose.pose.orientation.x*odom->pose.pose.orientation.z),(-sqx - sqy + sqz + sqw)); // rotation about y-axis
	//q = yaw
	q = atan2f(2.0f * (odom->pose.pose.orientation.w*odom->pose.pose.orientation.z - odom->pose.pose.orientation.x*odom->pose.pose.orientation.y),(-sqx + sqy - sqz + sqw)); // rotation about z-axis
   
 
	//converting from quaternian to theta
	x = odom->pose.pose.position.x;
	y = odom->pose.pose.position.y;
}


void stoping_func(const navigation::stopmsg::ConstPtr& stop)
{
	go = 0;
}


int main(int argc, char **argv)
{
	//initialize the node, and resister it to ROSCORE
	ros::init(argc, argv, "path_planning");
	//declare NodeHandle
	ros::NodeHandle nh;
	//declare pub,sub
	ros::Subscriber path_planning_odom = nh.subscribe("/odom", 1, odomCALLBACK);
	ros::Subscriber path_planning_break = nh.subscribe("/stop", 1, stoping_func);
	path_planning_motorpub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	path_planning_motorpowerpub = nh.advertise<std_msgs::Bool>("/motor_power", 1);
	ros::Subscriber path_planning_sub = nh.subscribe("/scan", 1, msgCALLBACK);
	ros::spin();
	return 0;
}