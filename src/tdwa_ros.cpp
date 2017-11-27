#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
// #include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <cmath>

class DynamicWindowApproach
{
public:
  DynamicWindowApproach();
  virtual ~DynamicWindowApproach();
  void run();
  bool initNode();
  // void onOdomMsg(const nav_msgs::Odometry::ConstPtr &odomPtr );


private:

  void Evaluation();
  void NormalizeEval();
  void GenerateTrajectory();
  void CalcDistEval();
  void CalcHeadingEval();
  void CalcDynamicWindow();
  void f();
  void MoveObstacle();

  double toRadian(double degree);
  double toDegree(double radian);

private:
  //ROS node stuff
  ros::NodeHandle m_nodeHandle;

  //Subscriber

  //Publisher
  ros::Publisher m_pubCmdVel;

  //TF

  //DWA
  Eigen::Vector2f goal;// ok
  double goalR; //ok
  double obstacleR; //ok
  double dt; //ok
  std::vector<double> model; //ok
  std::vector<double> evalParam; //ok

};

DynamicWindowApproach::DynamicWindowApproach()
  : m_nodeHandle()
{
  initNode();
}

DynamicWindowApproach::~DynamicWindowApproach()
{


}

double DynamicWindowApproach::toRadian(double degree){
  return degree/180*M_PI;
}
double DynamicWindowApproach::toDegree(double radian){
  return radian/M_PI*180;
}

bool DynamicWindowApproach::initNode()
{
    //TODO ROS_PARAM
    // goal = {10.0, 10.0};//goal
    // goalR = 1.0;
    // obstacle = {{5,9},{8,8},{8,9},{7,9}};
    obstacleR = 1.0;
    dt = 0.1;

    // [max_velocity[m/s],max_angular_velocity[rad/s],max_acc[m/ss], max_angular_acc[rad/ss],vel_resolution[m/s],angular_vel_resolution[rad/s]]
    std::vector<double> tmpModel{1.0,DynamicWindowApproach::toRadian(20.0),
      0.2,DynamicWindowApproach::toRadian(50.0),
      0.01,DynamicWindowApproach::toRadian(1.0)};
    model = tmpModel;

    //[heading,dist,velocity,predictSimTime]
    std::vector<double> tmpEval{0.1, 0.5, 0.2, 3.0};
    evalParam = tmpEval;

    m_pubCmdVel = m_nodeHandle.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10, true);

    return true;
}

void DynamicWindowApproach::run()
{
  ROS_INFO("Entering the spin.\n");
  Eigen::VectorXd x(5); //x, y, th, v, w
  x << 0.0, 0.0, 0.0, 0.0, 0.0;
  Eigen::Vector2f u(0.0, 0.0); //input v, w
  std::vector< std::vector<double> > obstacle{{5, 9}, {8, 8}, {8, 9}, {7, 9}}; //TODO use costmap

  ros::Rate rate(10.0); //HZ
  geometry_msgs::Twist msg;
  while(ros::ok()){
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
    m_pubCmdVel.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dyanamic_window_approach");
  DynamicWindowApproach dwaNode;
  dwaNode.run();

  return 0;
}
