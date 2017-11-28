#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
// #include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <cmath>
#include <iostream>

class DynamicWindowApproach
{
public:
  DynamicWindowApproach();
  virtual ~DynamicWindowApproach();
  void run();
  bool initNode();
  // void onOdomMsg(const nav_msgs::Odometry::ConstPtr &odomPtr );


private:
  void evaluation(const Eigen::VectorXd x, const Eigen::Vector4d Vr,
    Eigen::MatrixXd& evalDB);
  void normalizeEval(Eigen::MatrixXd& evalDB);
  void generateTrajectory(Eigen::VectorXd& xt, double vt, double ot, std::vector< Eigen::VectorXd >& traj);
  double calcDistEval(const Eigen::VectorXd x);
  double calcHeadingEval(const Eigen::VectorXd x);
  void calcDynamicWindow(const Eigen::VectorXd x, Eigen::Vector4d& Vr);
  void f(Eigen::VectorXd& x, const Eigen::Vector2d u);
  void moveObstacle();

  double toRadian(double degree);
  double toDegree(double radian);

private:
  //ROS node stuff
  ros::NodeHandle m_nodeHandle;

  //Subscriber

  //Publisher
  ros::Publisher m_pubCmdVel;

  //TF

  //DWA conditions
  Eigen::Vector2d goal;
  double goalR; //ok
  std::vector< std::vector<double> > obstacle;
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
  Eigen::Vector2d tmpGoal(10.0, 10.0);//(x,y), TODO set from TP or GUI
  goal = tmpGoal;
  //TODO ROS_PARAM
  goalR = 1.0;
  std::vector< std::vector<double> > tmpObstacle{{5, 9}, {8, 8}, {8, 9}, {7, 9}}; //TODO use costmap
  obstacle = tmpObstacle;
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


void DynamicWindowApproach::evaluation(const Eigen::VectorXd x, const Eigen::Vector4d Vr, Eigen::MatrixXd& evalDB){
  //TODO add trajDB for vizualization
  int count = 1;
  for(double vt=Vr[0]; vt<Vr[1]; vt=vt+model[4]){
    for(double ot=Vr[2]; ot<Vr[3]; ot=ot+model[5]){
      Eigen::VectorXd xt = x;
      std::vector< Eigen::VectorXd > traj;
      generateTrajectory(xt, vt, ot, traj);

      double heading = calcHeadingEval(xt);
      double dist = calcDistEval(xt);
      double vel = vt;

      evalDB.col(count-1) << vt, ot, heading, dist, vel;
      ++count;
      evalDB.conservativeResize(5,count);
    }
  }
}


void DynamicWindowApproach::normalizeEval(Eigen::MatrixXd& evalDB){

  Eigen::VectorXd sumArray = evalDB.rowwise().sum();
  if (sumArray[2] > 0.0)
    evalDB.row(2) = evalDB.row(2) / sumArray[2];
  if (sumArray[3] > 0.0)
    evalDB.row(3) = evalDB.row(3) / sumArray[3];
  if (sumArray[4] > 0.0)
    evalDB.row(4) = evalDB.row(4) / sumArray[4];

  // std::cout<<"must be 1.0:"<<evalDB.rowwise().sum()<<std::endl;
}


void DynamicWindowApproach::generateTrajectory(Eigen::VectorXd& xt, double vt, double ot, std::vector< Eigen::VectorXd >& traj){
  Eigen::Vector2d u(vt, ot);
  double t = 0.0;
  double duration = 0.1;
  while( t <= evalParam[3]){
    t = t + duration;
    f(xt,u);
    traj.push_back(xt);
  }
}


double DynamicWindowApproach::calcDistEval(const Eigen::VectorXd x){
  double dist = 2.0; //FIXME
  std::vector< std::vector<double> >  predictObstacle = obstacle;
  double obVel = -0.8;
  double preT = evalParam[3];
  for(int i=0; i<obstacle.size(); ++i){
    predictObstacle[i][0] = obstacle[i][0] + preT * obVel;
    predictObstacle[i][1] = obstacle[i][1] + preT * obVel;
    double tmpDist = std::sqrt((predictObstacle[i][0] - x[0])*(predictObstacle[i][0] - x[0]) + (predictObstacle[i][1] - x[1])*(predictObstacle[i][1] - x[1])) - obstacleR;
    if(dist > tmpDist)
      dist=tmpDist;
  }

  return dist;
}


double DynamicWindowApproach::calcHeadingEval(const Eigen::VectorXd x){
  double theta = toDegree(x[2]);
  double goalTheta = toDegree(std::atan2(goal[1]-x[1], goal[0]-x[0]));
  double targetTheta = 180.0;
  if(goalTheta > theta)
    targetTheta = goalTheta - theta;
  else
    targetTheta = theta - goalTheta;
  double heading = 180.0 - targetTheta;

  //calculate goal distance evaluation
  double dist = (goal[0] - x[0])*(goal[0] - x[0]) + (goal[1] - x[1])*(goal[1] - x[1]);

  return heading + 1.0/dist;
}


void DynamicWindowApproach::calcDynamicWindow(const Eigen::VectorXd x, Eigen::Vector4d& Vr){
  //window1: input range
  Eigen::Vector4d Vs(0.0, model[0], -model[1], model[1]);

  //window2: kinematic constraints
  Eigen::Vector4d Vd(x[3]-model[2]*dt, x[3]+model[2]*dt, x[4]-model[3]*dt, x[4]+model[3]*dt);

  //window: intersection window1 and window2
  Vr <<std::max(Vs[0],Vd[0]), std::min(Vs[1],Vd[1]), std::max(Vs[2],Vd[2]), std::min(Vs[3],Vd[3]);

}


void DynamicWindowApproach::f(Eigen::VectorXd& x, const Eigen::Vector2d u){
  Eigen::MatrixXd F(5,5);
  F <<  1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0;

  Eigen::MatrixXd B(5,2);
  B <<  dt*std::cos(x[2]), 0.0,
        dt*std::sin(x[2]), 0.0,
        0.0, dt,
        1.0, 0.0,
        0.0, 1.0;
  x = F * x + B * u;
  // std::cout<<x<<std::endl;
}


void DynamicWindowApproach::moveObstacle(){
  double obVel = -0.8;
  for(int i=0; i<obstacle.size(); ++i){
    obstacle[i][0] = obstacle[i][0] + dt * obVel;
    obstacle[i][1] = obstacle[i][1] + dt * obVel;
  }
}


void DynamicWindowApproach::run()
{
  ROS_INFO("Entering the spin.\n");
  Eigen::VectorXd x(5); //state: x, y, th, v, w
  x << 0.0, 0.0, 0.0, 0.0, 0.0;
  Eigen::Vector2d u(0.0, 0.0); //input: v, w
  ros::Rate rate(10.0); //HZ

  while(ros::ok()){
    std::cout<<x<<std::endl;
    Eigen::Vector4d Vr;
    calcDynamicWindow(x, Vr);

    Eigen::MatrixXd evalDB(5,1);//vt, ot, heading, dist, vel
    evaluation(x, Vr, evalDB);
    normalizeEval(evalDB);

    std::vector<double> feval;
    double maxEval = 0.0;
    int maxIndex = -1;
    for( int i=0; i<evalDB.cols(); ++i){
      double tmpEval = evalParam[0]*evalDB(2,i) + evalParam[1]*evalDB(3,i)
       + evalParam[2]*evalDB(4,i);
      feval.push_back(tmpEval);
      if (maxEval < tmpEval){
        maxEval = tmpEval;
        maxIndex = i;
      }
    }

    Eigen::Vector2d u(evalDB(0,maxIndex), evalDB(1,maxIndex));
    f(x,u);
    moveObstacle();
    geometry_msgs::Twist msg;

    if( (goal[0] - x[0])*(goal[0] - x[0])
     + (goal[1] - x[1])*(goal[1] - x[1]) < goalR*goalR){
      msg.linear.x = 0.0;
      msg.linear.y = 0.0;
      msg.linear.z = 0.0;
      msg.angular.x = 0.0;
      msg.angular.y = 0.0;
      msg.angular.z = 0.0;
      m_pubCmdVel.publish(msg);
      break;
     }

    msg.linear.x = u(0);
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = u(1);
    m_pubCmdVel.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "dyanamic_window_approach");

  Eigen::MatrixXd B(4, 3);
  B<<11.0,12.0,13.0,14.0,15.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0;
  std::cout << "Size of matrix B is : ("
       << B.rows() << "," << B.cols() << ")"
       << std::endl;
  std::cout<<B<<std::endl;
  std::cout<<B(0,1)<<std::endl;

  DynamicWindowApproach dwaNode;
  dwaNode.run();

  return 0;
}
