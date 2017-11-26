#include <ros/ros.h>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"



class MagneticLaserIntegrator
{
public:
  MagneticLaserIntegrator();
  virtual ~MagneticLaserIntegrator();
  void run();
  bool initNode();
  void onAmclPoseMsg(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclPosePtr );
  void onAmclMagPoseMsg(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& mfAmclPosePtr,
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& mfMagPosePtr );

private:
  bool transformMapToOdom(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& posePtr,
    tf::StampedTransform& map_to_odom_stamped);
  void naiveIntegratePose(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& mfAmclPosePtr,
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& mfMagPosePtr);
private:
  //ROS node stuff
  ros::NodeHandle m_nodeHandle;

  //Subscriber
  ros::Subscriber m_subAmclPose;

  //Message filters
  message_filters::Subscriber< geometry_msgs::PoseWithCovarianceStamped > m_mfAmclPose;
  message_filters::Subscriber< geometry_msgs::PoseWithCovarianceStamped > m_mfMagPose;
  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, geometry_msgs::PoseWithCovarianceStamped> SyncPolicy;
  message_filters::Synchronizer< SyncPolicy > *sync;
  geometry_msgs::PoseWithCovarianceStampedConstPtr m_posePtr;

  //Publisher
  ros::Publisher m_pubIntegratedPose;
  ros::Publisher m_pubPoseforAMCL; //publiseh "initial_pose"


  //TF
  tf::TransformListener m_tfListener;
  tf::Transform m_latestTf;
  tf::TransformBroadcaster* m_tfBroadcaster;
  bool m_latestTfValid;
  bool m_integratedPoseValid;

};

MagneticLaserIntegrator::MagneticLaserIntegrator()
  : m_nodeHandle()
  , m_latestTfValid(false)
  , m_integratedPoseValid(false)
  , m_mfAmclPose(m_nodeHandle, "/amcl_pose", 1)
  , m_mfMagPose(m_nodeHandle, "/mag_pose", 1)
{
  m_tfBroadcaster = new tf::TransformBroadcaster();
  sync = new message_filters::Synchronizer< SyncPolicy >( SyncPolicy( 10 ), m_mfAmclPose, m_mfMagPose);


  initNode();
}

MagneticLaserIntegrator::~MagneticLaserIntegrator()
{
  delete m_tfBroadcaster;
  delete sync;

}

bool MagneticLaserIntegrator::initNode()
{
  //AMCL Pose subscriber
  m_subAmclPose = m_nodeHandle.subscribe("/amcl_pose", 10, &MagneticLaserIntegrator::onAmclPoseMsg, this);

  //MessageFilter Callback
  sync->registerCallback( boost::bind( &MagneticLaserIntegrator::onAmclMagPoseMsg, this, _1, _2 ));

  //Integrated pose Publisher
  m_pubIntegratedPose = m_nodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("integrated_pose", 10, true); //TODO set queueSize

  m_pubPoseforAMCL = m_nodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",10,true); //TODO set queueSize

  return true;
}

void MagneticLaserIntegrator::onAmclPoseMsg(const geometry_msgs::PoseWithCovarianceStampedConstPtr& amclPosePtr )
{
  // ROS_INFO( "Use AMCL Pose only" )
  // ROS_INFO( "onAmclPoseMsg(): frame_id=%s, stamp=%i.%i", amclPosePtr->header.frame_id.c_str(), amclPosePtr->header.stamp.sec, amclPosePtr->header.stamp.nsec );
  // ROS_INFO("AmclPose x: [%f], y: [%f], z: [%f]", amclPosePtr->pose.pose.position.x, amclPosePtr->pose.pose.position.y, amclPosePtr->pose.pose.position.z);

  //main loop
  if(!m_integratedPoseValid) // if cannot use integrated pose made from mag_pose and amcl_pose, just use amcl_pose
  {
    tf::StampedTransform map_to_odom_stamped;
    bool tf_valid = transformMapToOdom(
      amclPosePtr,
      map_to_odom_stamped);

    if(tf_valid){
      ROS_INFO("Send transform");
      m_tfBroadcaster->sendTransform(map_to_odom_stamped);
    }

    m_pubIntegratedPose.publish(amclPosePtr);
  }

  m_integratedPoseValid = false;
}

bool MagneticLaserIntegrator::transformMapToOdom(
  const geometry_msgs::PoseWithCovarianceStampedConstPtr& posePtr,  tf::StampedTransform& map_to_odom_stamped){
  // subtracting base to odom from map to base and send map to odom instead

  ros::Time time = posePtr->header.stamp;
  // tf::Quaternion tmp_quat;
  // tf::quaternionMsgToTF(amclPosePtr->pose.pose.orientation, tmp_quat);
  tf::Transform map_to_base_link(
          tf::Quaternion(posePtr->pose.pose.orientation.x,
                        posePtr->pose.pose.orientation.y,
                        posePtr->pose.pose.orientation.z,
                        posePtr->pose.pose.orientation.w),
          tf::Vector3(posePtr->pose.pose.position.x,
                      posePtr->pose.pose.position.y,
                      0.0));

  tf::Stamped<tf::Pose> base_link_to_map_stamped(map_to_base_link.inverse(), time, "base_link");

  tf::Stamped<tf::Pose> odom_to_map;
  try{

    m_tfListener.waitForTransform("base_link", "odom", time, ros::Duration(10.0));
    m_tfListener.transformPose("odom", base_link_to_map_stamped, odom_to_map);

  }catch(tf::TransformException &e){

    ROS_WARN("%s", e.what());
    return false;

  }
  map_to_odom_stamped = tf::StampedTransform(
      odom_to_map.inverse(), time, "map", "odom");

  m_latestTf = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                                 tf::Point(odom_to_map.getOrigin()));
  m_latestTfValid = true;

  return true;
}

void MagneticLaserIntegrator::onAmclMagPoseMsg(
  const geometry_msgs::PoseWithCovarianceStampedConstPtr& mfAmclPosePtr,
   const geometry_msgs::PoseWithCovarianceStampedConstPtr& mfMagPosePtr )
{
  ROS_INFO( "Start integrate process");
  ROS_INFO( "AMCL pose time stamp: frame_id=%s, stamp=%i.%i", mfAmclPosePtr->header.frame_id.c_str(), mfAmclPosePtr->header.stamp.sec, mfAmclPosePtr->header.stamp.nsec );
  ROS_INFO( "MAG pose time stamp: frame_id=%s, stamp=%i.%i", mfMagPosePtr->header.frame_id.c_str(), mfMagPosePtr->header.stamp.sec, mfMagPosePtr->header.stamp.nsec );

  //integrate mag_pose and amcl_pose in a naive method
  naiveIntegratePose(mfAmclPosePtr, mfMagPosePtr);

  tf::StampedTransform map_to_odom_stamped;
  bool tf_valid = transformMapToOdom(
    m_posePtr,
    map_to_odom_stamped);

  if(tf_valid){
    ROS_INFO("Send transform");
    m_tfBroadcaster->sendTransform(map_to_odom_stamped);
  }
  m_pubIntegratedPose.publish(m_posePtr);

  m_integratedPoseValid = true; //if send transform, ignore onAmclPoseMsg() until subscribe new message
}

void MagneticLaserIntegrator::naiveIntegratePose(
  const geometry_msgs::PoseWithCovarianceStampedConstPtr& mfAmclPosePtr,
  const geometry_msgs::PoseWithCovarianceStampedConstPtr& mfMagPosePtr)
{
  ROS_INFO("Integrate mag_pose and amcl_pose in a naive method");
  /*

   something compare mag_pose and amcl_pose here

   */

  m_posePtr = mfAmclPosePtr;
}


void MagneticLaserIntegrator::run()
{
  ROS_INFO("Entering the spin.\n");

  ros::Duration transform_tolerance_;
  transform_tolerance_.fromSec(0.1);

  ros::Rate rate(10.0); //HZ
  while(ros::ok()){

    if (m_latestTfValid)//TODO when process under the loop that send transform by subscribe callback, do not send transform
    {
      // Nothing changed, so we'll just republish the last transform, to keep
      // everybody happy.
      ros::Time transform_expiration = (ros::Time::now() +
                                        transform_tolerance_);
      tf::StampedTransform tmp_tf_stamped(m_latestTf.inverse(),
                                          transform_expiration,
                                          "map", "odom");
      m_tfBroadcaster->sendTransform(tmp_tf_stamped);
    }

    ros::spinOnce();
    rate.sleep();

  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "magnetic_laser_integrator");
  MagneticLaserIntegrator mlNode;
  mlNode.run();

  return 0;
}
