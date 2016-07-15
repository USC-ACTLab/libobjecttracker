// ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include "vicon_ros/NamedPoseArray.h"

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl_ros/point_cloud.h>
#include "transformation_estimation_3DYaw.h"

#ifdef USE_VICON_DIRECTLY
// VICON
#include "Client.h"
#endif

#define ENABLE_GROUND_TRUTH

struct DynamicsConfiguration
{
  double maxXVelocity;
  double maxYVelocity;
  double maxZVelocity;
  double maxPitchRate;
  double maxRollRate;
  double maxYawRate;
  double maxRoll;
  double maxPitch;
};

class Object
{
public:
  std::string name;
  size_t markerConfigurationIdx;
  size_t dynamicsConfigurationIdx;
  Eigen::Affine3f lastTransformation;
  ros::Time lastValidTransform;
};

class ObjectTracker
{
public:
  ObjectTracker()
    : m_subscribeMarkers()
    , m_markerConfigurations()
    , m_dynamicsConfigurations()
    , m_objects()
    , m_br()
#ifdef USE_VICON_DIRECTLY
    , m_pubLatency()
    , m_pubPointCloud()
#endif
    , m_pubPoses()
  {
    ros::NodeHandle nl("~");
    nl.getParam("hostName", m_hostName);

    readMarkerConfigurations();
    readDynamicsConfigurations();
    readObjects();

#ifdef USE_VICON_DIRECTLY
    m_pubLatency = nl.advertise<std_msgs::Float32>("latency", 1);
    m_pubPointCloud = nl.advertise<sensor_msgs::PointCloud>("pointCloud", 1);
#else
    ros::NodeHandle n;
    m_subscribeMarkers = n.subscribe("/vicon/pointCloud", 1, &ObjectTracker::pointCloudCallback, this);
#endif
    m_pubPoses = nl.advertise<vicon_ros::NamedPoseArray>("poses", 1);
  }

  void run()
  {
#ifdef USE_VICON_DIRECTLY
    using namespace ViconDataStreamSDK::CPP;

    // Make a new client
    Client client;

    // Connect to a server
    ROS_INFO("Connecting to %s ...", m_hostName.c_str());
    while (ros::ok() && !client.IsConnected().Connected) {
      // Direct connection
      bool ok = (client.Connect(m_hostName).Result == Result::Success);
      if(!ok) {
        ROS_WARN("Connect failed...");
      }
      ros::spinOnce();
    }

    // Configure vicon
    client.EnableUnlabeledMarkerData();
#ifdef ENABLE_GROUND_TRUTH
    client.EnableSegmentData();
#endif

    client.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush);

    // Set the global up axis
    client.SetAxisMapping(Direction::Forward,
                          Direction::Left,
                          Direction::Up); // Z-up

    // Discover the version number
    Output_GetVersion version = client.GetVersion();
    ROS_INFO("VICON Version: %d.%d.%d", version.Major, version.Minor, version.Point);

    sensor_msgs::PointCloud msgPointCloud;
    msgPointCloud.header.seq = 0;
    msgPointCloud.header.frame_id = "world";

    while (ros::ok()) {
      // Get a frame
      while (client.GetFrame().Result != Result::Success) {
      }

      // Get the latency
      std_msgs::Float32 msgLatency;
      msgLatency.data = client.GetLatencyTotal().Total;
      m_pubLatency.publish(msgLatency);

      // Get the unlabeled markers
      size_t count = client.GetUnlabeledMarkerCount().MarkerCount;
      msgPointCloud.header.seq += 1;
      msgPointCloud.header.stamp = ros::Time::now();
      msgPointCloud.points.resize(count);

      // Create a point cloud from the markers
      pcl::PointCloud<pcl::PointXYZ>::Ptr markers(new pcl::PointCloud<pcl::PointXYZ>);

      for(size_t i = 0; i < count; ++i) {
        Output_GetUnlabeledMarkerGlobalTranslation translation =
          client.GetUnlabeledMarkerGlobalTranslation(i);
        markers->push_back(pcl::PointXYZ(
          translation.Translation[0] / 1000.0,
          translation.Translation[1] / 1000.0,
          translation.Translation[2] / 1000.0));

        msgPointCloud.points[i].x = translation.Translation[0] / 1000.0;
        msgPointCloud.points[i].y = translation.Translation[1] / 1000.0;
        msgPointCloud.points[i].z = translation.Translation[2] / 1000.0;
      }

      m_pubPointCloud.publish(msgPointCloud);

      runICP(markers);

#ifdef ENABLE_GROUND_TRUTH
      size_t subjectCount = client.GetSubjectCount().SubjectCount;
      for (size_t subjectIndex = 0; subjectIndex < subjectCount; ++subjectIndex) {
        std::string subjectName = client.GetSubjectName(subjectIndex).SubjectName;
        size_t segmentCount = client.GetSegmentCount(subjectName).SegmentCount;
        for (size_t segmentIndex = 0 ; segmentIndex < segmentCount ; ++segmentIndex) {
          std::string segmentName = client.GetSegmentName(subjectName, segmentIndex).SegmentName;

          Output_GetSegmentGlobalTranslation translation = client.GetSegmentGlobalTranslation(subjectName, segmentName);
          Output_GetSegmentGlobalRotationQuaternion quaternion = client.GetSegmentGlobalRotationQuaternion(subjectName, segmentName);

          tf::Transform transform;
          transform.setOrigin(tf::Vector3(
            translation.Translation[0] / 1000.0,
            translation.Translation[1] / 1000.0,
            translation.Translation[2] / 1000.0));
          tf::Quaternion q(
            quaternion.Rotation[0],
            quaternion.Rotation[1],
            quaternion.Rotation[2],
            quaternion.Rotation[3]);
          transform.setRotation(q);
          m_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "/vicon/" + subjectName + "/" + segmentName));
        }
      }
#endif

      ros::spinOnce();
    }
#else
    ros::spin();
#endif
  }

private:
  void readMarkerConfigurations()
  {
    ros::NodeHandle nl("~");
    int numConfigurations;
    nl.getParam("numMarkerConfigurations", numConfigurations);
    for (int i = 0; i < numConfigurations; ++i) {
      m_markerConfigurations.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));
      std::stringstream sstr;
      sstr << "markerConfigurations/" << i << "/numPoints";
      int numPoints;
      nl.getParam(sstr.str(), numPoints);

      std::vector<double> offset;
      std::stringstream sstr2;
      sstr2 << "markerConfigurations/" << i << "/offset";
      nl.getParam(sstr2.str(), offset);
      for (int j = 0; j < numPoints; ++j) {
        std::stringstream sstr3;
        sstr3 << "markerConfigurations/" << i << "/points/" << j;
        std::vector<double> points;
        nl.getParam(sstr3.str(), points);
        m_markerConfigurations.back()->push_back(pcl::PointXYZ(points[0] + offset[0], points[1] + offset[1], points[2] + offset[2]));
      }
    }
  }

  void readDynamicsConfigurations()
  {
    ros::NodeHandle nl("~");
    int numConfigurations;
    nl.getParam("numDynamicsConfigurations", numConfigurations);
    m_dynamicsConfigurations.resize(numConfigurations);
    for (int i = 0; i < numConfigurations; ++i) {
      std::stringstream sstr;
      sstr << "dynamicsConfigurations/" << i;
      nl.getParam(sstr.str() + "/maxXVelocity", m_dynamicsConfigurations[i].maxXVelocity);
      nl.getParam(sstr.str() + "/maxYVelocity", m_dynamicsConfigurations[i].maxYVelocity);
      nl.getParam(sstr.str() + "/maxZVelocity", m_dynamicsConfigurations[i].maxZVelocity);
      nl.getParam(sstr.str() + "/maxPitchRate", m_dynamicsConfigurations[i].maxPitchRate);
      nl.getParam(sstr.str() + "/maxRollRate", m_dynamicsConfigurations[i].maxRollRate);
      nl.getParam(sstr.str() + "/maxYawRate", m_dynamicsConfigurations[i].maxYawRate);
      nl.getParam(sstr.str() + "/maxRoll", m_dynamicsConfigurations[i].maxRoll);
      nl.getParam(sstr.str() + "/maxPitch", m_dynamicsConfigurations[i].maxPitch);
    }
  }

  void readObjects()
  {
    ros::NodeHandle nl("~");
    int numObjects;
    nl.getParam("numObjects", numObjects);
    m_objects.resize(numObjects);
    for (int i = 0; i < numObjects; ++i) {
      std::stringstream sstr;
      sstr << "objects/" << i << "/";

      std::string name;
      int markerConfiguration;
      int dynamicsConfiguration;
      std::vector<double> initialPosition;
      double initialYaw;
      nl.getParam(sstr.str() + "name", name);
      nl.getParam(sstr.str() + "markerConfiguration", markerConfiguration);
      nl.getParam(sstr.str() + "dynamicsConfiguration", dynamicsConfiguration);
      nl.getParam(sstr.str() + "initialPosition", initialPosition);
      nl.getParam(sstr.str() + "initialYaw", initialYaw);

      m_objects[i].name = name;
      m_objects[i].markerConfigurationIdx = markerConfiguration;
      m_objects[i].dynamicsConfigurationIdx = dynamicsConfiguration;
      m_objects[i].lastTransformation = pcl::getTransformation(
        initialPosition[0],
        initialPosition[1],
        initialPosition[2],
        initialYaw, 0, 0);
      m_objects[i].lastValidTransform = ros::Time::now();
    }
  }

  void pointCloudCallback(const sensor_msgs::PointCloud::ConstPtr pointCloud)
  {
    // Create a point cloud from the markers
    pcl::PointCloud<pcl::PointXYZ>::Ptr markers(new pcl::PointCloud<pcl::PointXYZ>);

    for(size_t i = 0; i < pointCloud->points.size(); ++i) {
      markers->push_back(pcl::PointXYZ(
        pointCloud->points[i].x,
        pointCloud->points[i].y,
        pointCloud->points[i].z
        ));
    }
    // if (pointCloud->points.size() != 6) {
    //   ROS_INFO("NumPoints: %lu", pointCloud->points.size());
    // }

    // struct timeval start, end;
    // gettimeofday(&start, NULL);

    // long totalTime = (time.tv_sec * 1000) + (time.tv_usec / 1000);

    runICP(markers, pointCloud->header.stamp);

    // gettimeofday(&end, NULL);
    // double dt = (end.tv_sec + end.tv_usec / 1e6) - (start.tv_sec + start.tv_usec / 1e6);
    // ROS_INFO("Latency: %f s", dt);
  }

  void runICP(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr markers,
    ros::Time stamp)
  {
    static bool initialized = false;
    static int seq = 0;

    vicon_ros::NamedPoseArray msgPoses;
    msgPoses.header.seq = seq++;
    msgPoses.header.frame_id = "world";
    msgPoses.header.stamp = stamp;//ros::Time::now();

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>::Ptr trans(new pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>);
    // pcl::registration::TransformationEstimation3DYaw<pcl::PointXYZ, pcl::PointXYZ>::Ptr trans(new pcl::registration::TransformationEstimation3DYaw<pcl::PointXYZ, pcl::PointXYZ>);
    // icp.setTransformationEstimation(trans);


    // // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations(5);
    // // Set the transformation epsilon (criterion 2)
    // icp.setTransformationEpsilon(1e-8);
    // // Set the euclidean distance difference epsilon (criterion 3)
    // icp.setEuclideanFitnessEpsilon(1);

    icp.setInputTarget(markers);

    // for (auto& object : m_objects) {
    for (size_t i = 0; i < m_objects.size(); ++i) {
      Object& object = m_objects[i];

      float dt = (stamp - object.lastValidTransform).toSec();
      // Set the max correspondence distance
      // TODO: take max here?
      const DynamicsConfiguration& dynConf = m_dynamicsConfigurations[object.dynamicsConfigurationIdx];
      float maxV = dynConf.maxXVelocity;
      icp.setMaxCorrespondenceDistance(maxV * dt);
      // ROS_INFO("max: %f", maxV * dt);

      // Update input source
      icp.setInputSource(m_markerConfigurations[object.markerConfigurationIdx]);

      // Perform the alignment
      pcl::PointCloud<pcl::PointXYZ> result;
      icp.align(result, object.lastTransformation.matrix());
      if (!icp.hasConverged()) {
        ros::Time t = ros::Time::now();
        ROS_INFO("ICP did not converge %d.%d", t.sec, t.nsec);
        continue;
      }

      // Obtain the transformation that aligned cloud_source to cloud_source_registered
      Eigen::Matrix4f transformation = icp.getFinalTransformation();

      Eigen::Affine3f tROTA(transformation);
      float x, y, z, roll, pitch, yaw;
      pcl::getTranslationAndEulerAngles(tROTA, x, y, z, roll, pitch, yaw);

      // Compute changes:
      float last_x, last_y, last_z, last_roll, last_pitch, last_yaw;
      pcl::getTranslationAndEulerAngles(object.lastTransformation, last_x, last_y, last_z, last_roll, last_pitch, last_yaw);

      float vx = (x - last_x) / dt;
      float vy = (y - last_y) / dt;
      float vz = (z - last_z) / dt;
      float wroll = (roll - last_roll) / dt;
      float wpitch = (pitch - last_pitch) / dt;
      float wyaw = (yaw - last_yaw) / dt;

      // ROS_INFO("v: %f,%f,%f, w: %f,%f,%f, dt: %f", vx, vy, vz, wroll, wpitch, wyaw, dt);

      if (   fabs(vx) < dynConf.maxXVelocity
          && fabs(vy) < dynConf.maxYVelocity
          && fabs(vz) < dynConf.maxZVelocity
          && fabs(wroll) < dynConf.maxRollRate
          && fabs(wpitch) < dynConf.maxPitchRate
          && fabs(wyaw) < dynConf.maxYawRate
          && fabs(roll) < dynConf.maxRoll
          && fabs(pitch) < dynConf.maxPitch)
      {

        object.lastTransformation = tROTA;
        object.lastValidTransform = stamp;

        // ROS_INFO("Pos: [%f, %f, %f, %f, %f, %f]", x, y, z, roll, pitch, yaw);

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x, y, z));
        tf::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        transform.setRotation(q);
        m_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", object.name));

        msgPoses.poses.resize(msgPoses.poses.size() + 1);
        msgPoses.poses.back().name = object.name;
        geometry_msgs::Pose& pose = msgPoses.poses.back().pose;

        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;

        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
      } else {
        ros::Time t = ros::Time::now();
        std::stringstream sstr;
        sstr << "Dynamic check failed at " << t.sec << "." << t.nsec << std::endl;
        if (fabs(vx) >= dynConf.maxXVelocity) {
          sstr << "vx: " << vx << " >= " << dynConf.maxXVelocity << std::endl;
        }
        if (fabs(vy) >= dynConf.maxYVelocity) {
          sstr << "vy: " << vy << " >= " << dynConf.maxYVelocity << std::endl;
        }
        if (fabs(vz) >= dynConf.maxZVelocity) {
          sstr << "vz: " << vz << " >= " << dynConf.maxZVelocity << std::endl;
        }
        if (fabs(wroll) >= dynConf.maxRollRate) {
          sstr << "wroll: " << wroll << " >= " << dynConf.maxRollRate << std::endl;
        }
        if (fabs(wpitch) >= dynConf.maxPitchRate) {
          sstr << "wpitch: " << wpitch << " >= " << dynConf.maxPitchRate << std::endl;
        }
        if (fabs(wyaw) >= dynConf.maxYawRate) {
          sstr << "wyaw: " << wyaw << " >= " << dynConf.maxYawRate << std::endl;
        }
        if (fabs(roll) >= dynConf.maxRoll) {
          sstr << "roll: " << roll << " >= " << dynConf.maxRoll << std::endl;
        }
        if (fabs(pitch) >= dynConf.maxPitch) {
          sstr << "pitch: " << pitch << " >= " << dynConf.maxPitch << std::endl;
        }

        ROS_INFO("%s", sstr.str().c_str());
      }

    }

    m_pubPoses.publish(msgPoses);

    initialized = true;
  }

private:
  ros::Subscriber m_subscribeMarkers;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> m_markerConfigurations;
  std::vector<DynamicsConfiguration> m_dynamicsConfigurations;
  std::vector<Object> m_objects;
  tf::TransformBroadcaster m_br;
  std::string m_hostName;
#ifdef USE_VICON_DIRECTLY
  ros::Publisher m_pubLatency;
  ros::Publisher m_pubPointCloud;
#endif
  ros::Publisher m_pubPoses;
};




int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_tracker");
  ObjectTracker ot;
  ot.run();

  return 0;
}
