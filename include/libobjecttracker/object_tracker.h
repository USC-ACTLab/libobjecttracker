#pragma once
#include <cstddef>
#include <stdint.h>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace libobjecttracker {

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
    double maxFitnessScore;
  };

  class ObjectTracker;
  class Object
  {
  public:
    Object(
      size_t markerConfigurationIdx,
      size_t dynamicsConfigurationIdx,
      const Eigen::Affine3f& initialTransformation);

    const Eigen::Affine3f& transformation() const;

    bool lastTransformationValid() const;

  private:
    size_t m_markerConfigurationIdx;
    size_t m_dynamicsConfigurationIdx;
    Eigen::Affine3f m_lastTransformation;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_lastValidTransform;
    bool m_lastTransformationValid;

    friend ObjectTracker;
  };

  typedef pcl::PointCloud<pcl::PointXYZ>::Ptr MarkerConfiguration;

  class ObjectTracker
  {
  public:
    ObjectTracker(
      const std::vector<DynamicsConfiguration>& dynamicsConfigurations,
      const std::vector<MarkerConfiguration>& markerConfigurations,
      const std::vector<Object>& objects);

    void update(
      pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);

    const std::vector<Object>& objects() const;

    void setLogWarningCallback(
      std::function<void(const std::string&)> logWarn);

  private:
    void runICP(
      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr markers);

    bool initialize(
      pcl::PointCloud<pcl::PointXYZ>::ConstPtr markers);

    void logWarn(const std::string& msg);

  private:
    std::vector<MarkerConfiguration> m_markerConfigurations;
    std::vector<DynamicsConfiguration> m_dynamicsConfigurations;
    std::vector<Object> m_objects;
    bool m_initialized;

    std::function<void(const std::string&)> m_logWarn;
  };

} // namespace libobjecttracker


