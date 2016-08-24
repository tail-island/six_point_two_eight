#include <boost/foreach.hpp>
#include <pcl/common/centroid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include "six_point_two_eight/point_cloud_utilities.h"

namespace point_cloud_utilities {
  // 実装。
  
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> PointCloud;

  tf::TransformListener transform_listener_;
  pcl::visualization::PCLVisualizer::Ptr visualizer_;

  PointCloud::Ptr downsamplePointCloud(PointCloud::ConstPtr point_cloud, double leaf_size) {
    pcl::VoxelGrid<Point> voxel_grid;
    voxel_grid.setInputCloud(point_cloud);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);

    PointCloud::Ptr downsampled_point_cloud(new PointCloud());
    voxel_grid.filter(*downsampled_point_cloud);

    return downsampled_point_cloud;
  }

  PointCloud::Ptr transformPointCloud(PointCloud::ConstPtr point_cloud, Eigen::Matrix4f transform) {
    PointCloud::Ptr transformed_point_cloud(new PointCloud());
    pcl::transformPointCloud(*point_cloud, *transformed_point_cloud, transform);

    return transformed_point_cloud;
  }

  PointCloud::Ptr transformPointCloud(PointCloud::ConstPtr point_cloud, const std::string& target_frame, double x_adjust) {
    // 奥行きがずれているようだったので、補正します。
    PointCloud::Ptr x_adjusted_point_cloud(new PointCloud(*point_cloud));
    BOOST_FOREACH(Point& point, *x_adjusted_point_cloud) {
      point.z += x_adjust;  // 深度センサーの生データは座標系が異なります。右がX軸で下がY軸、前がZ軸です。
    }

    std_msgs::Header header = pcl_conversions::fromPCL(point_cloud->header);
    if (!transform_listener_.waitForTransform(target_frame, header.frame_id, header.stamp, ros::Duration(1.0))) {
      ROS_WARN_STREAM("Can't wait...");
      throw std::exception();
    }
  
    PointCloud::Ptr transformed_point_cloud(new PointCloud());
    if (!pcl_ros::transformPointCloud(target_frame, *x_adjusted_point_cloud, *transformed_point_cloud, transform_listener_)) {
      ROS_WARN("Transform failed...");
      throw std::exception();
    }

    return transformed_point_cloud;
  }

  template <typename PointT>
  class SphereCondition : public pcl::ConditionBase<PointT> {
  private:
    Eigen::Vector3f centroid_;
    double radius_;

  public:
    SphereCondition(const Eigen::Vector3f& centroid, double radius)
      : centroid_(centroid), radius_(radius)
    {
      ;
    }

    virtual bool evaluate(const PointT& point) const {
      return (point.getVector3fMap() - centroid_).norm() < radius_;
    }
  };
  
  PointCloud::Ptr getSpherePointCloud(PointCloud::ConstPtr point_cloud, const Eigen::Vector3f& centroid, double radius) {
    SphereCondition<Point>::Ptr condition(new SphereCondition<Point>(centroid, radius));
  
    pcl::ConditionalRemoval<Point> conditional_removal;
    conditional_removal.setCondition(condition);
    conditional_removal.setInputCloud(point_cloud);
    
    PointCloud::Ptr sphere_point_cloud(new PointCloud());
    conditional_removal.filter(*sphere_point_cloud);
    
    return sphere_point_cloud;
  }

  PointCloud::Ptr removeIndicesFromPointCloud(PointCloud::ConstPtr point_cloud, pcl::PointIndices::ConstPtr point_indices, bool keep_organized = false) {
    pcl::ExtractIndices<Point> extract_indices;
    extract_indices.setInputCloud(point_cloud);
    extract_indices.setIndices(point_indices);
    extract_indices.setKeepOrganized(keep_organized);
    extract_indices.setNegative(true);

    PointCloud::Ptr indices_removed_point_cloud(new PointCloud());
    extract_indices.filter(*indices_removed_point_cloud);

    return indices_removed_point_cloud;
  }

  pcl::PointIndices::Ptr getFloorPointIndices(PointCloud::ConstPtr point_cloud) {
    if (point_cloud->size() == 0) {
      ROS_WARN_STREAM("Can't search plane more...");
      throw std::exception();
    }
    
    pcl::SACSegmentation<Point> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(0.025);
    segmentation.setInputCloud(point_cloud);

    pcl::ModelCoefficients::Ptr floor_model_coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr floor_point_indices(new pcl::PointIndices());
    segmentation.segment(*floor_point_indices, *floor_model_coefficients);

    if (floor_point_indices->indices.size() == 0) {
      ROS_WARN_STREAM("Can't find plane...");
      throw std::exception();
    }

    if (std::abs(floor_model_coefficients->values[0]) < 0.2 && std::abs(floor_model_coefficients->values[1]) < 0.2 && floor_model_coefficients->values[2] > 0.9 && std::abs(floor_model_coefficients->values[3] < 0.2)) {
      return floor_point_indices;
    }
    
    ROS_WARN_STREAM("Retry finding floor... Coefficients are " << floor_model_coefficients->values[0] << ", " << floor_model_coefficients->values[1] << ", " << floor_model_coefficients->values[2] << " and " << floor_model_coefficients->values[3]);
    return getFloorPointIndices(removeIndicesFromPointCloud(point_cloud, floor_point_indices, true));
  }

  PointCloud::Ptr removeFloorFromPointCloud(PointCloud::ConstPtr point_cloud) {
    return removeIndicesFromPointCloud(point_cloud, getFloorPointIndices(point_cloud));
  }

  Eigen::Vector4f getCentroidOfPointCloud(PointCloud::ConstPtr point_cloud) {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*point_cloud, centroid);
    
    return centroid;
  }
  
  unsigned long strtoul(const std::string& string) {
    char* e = 0;
    return std::strtoul(string.c_str(), &e, 10);
  }

  Eigen::Matrix4f registerPointCloud(PointCloud::ConstPtr source_point_cloud, PointCloud::ConstPtr target_point_cloud, double max_correspondence_distance) {
    pcl::IterativeClosestPointNonLinear<Point, Point> icp;
    icp.setInputSource(source_point_cloud);
    icp.setInputTarget(target_point_cloud);

    icp.setMaxCorrespondenceDistance(max_correspondence_distance);
    
    icp.setMaximumIterations(500);
    icp.setTransformationEpsilon(1.0e-9);
    
    // pcl::console::VERBOSITY_LEVEL verbosity_level = pcl::console::getVerbosityLevel();  // デバッグのために、ログ・レベルを設定します。
    // pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
    
    PointCloud::Ptr registered_point_cloud(new PointCloud());
    icp.align(*registered_point_cloud);

    // pcl::console::setVerbosityLevel(verbosity_level);  // ログ・レベルを元に戻します。
    
    if (!icp.hasConverged()) {
      ROS_WARN_STREAM("Can't register...");
      throw std::exception();
    }
    
    return icp.getFinalTransformation();
  }
  
  PointCloud::Ptr loadPointCloudFile(const std::string& file_path) {
    PointCloud::Ptr loaded_point_cloud(new PointCloud());
    pcl::io::loadPCDFile(file_path, *loaded_point_cloud);

    // ファイルにはROSヘッダーに相当する情報がないので、別途設定します。
    std_msgs::Header header;
    header.stamp = ros::Time().fromNSec(point_cloud_utilities::strtoul(file_path.substr(file_path.size() - 4 - 19, 19)));
    loaded_point_cloud->header = pcl_conversions::toPCL(header);
    
    return loaded_point_cloud;
  }

  std::string filePathString(PointCloud::ConstPtr point_cloud) {
    std::stringstream file_path_stringstream;
    file_path_stringstream << "/tmp/six_point_two_eight_" << std::setfill('0') << std::setw(19) << pcl_conversions::fromPCL(point_cloud->header).stamp.toNSec() << ".pcd";

    return file_path_stringstream.str();
  }
  
  PointCloud::Ptr savePointCloudFile(PointCloud::ConstPtr point_cloud) {
    std::string file_path_string = filePathString(point_cloud);
    
    pcl::io::savePCDFileBinary(file_path_string, *point_cloud);
    ROS_INFO_STREAM("PointCloud is saved to " << file_path_string);
    
    return PointCloud::Ptr(new PointCloud(*point_cloud));
  }

  PointCloud::Ptr removePointCloudFile(PointCloud::ConstPtr point_cloud) {
    std::remove(filePathString(point_cloud).c_str());

    return PointCloud::Ptr(new PointCloud(*point_cloud));
  }

  PointCloud::Ptr showPointCloud(PointCloud::ConstPtr point_cloud, const std::string& id, int rgb) {
    if (!visualizer_) {
      visualizer_.reset(new pcl::visualization::PCLVisualizer("Viewer"));
    }

    pcl::visualization::PointCloudColorHandlerCustom<Point> point_cloud_color(point_cloud, (rgb >> 16) & 0x00ff, (rgb >> 8) & 0x00ff, rgb & 0x00ff);
    if (!visualizer_->updatePointCloud(point_cloud, point_cloud_color, id)) {
      visualizer_->addPointCloud(point_cloud, point_cloud_color, id);
      visualizer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id);
    }
    
    return PointCloud::Ptr(new PointCloud(*point_cloud));
  }

  void spinVisualizer() {
    visualizer_->spinOnce(100);
  }
  
  // 型変換。

  PointCloud::Ptr fromROSMsg(sensor_msgs::PointCloud2ConstPtr points) {
    PointCloud::Ptr converted_point_cloud(new PointCloud());
    pcl::fromROSMsg(*points, *converted_point_cloud);
    
    return converted_point_cloud;
  }

  sensor_msgs::PointCloud2Ptr toROSMsg(PointCloud::ConstPtr point_cloud) {
    sensor_msgs::PointCloud2Ptr converted_points(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*point_cloud, *converted_points);
    
    return converted_points;
  }

  Eigen::Matrix4f fromROSMsg(const geometry_msgs::Transform& transform) {
    tf::Transform tf_transform;
    tf::transformMsgToTF(transform, tf_transform);

    Eigen::Matrix4f converted_transform;
    pcl_ros::transformAsMatrix(tf_transform, converted_transform);

    return converted_transform;
  }

  geometry_msgs::Transform toROSMsg(const Eigen::Matrix4f& transform) {
    tf::Transform tf_transform;
    tf::transformEigenToTF(Eigen::Affine3d(transform.cast<double>()), tf_transform);

    geometry_msgs::Transform converted_transform;
    tf::transformTFToMsg(tf_transform, converted_transform);

    return converted_transform;
  }
}

// インターフェース。

sensor_msgs::PointCloud2Ptr six_point_two_eight::downsamplePointCloud2(sensor_msgs::PointCloud2ConstPtr points, double leaf_size) {
  return point_cloud_utilities::toROSMsg(point_cloud_utilities::downsamplePointCloud(point_cloud_utilities::fromROSMsg(points), leaf_size));
}

geometry_msgs::Point six_point_two_eight::getCentroidOfPointCloud2(sensor_msgs::PointCloud2ConstPtr points) {
  Eigen::Vector4f centroid = point_cloud_utilities::getCentroidOfPointCloud(point_cloud_utilities::fromROSMsg(points));
  
  geometry_msgs::Point point;
  point.x = centroid.x();
  point.y = centroid.y();
  point.z = centroid.z();

  return point;
}

sensor_msgs::PointCloud2Ptr six_point_two_eight::getSpherePointCloud2(sensor_msgs::PointCloud2ConstPtr points, const geometry_msgs::Point& centroid, double radius) {
  return point_cloud_utilities::toROSMsg(point_cloud_utilities::getSpherePointCloud(point_cloud_utilities::fromROSMsg(points), Eigen::Vector3f(centroid.x, centroid.y, centroid.z), radius));
}

sensor_msgs::PointCloud2Ptr six_point_two_eight::loadPointCloud2File(const std::string& file_path) {
  return point_cloud_utilities::toROSMsg(point_cloud_utilities::loadPointCloudFile(file_path));
}

geometry_msgs::Transform six_point_two_eight::registerPointCloud2(sensor_msgs::PointCloud2ConstPtr source_points, sensor_msgs::PointCloud2ConstPtr target_points, double max_correspondence_distance) {
  return point_cloud_utilities::toROSMsg(point_cloud_utilities::registerPointCloud(point_cloud_utilities::fromROSMsg(source_points), point_cloud_utilities::fromROSMsg(target_points), max_correspondence_distance));
}

sensor_msgs::PointCloud2Ptr six_point_two_eight::removeFloorFromPointCloud2(sensor_msgs::PointCloud2ConstPtr points) {
  return point_cloud_utilities::toROSMsg(point_cloud_utilities::removeFloorFromPointCloud(point_cloud_utilities::fromROSMsg(points)));
}

sensor_msgs::PointCloud2Ptr six_point_two_eight::removePointCloud2File(sensor_msgs::PointCloud2ConstPtr points) {
  return point_cloud_utilities::toROSMsg(point_cloud_utilities::removePointCloudFile(point_cloud_utilities::fromROSMsg(points)));
}
    
sensor_msgs::PointCloud2Ptr six_point_two_eight::savePointCloud2File(sensor_msgs::PointCloud2ConstPtr points) {
  return point_cloud_utilities::toROSMsg(point_cloud_utilities::savePointCloudFile(point_cloud_utilities::fromROSMsg(points)));
}

sensor_msgs::PointCloud2Ptr six_point_two_eight::showPointCloud2(sensor_msgs::PointCloud2ConstPtr points, const std::string& id, int rgb) {
  return point_cloud_utilities::toROSMsg(point_cloud_utilities::showPointCloud(point_cloud_utilities::fromROSMsg(points), id, rgb));
}

void six_point_two_eight::spinVisualizer() {
  point_cloud_utilities::spinVisualizer();
}

sensor_msgs::PointCloud2Ptr six_point_two_eight::transformPointCloud2(sensor_msgs::PointCloud2ConstPtr points, const geometry_msgs::Transform& transform) {
  return point_cloud_utilities::toROSMsg(point_cloud_utilities::transformPointCloud(point_cloud_utilities::fromROSMsg(points), point_cloud_utilities::fromROSMsg(transform)));
}

sensor_msgs::PointCloud2Ptr six_point_two_eight::transformPointCloud2(sensor_msgs::PointCloud2ConstPtr points, const std::string& target_frame, double x_adjust) {
  return point_cloud_utilities::toROSMsg(point_cloud_utilities::transformPointCloud(point_cloud_utilities::fromROSMsg(points), target_frame, x_adjust));
}

// PCL1.7.2はC++14に対応していないので、C++03で書いています。地味にキツイです。。。
