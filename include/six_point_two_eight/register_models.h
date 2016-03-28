#pragma once

#include <boost/filesystem.hpp>
#include <boost/range.hpp>
#include <boost/range/adaptors.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/range/numeric.hpp>
#include <limits>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <thread>

#include <sensor_msgs/PointCloud2.h>

#include "six_point_two_eight/point_cloud_utilities.h"
#include "six_point_two_eight/utilities.h"

namespace six_point_two_eight {
  class RegisterModels : public nodelet::Nodelet {
  private:
    bool cut_next_round_;
    std::thread working_thread_;

    auto isSixPointTwoEightFile(const boost::filesystem::directory_entry& directory_entry) const {
      auto path = directory_entry.path();
      auto file_prefix = std::string("six_point_two_eight_");

      return path.filename().string().substr(0, file_prefix.size()) == file_prefix && path.extension().string() == ".pcd";
    }

    auto getNextRoundStartingIndex(const std::vector<sensor_msgs::PointCloud2ConstPtr>& point_clouds) const {
      auto next_round_starting_index = point_clouds.size();

      auto min_distance = std::numeric_limits<double>::max();
      for (auto i = point_clouds.size() / 2; i < point_clouds.size(); ++i) {
        auto centroid_1 = getCentroidOfPointCloud2(point_clouds[0]);
        auto centroid_2 = getCentroidOfPointCloud2(point_clouds[i]);

        auto distance = std::sqrt(std::pow(centroid_1.x - centroid_2.x, 2) + std::pow(centroid_1.y - centroid_2.y, 2));  // 上下には動かないので、zは無視します。
        if (distance < min_distance) {
          min_distance = distance;
          next_round_starting_index = i + 1;  // 計算対象は二周目の最初なので、+1します。
        }
      }

      return next_round_starting_index;
    }
    
    auto showPointCloud2(sensor_msgs::PointCloud2ConstPtr point_cloud) const {
      std::vector<int> colors{0x00ff0000, 0x00ffff00, 0x0000ffff, 0x000000ff, 0x00ff00ff};

      std::stringstream id;
      id << point_cloud->header.stamp;

      return six_point_two_eight::showPointCloud2(point_cloud, id.str(), colors[(point_cloud->header.stamp.toNSec() / 1000) & colors.size()]);
    }

    auto showPointCloud2s(const std::vector<sensor_msgs::PointCloud2ConstPtr>& point_clouds) {
      boost::for_each(point_clouds, std::bind(&RegisterModels::showPointCloud2, this, std::placeholders::_1));
    }
    
    auto processPointCloud2s(
      const std::vector<sensor_msgs::PointCloud2ConstPtr>& point_clouds,
      const std::string& caption,
      std::function<void(std::vector<sensor_msgs::PointCloud2ConstPtr>*, const sensor_msgs::PointCloud2ConstPtr&, int)> op) const
    {
      int index = 1;  // Ubuntu14.04のBoostのバージョンは1.54で、1.54のboost::adaptors::indexedが使いづらいので、変数管理します。
      
      return
        boost::accumulate(
          point_clouds | boost::adaptors::sliced(1, boost::distance(point_clouds)),  // 処理するのは、2番目の点群から。
          std::vector<sensor_msgs::PointCloud2ConstPtr>{point_clouds.front()},       // 1番目の点群の処理は不要。
          [&](auto& processed_point_clouds, const auto& point_cloud) {
            ROS_INFO_STREAM(caption << " " << index++ << "/" << (point_clouds.size() - 1) << ".");
            
            op(&processed_point_clouds, point_cloud, index);

            this->showPointCloud2(processed_point_clouds.back());
            spinVisualizer();
            
            return processed_point_clouds;
          });
    }

    auto adjustPointCloud2s(const std::vector<sensor_msgs::PointCloud2ConstPtr>& point_clouds, double max_correspondence_distance) const {
      ROS_INFO_STREAM("Calculating transform for adjusting.");
      auto adjusting_tf_transform = transformMsgToTF(registerPointCloud2(point_clouds.back(), point_clouds.front(), max_correspondence_distance));  // 最終的に補正しなければならない誤差を計算します。
      
      return
        processPointCloud2s(
          point_clouds, "Adjust",
          [&](auto* adjusted_point_clouds, const auto& point_cloud, const auto& index) {
            adjusted_point_clouds->push_back(
              transformPointCloud2(
                point_cloud,
                transformTFToMsg(
                  tf::Transform(
                    tf::Quaternion::getIdentity().slerp(  // 四元数を補間して、解消すべき誤差を計算します。
                      adjusting_tf_transform.getRotation(),
                      static_cast<double>(index) / static_cast<double>(point_clouds.size() - 1)),
                    adjusting_tf_transform.getOrigin() * index / (point_clouds.size() - 1)))));
          });
    }
    
    auto registerPointCloud2s(const std::vector<sensor_msgs::PointCloud2ConstPtr>& point_clouds, double max_correspondence_distance) const {
      return
        processPointCloud2s(
          point_clouds, "Register",
          [&](auto* registered_point_clouds, const auto& point_cloud, const auto& index) {
            registered_point_clouds->push_back(
              transformPointCloud2(
                point_cloud,
                registerPointCloud2(point_cloud, registered_point_clouds->back(), max_correspondence_distance)));
          });
    }

    auto registerAndAdjustPointCloud2s(const std::vector<sensor_msgs::PointCloud2ConstPtr>& point_clouds) const {
      return
        adjustPointCloud2s(          // 5. レジストレーションの誤差を分散。
          registerPointCloud2s(      // 4. 細かくレジストレーション。
            adjustPointCloud2s(      // 3. レジストレーションの誤差を分散。
              registerPointCloud2s(  // 2. 粗くレジストレーション
                adjustPointCloud2s(  // 1. オドメトリーの誤差を分散。
                  point_clouds,
                  0.1),
                0.1),
              0.1),
            0.05),
          0.05);
    }
    
  public:
    void onInit() {
      getPrivateNodeHandle().getParam("cut_next_round", cut_next_round_);
      
      working_thread_ = std::thread(
        [&]() {
          std::vector<std::string> six_point_two_eight_file_paths;  // boost::sort()はRandom Access Rangeを要求するので、std::vectorに変換します。
          boost::copy(
            boost::make_iterator_range(boost::filesystem::directory_iterator(boost::filesystem::path("/tmp")), {}) |
            boost::adaptors::filtered(std::bind(&RegisterModels::isSixPointTwoEightFile, this, std::placeholders::_1)) |
            boost::adaptors::transformed([](const auto& directory_entry) { return directory_entry.path().string(); }),
            std::back_inserter(six_point_two_eight_file_paths));
          
          boost::sort(six_point_two_eight_file_paths);
          
          std::vector<sensor_msgs::PointCloud2ConstPtr> point_clouds;  // 後の処理で複数回使用したいので、std::vectorに変換しておきます。
          boost::copy(
            six_point_two_eight_file_paths | boost::adaptors::transformed(loadPointCloud2File),
            std::back_inserter(point_clouds));
          
          if (cut_next_round_) {
            auto next_round_starting_index = getNextRoundStartingIndex(point_clouds);

            std::vector<sensor_msgs::PointCloud2ConstPtr> this_round_point_clouds;
            boost::copy(
              point_clouds | boost::adaptors::sliced(0, next_round_starting_index),
              std::back_inserter(this_round_point_clouds));

            // 無関係なファイルが残っていると混乱するので、削除します。
            boost::for_each(
              point_clouds | boost::adaptors::sliced(next_round_starting_index, boost::distance(point_clouds)),
              removePointCloud2File);

            point_clouds = this_round_point_clouds;
          }

          // とりあえず、表示します。
          this->showPointCloud2s(point_clouds);
          spinVisualizer();

          // 誤差の修正とレジストレーションを実施し、結果を保存します。
          std::vector<sensor_msgs::PointCloud2ConstPtr> registered_and_adjusted_point_clouds;
          boost::copy(
            this->registerAndAdjustPointCloud2s(point_clouds) |
            boost::adaptors::transformed(savePointCloud2File),
            std::back_inserter(registered_and_adjusted_point_clouds));

          // 結果を表示します。registerAndAdjustPointCloud2sの中で表示されているので不要なのですけど、念の為。
          this->showPointCloud2s(registered_and_adjusted_point_clouds);
          while (ros::ok()) {
            spinVisualizer();
          }
        });
    }
  };
}
