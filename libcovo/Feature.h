#pragma once

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

namespace covo
{
  class Feature{
    public:
      Feature(
          const nlohmann::json& _COVO_SETTINGS, 
          const cv::Mat& _img
          );
      void detectFeatures();
      void drawFeatureDescriptor();
      const std::vector<cv::KeyPoint> getKeyPoints() const;
      const cv::Mat getDescriptors() const;
      bool isSufficientNoKeyPoints() const;
      const cv::Mat getImage() const;
    private:
      const nlohmann::json& COVO_SETTINGS;
      const cv::Mat& img;
      std::vector<cv::KeyPoint> keyPoints;
      cv::Mat descriptors;
      void detectOrbFeatures();
  };
}
