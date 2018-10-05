#pragma once

#include "Feature.h"
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>


namespace covo
{
  class Matcher
  {
    public:
      Matcher(
          const nlohmann::json&, 
          const covo::Feature&,
          const covo::Feature&
          //const std::vector<cv::KeyPoint>&,
          //const std::vector<cv::KeyPoint>&,
          //const std::Mat&,
          //const std::Mat&
          );
      void findMatches();
      void filterOutliers();
      const std::string getCovoMatcherSettings() const;
      void drawMatches() const;
    private:
      const nlohmann::json COVO_SETTINGS;
      const covo::Feature feature1;
      const covo::Feature feature2;
      std::vector<cv::DMatch> matches;
  };

}
