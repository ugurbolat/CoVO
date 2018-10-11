#pragma once

#include "Feature.h"
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

typedef std::vector<cv::Point3f> PointMatrixXYZ;
typedef std::vector<cv::Point3f> PointMatrixUVD;

namespace covo
{
  class Matcher
  {
    public:
      Matcher(
          const nlohmann::json&, 
          const cv::Mat&,
          const cv::Mat&,
          const cv::Mat&,
          const cv::Mat&,
          const covo::Feature&,
          const covo::Feature&
          );
      void findMatches();
      void filterOutliers();
      const std::string getCovoMatcherSettings() const;
      void drawRawMatches() const;
      void drawRawMatches(const std::string&) const;
      void drawInlierMatches() const;
      void drawInlierMatches(const std::string&) const;
      void drawOutlierMatches() const;
      void drawOutlierMatches(const std::string&) const;
      bool isSufficientNoMatches() const;
      PointMatrixXYZ getXyz1() const;
      PointMatrixUVD getUvd1() const;
      PointMatrixXYZ getXyz2() const;
      PointMatrixUVD getUvd2() const;

    private:
      cv::Point3f cloudifyUV2XYZ(const cv::Point2f&, const cv::Mat&);
      cv::Point3f convertUV2UVD(const cv::Point2f&, const cv::Mat&);

    private:
      const nlohmann::json& COVO_SETTINGS;
      const cv::Mat& rgbImg1;
      const cv::Mat& depthImg1;
      const cv::Mat& rgbImg2;
      const cv::Mat& depthImg2;
      const covo::Feature& feature1;
      const covo::Feature& feature2;
      std::vector<cv::DMatch> matches;
      std::vector<cv::DMatch> inlierMatches;
      std::vector<cv::DMatch> outlierMatches;

      PointMatrixXYZ xyz1;
      PointMatrixUVD uvd1;
      PointMatrixXYZ xyz2;
      PointMatrixUVD uvd2;
  };

}
