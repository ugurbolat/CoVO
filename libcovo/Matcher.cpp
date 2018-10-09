#include "Matcher.h"
#include "Feature.h"

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/features2d.hpp"

using json = nlohmann::json;

covo::Matcher::Matcher(
    const json& _COVO_SETTINGS, 
    const cv::Mat& _rgbImg1, const cv::Mat& _depthImg1,
    const cv::Mat& _rgbImg2, const cv::Mat& _depthImg2, 
    const covo::Feature& _feature1,
    const covo::Feature& _feature2) : 
  COVO_SETTINGS(_COVO_SETTINGS), 
  rgbImg1(_rgbImg1), depthImg1(_depthImg1),
  rgbImg2(_rgbImg2), depthImg2(_depthImg2),
  feature1(_feature1), feature2(_feature2) {}


void covo::Matcher::findMatches()
{

  cv::Ptr<cv::DescriptorMatcher> matcher = 
    cv::DescriptorMatcher::create(getCovoMatcherSettings());

  matcher->match(
      feature1.getDescriptors(), 
      feature2.getDescriptors(), 
      matches, 
      cv::Mat());

  // sort matches based on their scores
  std::sort(matches.begin(), matches.end());

  // remove matches having low scores
  const int& filterScorePercentage = 
    COVO_SETTINGS["feature_settings"]["filter_score_percentage"];
  const int nGoodMatches = matches.size() * filterScorePercentage / 100;
  matches.erase(matches.begin()+nGoodMatches, matches.end());
}

void covo::Matcher::filterOutliers()
{
  cv::Mat inlierMask, homography;
  //std::vector<cv::KeyPoint> inliers1, inliers2;
  std::vector<cv::DMatch> inlierMatches;
  std::vector<cv::DMatch> outlierMatches;


  std::vector<cv::Point2f> pixelPoint1, pixelPoint2;
  spdlog::get("console")->debug("No of matches before RANSAC: {}", matches.size());
  for (auto kp : matches)
  {
    pixelPoint1.push_back(feature1.getKeyPoints()[kp.queryIdx].pt);
    pixelPoint2.push_back(feature2.getKeyPoints()[kp.trainIdx].pt);
  }
  if (matches.size() >= 4)
    homography = cv::findHomography(
        pixelPoint1, pixelPoint2, 
        cv::RANSAC,
        COVO_SETTINGS["covo_settings"]["ransac_threshold"],
        inlierMask);
  else
    spdlog::get("console")->error("Cannot apply RANSAC due to insufficient no of matches!");
  if (!homography.empty())
  {
    // filter outliers
    for (unsigned int i = 0; i < matches.size(); i++)
    {
      if (inlierMask.at<uchar>(i))
      {
        inlierMatches.push_back(std::move(matches[i]));
        const cv::Point2f& uv1 = feature1.getKeyPoints()[matches[i].queryIdx].pt;
        const cv::Point2f& uv2 = feature2.getKeyPoints()[matches[i].trainIdx].pt;
        cv::Point3f pointXyz1 = covo::Matcher::cloudifyUV2XYZ(uv1, depthImg1);
        cv::Point3f pointXyz2 = covo::Matcher::cloudifyUV2XYZ(uv2, depthImg2);
        if (pointXyz1.z < COVO_SETTINGS["covo_settings"]["depth_near_point_cloud_threshold_in_m"] || 
            pointXyz1.z > COVO_SETTINGS["covo_settings"]["depth_far_point_cloud_threshold_in_m"] || 
            pointXyz2.z < COVO_SETTINGS["covo_settings"]["depth_near_point_cloud_threshold_in_m"] || 
            pointXyz2.z > COVO_SETTINGS["covo_settings"]["depth_far_point_cloud_threshold_in_m"])
        {
          spdlog::get("console")->trace("Invalid depth. Removing from matches..."); 
        }
        else
        {
          xyz1.push_back(pointXyz1);
          xyz2.push_back(pointXyz2);
          uvd1.push_back(covo::Matcher::convertUV2UVD(uv1, depthImg1));
          uvd2.push_back(covo::Matcher::convertUV2UVD(uv2, depthImg2));
          spdlog::get("console")->trace("Good point clouds 1: {}, {}, {}", 
              pointXyz1.x, pointXyz1.y, pointXyz1.z);
          spdlog::get("console")->trace("Good point clouds 2: {}, {}, {}", 
              pointXyz2.x, pointXyz2.y, pointXyz2.z);
        }
      }
      else
      {
        outlierMatches.push_back(std::move(matches[i]));
      }
    }
    spdlog::get("console")->debug("No of matches after RANSAC: {}", inlierMatches.size());
    spdlog::get("console")->debug("No of matches after depth filtering: {}", xyz1.size());
  }
  else
    spdlog::get("console")->error("Homography mask is empty!");

  cv::Mat imgMatches;
  cv::drawMatches(
      feature1.getImage(), feature1.getKeyPoints(),
      feature2.getImage(), feature2.getKeyPoints(),
      inlierMatches,
      imgMatches
      );
  cv::imshow("Inlier Matches", imgMatches);
  cv::waitKey(COVO_SETTINGS["wait_key_settings"]);

  cv::Mat imgMatches_;
  cv::drawMatches(
      feature1.getImage(), feature1.getKeyPoints(),
      feature2.getImage(), feature2.getKeyPoints(),
      outlierMatches,
      imgMatches_
      );
  cv::imshow("Outlier Matches", imgMatches_);
  cv::waitKey(COVO_SETTINGS["wait_key_settings"]);


}

cv::Point3f covo::Matcher::cloudifyUV2XYZ(const cv::Point2f& pix, const cv::Mat& depthImg)
{
  auto& fx = COVO_SETTINGS["camera_settings"]["fx"];
  auto& fy = COVO_SETTINGS["camera_settings"]["fy"];
  auto& cx = COVO_SETTINGS["camera_settings"]["cx"];
  auto& cy = COVO_SETTINGS["camera_settings"]["cy"];
  auto& scale = COVO_SETTINGS["camera_settings"]["scale"];

  float d = depthImg.at<ushort>(ushort(pix.x), ushort(pix.y)) / float(scale) ;
  //spdlog::get("console")->trace("depth {}", depthImg.at<ushort>(ushort(pix.x), ushort(pix.y)));
  cv::Point3f p;
  p.x = (pix.x - float(cx)) * d / float(fx);
  p.y = (pix.y - float(cy)) * d / float(fy);
  p.z = d;

  return p;
}

cv::Point3f covo::Matcher::convertUV2UVD(const cv::Point2f& pix, const cv::Mat& depthImg)
{
  auto& scale = COVO_SETTINGS["camera_settings"]["scale"];
  float d = depthImg.at<ushort>(ushort(pix.x), ushort(pix.y)) / float(scale) ;
  //TODO what's up with this warning?
  cv::Point3f p(p.x, p.y, d);
  return p;
}

bool covo::Matcher::isSufficientNoMatches() const
{
  if (xyz1.size() < 
      COVO_SETTINGS["covo_settings"]["insufficient_n_feature_threshold"])
  {
    spdlog::get("console")->error("Insufficient no of key points!");
    return false;
  }
  else
    return true;
}

const std::string covo::Matcher::getCovoMatcherSettings() const
{
  // TODO double-check naming convention
  if (COVO_SETTINGS["matcher_settings"] == "FLANNBASED")
    return "Flann-Based";
  if (COVO_SETTINGS["matcher_settings"] == "BRUTEFORCE")
    return "BruteForce";
  if (COVO_SETTINGS["matcher_settings"] == "BRUTEFORCE_L1")
    return "BruteForce-L1";
  if (COVO_SETTINGS["matcher_settings"] == "BRUTEFORCE_HAMMING")
    return "BruteForce-Hamming";
  if (COVO_SETTINGS["matcher_settings"] == "BRUTEFORCE_HAMMINGLUT")
    return "BruteForce-HammingLut";
  if (COVO_SETTINGS["matcher_settings"] == "BRUTEFORCE_SL2")
    return "BruteForce-Sl2";
  else
  {
    spdlog::get("console")->error("Invalid matcher settings type! "
        "Default set to BruteForce.");
    return "BruteForce";
  }
}


void covo::Matcher::drawMatches(const std::string& windowTitle="Matches") const
{
  cv::Mat imgMatches;
  cv::drawMatches(
      feature1.getImage(), feature1.getKeyPoints(),
      feature2.getImage(), feature2.getKeyPoints(),
      matches,
      imgMatches
      );
  cv::imshow(windowTitle, imgMatches);
  cv::waitKey(COVO_SETTINGS["wait_key_settings"]);
}
