#include "Feature.h"
#include <nlohmann/json.hpp>
#include <args.hxx>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>


using json = nlohmann::json;

covo::Feature::Feature(const json& _COVO_SETTINGS, const cv::Mat& _img) :
  COVO_SETTINGS(_COVO_SETTINGS), img(_img) {};

void covo::Feature::detectFeatures()
{
  if (COVO_SETTINGS["feature_descriptor_choice"] == "ORB")
    covo::Feature::detectOrbFeatures();
  // TODO add other feature descriptor type, e.g., FAST, BRIEF...
  else
    spdlog::get("console")->error("Invalid feature descriptor choice."
        " Default value is set");
  covo::Feature::detectOrbFeatures();
}

// TODO add a flag arg to enable/disable drawing
void covo::Feature::drawFeatureDescriptor()
{
  if (!keyPoints.empty())
  {
    cv::Mat imgKeyPoints;
    cv::drawKeypoints(img, keyPoints, imgKeyPoints, cv::Scalar::all(-1),
        cv::DrawMatchesFlags::DEFAULT);
    cv::imshow("Key Points", imgKeyPoints);
    cv::waitKey(COVO_SETTINGS["wait_key_settings"]);
  }
  else
  {
    spdlog::get("console")->error("There is no key points to draw!");
  }
}

const std::vector<cv::KeyPoint> covo::Feature::getKeyPoints() const
{
  return keyPoints;
}

const cv::Mat covo::Feature::getDescriptors() const
{
  return descriptors;
}

bool covo::Feature::isSufficientKeyPoints() const
{
  if (keyPoints.size() < 
      COVO_SETTINGS["covo_settings"]["insufficient_n_feature_threshold"])
  {
    spdlog::get("console")->error("Insufficient no of key points!");
    return false;
  }
  else
    return true;
}

void covo::Feature::detectOrbFeatures()
{
  int n_features = 500;
  if (COVO_SETTINGS["feature_settings"]["n_features"] != "DEFAULT")
    n_features = COVO_SETTINGS["feature_settings"]["n_features"];
  float scale_factor = 1.2f;
  if (COVO_SETTINGS["feature_settings"]["scale_factor"] != "DEFAULT")
    scale_factor = COVO_SETTINGS["feature_settings"]["scale_factor"];
  int n_levels = 8;
  if (COVO_SETTINGS["feature_settings"]["n_levels"] != "DEFAULT")
    n_levels = COVO_SETTINGS["feature_settings"]["n_levels"];
  int edge_threshold = 31;
  if (COVO_SETTINGS["feature_settings"]["edge_threshold"] != "DEFAULT")
    edge_threshold = COVO_SETTINGS["feature_settings"]["edge_threshold"];
  int first_level = 0;
  if (COVO_SETTINGS["feature_settings"]["first_level"] != "DEFAULT")
    first_level = COVO_SETTINGS["feature_settings"]["first_level"];
  int wta_k = 2;
  if (COVO_SETTINGS["feature_settings"]["wta_k"] != "DEFAULT")
      wta_k = COVO_SETTINGS["feature_settings"]["wta_k"];
  int score_type = cv::ORB::HARRIS_SCORE;
  if (COVO_SETTINGS["feature_settings"]["score_type"] != "DEFAULT")
  {
    if (COVO_SETTINGS["feature_settings"]["score_type"] == "HARRIS_SCORE")
      score_type = cv::ORB::HARRIS_SCORE;
    else if (COVO_SETTINGS["feature_settings"]["score_type"] == "FAST_SCORE")
      score_type = cv::ORB::FAST_SCORE;
    else
      spdlog::get("console")->error("Invalid ORB score type in "
          "SETTINGS.json. Default value is set!");
  }
  int patch_size = 31;
  if (COVO_SETTINGS["feature_settings"]["patch_size"] != "DEFAULT")
      patch_size = COVO_SETTINGS["feature_settings"]["patch_size"];
  int fast_threshold;
  if (COVO_SETTINGS["feature_settings"]["fast_threshold"] != "DEFAULT")
      patch_size = COVO_SETTINGS["feature_settings"]["patch_size"];


  cv::Ptr<cv::Feature2D> orb = cv::ORB::create(
      n_features,
      scale_factor,
      n_levels,
      edge_threshold,
      first_level,
      wta_k,
      score_type,
      patch_size,
      fast_threshold
      );
  orb->detectAndCompute(img, cv::Mat(), keyPoints, descriptors);
}

const cv::Mat covo::Feature::getImage() const
{
  return img;
}

