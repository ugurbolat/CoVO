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
    const covo::Feature& _feature1,
    const covo::Feature& _feature2) : 
  COVO_SETTINGS(_COVO_SETTINGS), feature1(_feature1), feature2(_feature2) {}


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
  cv::Mat inlier_mask, homography;
  std::vector<cv::KeyPoint> inliers1, inliers2;
  std::vector<cv::DMatch> inlier_matches;

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


void covo::Matcher::drawMatches() const
{
  cv::Mat imgMatches;
  cv::drawMatches(
      feature1.getImage(), feature1.getKeyPoints(),
      feature2.getImage(), feature2.getKeyPoints(),
      matches,
      imgMatches
      );
  cv::imshow("Key Point Matches", imgMatches);
  cv::waitKey(COVO_SETTINGS["wait_key_settings"]);
}
