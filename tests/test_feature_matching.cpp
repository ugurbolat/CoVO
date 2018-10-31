#include "ReadArgs.h"
#include "Feature.h"
#include "Matcher.h"
#include <nlohmann/json.hpp>
#include <arg/args.hxx>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>


using json = nlohmann::json;


int main(int argc, char** argv)
{
  json covo_settings;
  std::vector<std::array<std::string, 2>> rgbdImgFiles;
  std::vector<std::array<double, 2>> rgbdImgTimestamps;

  // Reading user arguments
  if(read_args(argc, argv, &covo_settings, &rgbdImgFiles, &rgbdImgTimestamps))
    spdlog::get("console")->info("Reading args is successful");
  else
  {
    spdlog::get("console")->critical("Reading args FAILED");
    return 0;
  }


  // Adding const-ness to global settings
  const json& COVO_SETTINGS = covo_settings;


  // Extracting features from RGB images
  spdlog::get("console")->info("Reading 10 rgb&depth images from dataset");
  for (int i = 0; i < 500; i++)
  {
    // extract features from img 1
    cv::String rgbImg1(rgbdImgFiles.at(0)[0]);
    cv::String depthImg1(rgbdImgFiles.at(0)[1]);
    cv::Mat rgbImgMat1, depthImgMat1;
    depthImgMat1 = cv::imread(depthImg1, cv::IMREAD_ANYDEPTH);
    //std::cout << cv::format(depthImgMat1, cv::Formatter::FMT_CSV) << std::endl;
    rgbImgMat1 = cv::imread(rgbImg1, cv::IMREAD_GRAYSCALE);
    if (rgbImg1.empty())
    {
      spdlog::get("console")->critical("Could not open or find the rgb image");
    }
    spdlog::get("console")->debug("Reading {} RGB image", rgbdImgTimestamps.at(0)[0]);

    covo::Feature feature1(COVO_SETTINGS, rgbImgMat1);
    feature1.detectFeatures();

    // extract features from img 2
    cv::String rgbImg2(rgbdImgFiles.at(i+1)[0]);
    cv::String depthImg2(rgbdImgFiles.at(i+1)[1]);
    cv::Mat rgbImgMat2, depthImgMat2;
    depthImgMat2 = cv::imread(depthImg2, cv::IMREAD_ANYDEPTH);
    rgbImgMat2 = cv::imread(rgbImg2, cv::IMREAD_GRAYSCALE);
    if (rgbImg2.empty())
    {
      spdlog::get("console")->critical("Could not open or find the rgb image");
    }
    spdlog::get("console")->debug("Reading {} RGB image", rgbdImgTimestamps.at(i+1)[0]);

    covo::Feature feature2(COVO_SETTINGS, rgbImgMat2);
    feature2.detectFeatures();

    // matching features
    covo::Matcher matcher(COVO_SETTINGS, 
        rgbImgMat1, depthImgMat1, 
        rgbImgMat2, depthImgMat2, 
        feature1, feature2);
    matcher.findMatches();
    matcher.drawRawMatches();
    matcher.filterOutliers();

    if (!matcher.isSufficientNoMatches())
    {
      spdlog::get("console")->error("Stopping due to insufficient no of matches");
      return 0;
    }

  }

  return 1;
}
