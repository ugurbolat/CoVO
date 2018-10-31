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
  for (int i = 0; i < 10; i++)
  {
    cv::String rgbImg(rgbdImgFiles.at(i)[0]);
    cv::Mat rgbImgMat;
    rgbImgMat = cv::imread(rgbImg, cv::IMREAD_GRAYSCALE);
    if (rgbImg.empty())
    {
      spdlog::get("console")->critical("Could not open or find the rgb image");
    }
    spdlog::get("console")->debug("Reading {} RGB image", rgbdImgTimestamps.at(i)[0]);

    covo::Feature feature(COVO_SETTINGS, rgbImgMat);
    feature.detectFeatures();
    feature.drawFeatureDescriptor();

    cv::String depthImg(rgbdImgFiles.at(i)[1]);
    cv::Mat depthImgMat;
    depthImgMat = cv::imread(depthImg, cv::IMREAD_GRAYSCALE);
    if (depthImg.empty())
    {
      spdlog::get("console")->critical("Could not open or find the depth image");
    }
    spdlog::get("console")->debug("Reading {} depth image", rgbdImgTimestamps.at(i)[1]);
  }

  return 0;
}
