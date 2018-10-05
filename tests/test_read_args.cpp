#include "ReadArgs.h"
#include <nlohmann/json.hpp>
#include <args.hxx>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>


using json = nlohmann::json;


int main(int argc, char** argv)
{
  json COVO_SETTINGS;
  std::vector<std::array<std::string, 2>> rgbdImgFiles;
  std::vector<std::array<double, 2>> rgbdImgTimestamps;

  if(read_args(argc, argv, &COVO_SETTINGS, &rgbdImgFiles, &rgbdImgTimestamps))
    spdlog::get("console")->info("Reading args is successful");
  else
  {
    spdlog::get("console")->critical("Reading args FAILED");
    return 0;
  }

  spdlog::get("console")->info("Reading 10 rgb&depth images from dataset");
  for (int i = 0; i < 10; i++)
  {
    cv::String rgbImg(rgbdImgFiles.at(i)[0]);
    cv::Mat img_1;
    img_1 = cv::imread(rgbImg, cv::IMREAD_GRAYSCALE);
    if (rgbImg.empty())
    {
      spdlog::get("console")->critical("Could not open or find the rgb image");
    }
    spdlog::get("console")->debug("Reading {} RGB image", rgbdImgTimestamps.at(i)[0]);
    cv::imshow("Display RGB image", img_1);

    cv::String depthImg(rgbdImgFiles.at(i)[1]);
    cv::Mat img_2;
    img_2 = cv::imread(depthImg, cv::IMREAD_GRAYSCALE);
    if (depthImg.empty())
    {
      spdlog::get("console")->critical("Could not open or find the depth image");
    }
    spdlog::get("console")->debug("Reading {} depth image", rgbdImgTimestamps.at(i)[1]);
    cv::imshow("Display disparsity image", img_2);

    cv::waitKey(COVO_SETTINGS["wait_key_settings"]);
  }
  return 1;
}

