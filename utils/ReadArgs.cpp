#include "ReadArgs.h"

#include <stdio.h>
#ifdef WINDOWS
#include <direct.h>
#define GetCurrentDir _getcwd
#else
#include <unistd.h>
#define GetCurrentDir getcwd
#endif

#include <iostream>
#include <string>
#include <fstream>
#include <nlohmann/json.hpp>
#include <arg/args.hxx>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

using json = nlohmann::json;

LOG_LEVEL hasLogLevel(std::string const& in)
{
  if (in == "trace") return trace;
  if (in == "debug") return debug;
  if (in == "info") return info;
  if (in == "warn") return warn;
  if (in == "error") return error;
  if (in == "critical") return critical;
  spdlog::get("console")->warn("Retrieving invalid LOG_LEVEL");
  return none;
}

std::string getCurrentWorkingDir( void ) {
  char buff[FILENAME_MAX];
  GetCurrentDir( buff, FILENAME_MAX );
  std::string current_working_dir(buff);
  return current_working_dir;
}

int read_args(int argc, char** argv,
    json* COVO_SETTINGS,
    std::vector<std::array<std::string, 2>>* rgbdImgFiles,
    std::vector<std::array<double, 2>>* rgbdImgTimestamps) {

  args::ArgumentParser parser("Welcome to CoVO!"
      "\nCoVO is an Uncertainty-Aware RGB-D Visual Odometry Tool"
      "\nFor more information visit https://github.com/ugurbolat/CoVO");
  args::HelpFlag help(parser, "help", "Display help menu", {'h', "help"});
  args::ValueFlag<std::string> settings_file(parser,
      "/path/to/SETTINGS.json",
      "Configuration file of CoVO's settings; e.g., camera intrinsics, sensor noise model etc."
      "\nDefault SETTINGS.json located in params_settings folder",
      {'s', "settings"});
  args::ValueFlag<std::string> dataset_root_dir(parser,
      "/path/to/dataset_dir",
      "Dataset root directory."
      "\nDefault dataset located in docs folder",
      {'d', "dataset_dir"});
  args::ValueFlag<std::string> log_level(parser,
      "trace, debug, info, warn, error, critical",
      "Log level settings. \nDefault is at info level",
      {'l', "log_level"});
  args::ValueFlag<std::string> output_dir(parser,
      "/path/to/output_dir/",
      "Output directory in which CoVO results will be saved"
      "\nDefault is './'",
      {'o', "output_dir"});
  args::ValueFlag<std::string> no_img_pairs_to_process(parser,
      "1,2,...,all",
      "No of image pairs to be processed. An integer or all can be entered"
      "\nDefault is all",
      {'i',"no_imgs"});
  args::Flag draw_matches(parser,
      "draw-matches",
      "Enable flag for drawing matches on image pairs"
      "\nDefault is disabled",
      {"draw-matches"});
  args::ValueFlag<int> wait_key_delay(parser, "delay in ms",
      "Delay btw image readings. "
      "This functionality is only available if draw_matches is enabled"
      "\nDefault is 0 which means it will wait indefinitely for key press to process next image pairs",
      {'t', "time_delay"});

  // parsing args
  try
  {
    parser.ParseCLI(argc, argv);
  }
  catch (args::Help)
  {
    std::cout << parser;
    return 0;
  }
  catch (args::ParseError e)
  {
    std::cerr << e.what() << std::endl;
    std::cerr << parser;
    return 0;
  }
  catch (args::ValidationError e)
  {
    std::cerr << e.what() << std::endl;
    std::cerr << parser;
    return 0;
  }


  // setting global log level
  auto console = spdlog::stdout_color_mt("console");
  spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%F] [%^%l%$] %v");
  if (log_level)  // user's choice
  {
    switch (hasLogLevel(args::get(log_level)))
    {
      case trace:
        spdlog::set_level(spdlog::level::trace); break;
      case debug:
        spdlog::set_level(spdlog::level::debug); break;
      case info:
        spdlog::set_level(spdlog::level::info); break;
      case warn:
        spdlog::set_level(spdlog::level::warn); break;
      case error:
        spdlog::set_level(spdlog::level::err); break;
      case critical:
        spdlog::set_level(spdlog::level::critical); break;
      default:
        console->critical("Unknown log level choice");
        return 0;
    }
  }
  else  // default choice
  {
    spdlog::set_level(spdlog::level::info);
  }


  // Reading folder&file path stuff
  std::string projectRootFolder;
  std::string settingsFilePath;
  std::string datasetRootFolder;

  std::string binPath = getCurrentWorkingDir();
  std::size_t found = binPath.find("CoVO");
  if (found != std::string::npos)
  {
    console->info("Your project root folder found automatically: " + binPath.substr(0, found+5));
  }
  else
  {
    console->error("Could not find your project root folder. "
        "Sample dataset and SETTINGS.json will not work properly");
  }

  projectRootFolder = binPath.substr(0, found+5);
  settingsFilePath = projectRootFolder + "param_settings/SETTINGS.json";
  datasetRootFolder = projectRootFolder + "docs/sample_dataset/";

  std::ifstream ifSettings;
  if (settings_file)  // getting settings.json from user
  {
    console->info("Reading settings JSON file from user");
    ifSettings.open(args::get(settings_file));
    if (ifSettings.fail())
    {
      console->critical("Cannot read the settings file");
      return 0;
    }
    ifSettings >> *COVO_SETTINGS;
    console->debug(COVO_SETTINGS->dump(2));
  }
  else  // default path used for settings.json
  {
    console->info("Reading default settings JSON file");
    ifSettings.open(settingsFilePath);
    if (ifSettings.fail())
    {
      console->critical("Cannot read the default settings file located in " +
          settingsFilePath);
      return 0;
    }
    ifSettings >> *COVO_SETTINGS;
    console->debug(COVO_SETTINGS->dump(2));
  }
  ifSettings.close();

  // adding wait key delay settings to COVO_SETTINGS
  int waitKeySettings;
  if (wait_key_delay)
  {
    waitKeySettings = args::get(wait_key_delay);
    console->info("User set image reading delay from user to {0:d}",
        waitKeySettings);
  }
  else
  {
    waitKeySettings = 0;
    console->info("Default image reading delay is setted to {0:d}",
        waitKeySettings);
  }
  (*COVO_SETTINGS)["wait_key_settings"] = waitKeySettings;

  // adding log level settings to COVO_SETTINGS
  (*COVO_SETTINGS)["log_level"] = args::get(log_level);

  // getting project related paths
  if (dataset_root_dir)
  {
    console->info("Getting dataset root directory from user");
    datasetRootFolder = args::get(dataset_root_dir);
  }
  else
  {
    console->info("Getting default dataset root directory");
  }

  // adding draw-matches enabled settings to COVO_SETTINGS
  if (draw_matches)
  {
    console->info("Enabling draw-matches");
    (*COVO_SETTINGS)["draw_matches"] = true;
  }
  else
  {
    console->info("Disabling draw-matches");
    (*COVO_SETTINGS)["draw_matches"] = false;
  }

  // adding output_dir to COVO_SETTINGS
  std::string outputDir;
  if (output_dir)
  {
    console->info("Getting output directory from user");
    if (args::get(output_dir).back() != '/')
    {
      outputDir = args::get(output_dir) + '/';
    }
    else
    {
      outputDir = args::get(output_dir);
    }
  }
  else
  {
    console->info("Getting default output directory");
    outputDir = "./";
  }
  (*COVO_SETTINGS)["output_dir"] = outputDir;

  // adding no of imgs to process to COVO_SETTINGS
  int noImg;
  if (no_img_pairs_to_process)
  {
    console->info("Getting no of img pairs to process from user");
    if (args::get(no_img_pairs_to_process) == "all")
    {
      noImg = -1;
    }
    else
    {
      noImg = std::stoi(args::get(no_img_pairs_to_process));
    }
  }
  else
  {
    console->info("Getting default no if img pairs to process");
    noImg = -1; // all option
  }
  (*COVO_SETTINGS)["no_img_pairs_to_process"] = noImg;

  std::string associationsFile;

  if (datasetRootFolder.back() != '/')
  {
    datasetRootFolder += '/';
  }

  associationsFile = datasetRootFolder + "associations.txt";

  std::string rgbImgFile;
  std::string rgbImgTimestamp;
  std::string depthImgFile;
  std::string depthImgTimestamp;

  std::ifstream ifAssociation(associationsFile, std::ios_base::in);
  console->info("Reading associations.txt file");
  if (ifAssociation.fail())
  {
    console->critical("Cannot read association.txt file");
    return 0;
  }
  while (ifAssociation >>
      rgbImgTimestamp >> rgbImgFile >>
      depthImgTimestamp >> depthImgFile)
  {
    std::array<std::string, 2> rgbdImgFile;
    rgbdImgFile[0] = datasetRootFolder + rgbImgFile;
    rgbdImgFile[1] = datasetRootFolder + depthImgFile;
    rgbdImgFiles->push_back(std::move(rgbdImgFile));

    std::array<double, 2> rgbdImgTimestamp;
    rgbdImgTimestamp[0] = std::stod(rgbImgTimestamp);
    rgbdImgTimestamp[1] = std::stod(rgbImgTimestamp);
    rgbdImgTimestamps->push_back(std::move(rgbdImgTimestamp));
   }
  ifAssociation.close();

  return 1;

}

