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
#include <args.hxx>
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

  

  args::ArgumentParser parser("This reads <covo_settings.json> files,");
  args::HelpFlag help(parser, "help", "Display help menu", {'h', "help"});
  args::ValueFlag<std::string> settings_file(parser,
      "/path/to/SETTINGS.json",
      "JSON settings file to read. "
      "\nDefault SETTINGS.json located in params_settings folder",
      {'s', "settings_file"});
  args::ValueFlag<std::string> dataset_root_dir(parser,
      "/path/to/dataset_dir",
      "Raw dataset root directory."
      "\nDefault dataset located in docs folder",
      {'d', "dataset_root_dir"});
  args::ValueFlag<std::string> log_level(parser,
      "trace, debug, info, warn, error, critical",
      "Log level settings. \nDefault is at info level",
      {'l', "log_level"});
  args::ValueFlag<int> wait_key_delay(parser, "delay in ms",
      "Delay btw image readings. "
      "\nDefault is 0 (it will wait indefinitely for key press",
      {'t', "time_delay"});
  args::ValueFlag<std::string> project_root_folder(parser, 
      "/path/to/project/root/folder/", 
      "The root of folder COVO project", 
      {'p', "project_path"});

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

  if (project_root_folder)
  {
    projectRootFolder = args::get(project_root_folder);
    if (projectRootFolder.back() != '/')
      projectRootFolder += '/';
    settingsFilePath = projectRootFolder + "param_settings/SETTINGS.json";
    datasetRootFolder = projectRootFolder + "docs/sample_dataset/";
    console->info("Getting project root folder path from user");
  }
  else
  {
    std::string binPath = getCurrentWorkingDir();
    std::cout << binPath << std::endl;
    std::size_t found = binPath.find("covo_vim");
    if (found != std::string::npos)
    {
      console->info("Your project root folder found automatically: " + binPath.substr(0, found+9));
    }
    else
    {
      console->error("Could not find your project root folder. " 
          "Please, provide as a argument");
    }

    projectRootFolder = binPath.substr(0, found+9);
    settingsFilePath = projectRootFolder + "param_settings/SETTINGS.json";
    datasetRootFolder = projectRootFolder + "docs/sample_dataset/";
  }

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
    console->trace(COVO_SETTINGS->dump(2));
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
    console->trace(COVO_SETTINGS->dump(2));
  }

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

  if (dataset_root_dir)
  {
    console->info("Getting dataset root directory from user");
    datasetRootFolder = args::get(dataset_root_dir);
  }
  else
  {
    console->info("Getting default dataset root directory");
  }

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

  return 1;

}

