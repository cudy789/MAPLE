#pragma once

#include <string>
#include <cstdlib>
#include "Logger.h"

class Updater{
public:
    Updater() = delete;

    static bool Update(const std::string& bin_zip){
        AppLogger::Logger::Log("Updating MAPLE");

        // 1. locate .syrup update file. This is a .zip file of the build directory
        int return_code = system("mv ../bin ../bin-backup");

        std::string unzip_str = "unzip " + bin_zip + " -d ../bin";

        return_code = system(unzip_str.c_str());

        if (return_code){
            AppLogger::Logger::Log("Error updating MAPLE, reverting back to previous version.", AppLogger::SEVERITY::ERROR);
            return_code = system("rm -rf ../bin");
            return_code = system("mv ../bin-backup ../bin");
            if (return_code){
                AppLogger::Logger::Log("Error reverting MAPLE to previous version, aborting", AppLogger::SEVERITY::ERROR);
                exit(return_code);
            }
            return false;
        }

        return_code = system("rm -rf ../bin-backup");
        if (return_code){
            AppLogger::Logger::Log("Error removing backup, aborting", AppLogger::SEVERITY::ERROR);
            exit(return_code);
        }

        AppLogger::Logger::Log("Finished updating MAPLE. Restart to apply changes.");

        return true;
    }


};