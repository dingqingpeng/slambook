#ifndef CONFIG_H
#define CONFIG_H

#include "myslam/common_include.h"

namespace myslam
{

class Config
{
private:
    Config() {}

public:
    ~Config();  // Close file in deconstructing function

    static void setParameterFile( const std::string& filename );

    template<typename T>
    static T get( const std::string& key )
    {
        return T( Config::config_->file_[key] );
    }

private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;
};

}

#endif // CONFIG_H