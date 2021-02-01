#ifndef __TEST_API_H__
#define __TEST_API_H__

#include <mutex>
#include <thread>
#include "ORBSLAM3_api.hpp"

class TestApi
{
public:
    TestApi(const std::string &yaml_config_fname);
    ~TestApi();
    void Run(const std::string &yaml_config_fname);
    void Stop();

private:
    void ProcessMonoImu(void *handle);

private:
    std::thread data_process_thread_;

    volatile bool stop_;
    std::mutex stop_mutex_;

    void *handle_;
};

#endif
