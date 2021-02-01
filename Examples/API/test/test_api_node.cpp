
#include "test_api.h"

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        fprintf(stderr, "Usage: %s yaml_config_fname\n", argv[0]);
        return -1;
    }

    std::string yaml_config_fname = argv[1];
    printf("yaml_config_fname: %s\n", yaml_config_fname.c_str());
    TestApi(yaml_config_fname.c_str());
    return 0;
}