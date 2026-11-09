// Generates assets/FH4RPO.ini from the compiled-in defaults so the shipped
// configuration file can never drift from the code.
#include "fh4rpo/Config.hpp"
#include <cstdio>

int main(int argc, char** argv)
{
    const char* path = (argc > 1) ? argv[1] : "assets/FH4RPO.ini";
    fh4rpo::Config config;
    config.moduleName = "FH4RPO";
    config.Sanitize();
    if (!config.Save(path))
    {
        std::printf("failed to write %s\n", path);
        return 1;
    }
    std::printf("wrote %s\n", path);
    return 0;
}
