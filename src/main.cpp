// composition root: 3 arguments (or 3 + --from-snapshot <file>) run the command line, --replay <capture> <params> replays a
// window capture, anything else opens the GUI

#include "cli.h"

#include "file_dialogs.h"

#include <iostream>
#include <new>
#include <stdexcept>
#include <string>

int run_gui(int argc, char *argv[]); // lidar_odometry_gui.cpp

int main(int argc, char *argv[])
{
    try
    {
        if (argc == 4 && std::string(argv[1]) == "--replay")
            return run_replay(argv[2], argv[3]);
        if (argc == 4 || (argc == 6 && std::string(argv[4]) == "--from-snapshot")) // running from the command line
            return run_cli(argc, argv);
        return run_gui(argc, argv);
    }
    // a run that ended in an exception exits with 1, so that scripts see it failed
    catch (const std::bad_alloc& e)
    {
        std::cerr << "System is out of memory : " << e.what() << std::endl;
        mandeye::fd::OutOfMemMessage();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Fatal error: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "Unknown fatal error occurred." << std::endl;
    }
    return 1;
}