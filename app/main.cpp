// composition root: 3 arguments run the command line, anything else opens the GUI

#include "cli.h"

#include <pfd_wrapper.hpp>

#include <iostream>
#include <new>
#include <stdexcept>

int run_gui(int argc, char *argv[]); // lidar_odometry_gui.cpp

int main(int argc, char *argv[])
{
    try
    {
        if (argc == 4) //runnning from command line
            return run_cli(argv);
        return run_gui(argc, argv);
    }
    catch (const std::bad_alloc& e)
    {
        std::cerr << "System is out of memory : " << e.what() << std::endl;
        mandeye::fd::OutOfMemMessage();
    }
    catch (const std::exception& e)
    {
        std::cout << e.what();
    }
    catch (...)
    {
        std::cerr << "Unknown fatal error occurred." << std::endl;
    }

    return 0;
}