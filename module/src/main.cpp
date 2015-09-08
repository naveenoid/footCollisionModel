#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/LogStream.h>

#include "CompliantAnkleModule.h"

int main(int argc, char **argv)
{
    yarp::os::Network network;
    if (!yarp::os::Network::checkNetwork()) {
        yError("Could not find yarp network");
        return -1;
    }
    
    yarp::os::ResourceFinder finder;

    finder.configure(argc, argv);
    
    CompliantAnkleModule module;
    module.setName("/ankleExp");
    
    return module.runModule(finder);
}
