#ifndef COMPLIANTANKLEMODULE_H
#define COMPLIANTANKLEMODULE_H

#include <yarp/os/RFModule.h>

namespace yarp {
    namespace os {
        class ResourceFinder;
        class Bottle;
        class Port;
    }
}

namespace wbi {
    class wholeBodyInterface;
}

class CompliantAnkleModule : public yarp::os::RFModule
{
    yarp::os::Port *m_rpcPort;
    wbi::wholeBodyInterface* m_robot;

    int m_impedanceJointIndex;
    double m_impedanceStiffness;
    double m_impedanceDamping;

    void cleanup();

    bool reset();
    bool start();
    bool stopDumping();

public:
    virtual double getPeriod();
    virtual bool updateModule();
    virtual bool configure(yarp::os::ResourceFinder&);
    virtual bool close();
    virtual bool respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply);
};

#endif /* end of include guard: COMPLIANTANKLEMODULE_H */
