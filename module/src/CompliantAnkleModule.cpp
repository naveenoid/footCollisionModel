#include "CompliantAnkleModule.h"

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Port.h>
#include <yarp/os/LogStream.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>

#define DEG_2_RAD (M_PI / 180.0)

CompliantAnkleModule::CompliantAnkleModule()
: m_rpcPort(0)
, m_robot(0)
, m_homePosition(0, 0.0) {}

CompliantAnkleModule::~CompliantAnkleModule() { cleanup(); }

double CompliantAnkleModule::getPeriod() { return 0.5; }

bool CompliantAnkleModule::updateModule()
{
    //nothing to do here
    return true;
}

bool CompliantAnkleModule::configure(yarp::os::ResourceFinder &rf)
{
    using namespace yarp::os;

    m_rpcPort = new Port();
    if (!m_rpcPort || !m_rpcPort->open(getName("/rpc"))
        || !attach(*m_rpcPort)) {
        yError("Could not open RPC port");
        cleanup();
        return false;
    }

    Property wbiProperties;
    if (!rf.check("wbi_config_file", "Checking wbi configuration file")) {
        yError() << "No WBI configuration file found.";
        return false;
    }

    if (!wbiProperties.fromConfigFile(rf.findFile("wbi_config_file"))) {
        yError() << "Not possible to load WBI properties from file.";
        return false;
    }
    wbiProperties.fromString(rf.toString(), false);

    //retrieve the joint list
    std::string wbiList = rf.check("wbi_list", Value("ROBOT_TORQUE_CONTROL_JOINTS_WITHOUT_PRONOSUP"), "Looking for wbi list").asString();

    wbi::IDList iCubMainJoints;
    if (!yarpWbi::loadIdListFromConfig(wbiList, wbiProperties, iCubMainJoints)) {
        yError() << "Cannot find joint list";
        return false;
    }

    std::string jointName = rf.check("imp_joint", Value("l_ankle_hip"), "Check impedance joint name").asString();

    if (!iCubMainJoints.idToIndex(jointName, m_impedanceJointIndex)) {
        yError() << "Could not find specified joint.";
        return false;
    }

    //create an instance of wbi
    m_robot = new yarpWbi::yarpWholeBodyInterface(getName("/wbi").c_str(), wbiProperties);
    if (!m_robot) {
        yError() << "Could not create wbi object.";
        return false;
    }

    m_robot->addJoints(iCubMainJoints);
    if (!m_robot->init()) {
        yError() << "Could not initialize wbi object.";
        return false;
    }

    std::string tempString;
    yarpWbi::yarpWholeBodyActuators *actuators = ((yarpWbi::yarpWholeBodyInterface*)m_robot)->wholeBodyActuator();
    actuators->getControlProperty(yarpWbi::YarpWholeBodyActuatorsPropertyImpedanceStiffnessKey, tempString);
    double stiffness = Value(tempString).asDouble();
    actuators->getControlProperty(yarpWbi::YarpWholeBodyActuatorsPropertyImpedanceDampingKey, tempString);
    double damping = Value(tempString).asDouble();

    m_impedanceStiffness = rf.check("stiff", "check stiffness")
    ? rf.find("stiff").asDouble() : stiffness;

    m_impedanceDamping = rf.check("damp", "check damping")
    ? rf.find("damp").asDouble() : damping;


    if (rf.check("home","Getting home positons"))
    {
        Bottle &grp = rf.findGroup("home");
        int size = grp.size() - 1;

        if (size != iCubMainJoints.size()) {
            yError() << "Home position size has wrong dimension";
            return false;
        }

        m_homePosition.resize(size, 0);
        for (int i = 0; i < size; ++i)
            m_homePosition[i] = DEG_2_RAD * grp.get(i + 1).asDouble();
    }

    return true;
}

bool CompliantAnkleModule::close()
{
    cleanup();
    return true;
}

void CompliantAnkleModule::cleanup()
{
    if (m_rpcPort) {
        m_rpcPort->close();
        delete m_rpcPort; m_rpcPort = 0;
    }
    if (m_robot) {
        m_robot->close();
        delete m_robot; m_robot = 0;
    }
}

bool CompliantAnkleModule::respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply)
{
    std::cout << command.size() << "\n" << command.toString();
    if (command.size() > 0) {
        //just get the first command
        yarp::os::ConstString cmd = command.get(0).asString();
        if (cmd == "reset") {
            if (reset()) {
                reply.addInt(0);
                return true;
            } else {
                reply.addInt(-1);
                return false;
            }
        } else if (cmd == "start") {
            if (start()) {
                reply.addInt(0);
                return true;
            } else {
                reply.addInt(-1);
                return false;
            }
        } else if (cmd == "stop_dump") {
            if (stopDumping()) {
                reply.addInt(0);
                return true;
            } else {
                reply.addInt(-1);
                return false;
            }
        }
    }
    return RFModule::respond(command, reply);
}

bool CompliantAnkleModule::reset()
{
    m_robot->setControlMode(wbi::CTRL_MODE_POS);
    m_robot->setControlReference(m_homePosition.data());
    return true;
}

bool CompliantAnkleModule::start()
{
    using namespace yarpWbi;
    yarpWholeBodyActuators *actuators = ((yarpWholeBodyInterface*)m_robot)->wholeBodyActuator();

    actuators->setControlProperty(YarpWholeBodyActuatorsPropertyInteractionModeKey, YarpWholeBodyActuatorsPropertyInteractionModeCompliant, m_impedanceJointIndex);
    actuators->setControlProperty(YarpWholeBodyActuatorsPropertyImpedanceStiffnessKey, yarp::os::Value(m_impedanceStiffness).asString());
    actuators->setControlProperty(YarpWholeBodyActuatorsPropertyImpedanceDampingKey, yarp::os::Value(m_impedanceDamping).asString());
    return true;
}

bool CompliantAnkleModule::stopDumping()
{
    return true;
}
