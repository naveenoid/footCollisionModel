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

    std::cout << iCubMainJoints.toString() << "\n";

    std::string jointName = rf.check("imp_joint", Value("r_ankle_pitch"), "Check impedance joint name").asString();

    if (!iCubMainJoints.idToIndex(jointName, m_impedanceJointIndex)) {
        yError() << "Could not find " << jointName << " joint.";
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
    actuators->getControlProperty(yarpWbi::YarpWholeBodyActuatorsPropertyImpedanceStiffnessKey, tempString, m_impedanceJointIndex);
    double stiffness = Value(tempString).asDouble();
    actuators->getControlProperty(yarpWbi::YarpWholeBodyActuatorsPropertyImpedanceDampingKey, tempString, m_impedanceJointIndex);
    double damping = Value(tempString).asDouble();

    yInfo() << "Initial stiffness and damping = " << stiffness << " " << damping;

    m_impedanceStiffness = rf.check("stiff", "check stiffness")
    ? rf.find("stiff").asDouble() : stiffness;

    m_impedanceDamping = rf.check("damp", "check damping")
    ? rf.find("damp").asDouble() : damping;


    m_homePosition.resize(iCubMainJoints.size(), 0);
    //By default read initial position
    m_robot->getEstimates(wbi::ESTIMATE_JOINT_POS, m_homePosition.data());
    if (rf.check("home","Getting home positons"))
    {
        Bottle &home = rf.findGroup("home");
        Bottle *grp = home.get(1).asList();

        int size = grp->size();

        if (size != iCubMainJoints.size()) {
            yError() << "Home position size has wrong dimension";
            return false;
        }

        for (int i = 0; i < size; ++i)
            m_homePosition[i] = DEG_2_RAD * grp->get(i + 1).asDouble();
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
        } else if (cmd == "set") {
            if (command.size() > 2) {
                yarp::os::ConstString type = command.get(1).asString();
                if (command.get(2).isDouble()) {
                    double value = command.get(2).asDouble();
                    if (type == "stiff") {
                        m_impedanceStiffness = value;
                        yInfo() << "New stiffness set to " << m_impedanceStiffness;
                        reply.addInt(0);
                        return true;
                    } else if (type == "damp") {
                        m_impedanceDamping = value;
                        yInfo() << "New damping set to " << m_impedanceDamping;
                        reply.addInt(0);
                        return true;
                    }
                }
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
    yarpWbi::yarpWholeBodyActuators *actuators = ((yarpWbi::yarpWholeBodyInterface*)m_robot)->wholeBodyActuator();

    actuators->setControlProperty(yarpWbi::YarpWholeBodyActuatorsPropertyInteractionModeKey, yarpWbi::YarpWholeBodyActuatorsPropertyInteractionModeStiff, m_impedanceJointIndex);

    m_robot->setControlReference(m_homePosition.data());
    return true;
}

bool CompliantAnkleModule::start()
{
    using namespace yarpWbi;
    yarpWholeBodyActuators *actuators = ((yarpWholeBodyInterface*)m_robot)->wholeBodyActuator();

    yInfo() << "Setting impedance to " << m_impedanceStiffness << " " << m_impedanceDamping;
    bool result;
    result = actuators->setControlProperty(YarpWholeBodyActuatorsPropertyInteractionModeKey, YarpWholeBodyActuatorsPropertyInteractionModeCompliant, m_impedanceJointIndex);
    result = result && actuators->setControlProperty(YarpWholeBodyActuatorsPropertyImpedanceStiffnessKey, yarp::os::Value(m_impedanceStiffness).toString(), m_impedanceJointIndex);
    result = result && actuators->setControlProperty(YarpWholeBodyActuatorsPropertyImpedanceDampingKey, yarp::os::Value(m_impedanceDamping).toString(), m_impedanceJointIndex);


    return result;
}

bool CompliantAnkleModule::stopDumping()
{
    return true;
}
