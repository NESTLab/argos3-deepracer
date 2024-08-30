#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/configuration/command_line_arg_parser.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <iostream>
#include <argos3/plugins/robots/deepracer/real_robot/real_deepracer.h>
#include <rclcpp/rclcpp.hpp>

using namespace argos;

int main(int argc, char* argv[]) {
    /*
     * Parse the command line
     */

    std::string strARGoSFName;
    std::string strControllerId;
    CCommandLineArgParser cCLAP;
    cCLAP.AddArgument<std::string>(
            'c',
            "--config-file",
            "the experiment XML configuration file",
            strARGoSFName);
    cCLAP.AddArgument<std::string>(
            'i',
            "--controller-id",
            "the id of the controller to run",
            strControllerId);
    try {
        cCLAP.Parse(argc, argv);

        /*
         * Initialize ROS
        */
        rclcpp::init(0, nullptr);

        /*
         * Initialize the robot
         */
        std::shared_ptr<CRealDeepracer> pcRobot = std::make_shared<CRealDeepracer>();

        pcRobot->Init(strARGoSFName, strControllerId);

        rclcpp::spin_some(pcRobot->GetNodeHandlePtr());

        /*
         * Perform the main loop
         */
        pcRobot->Execute();

        /*
         * Clean up
        */
        rclcpp::shutdown();
    }
    catch(CARGoSException& ex) {
        /* A fatal error occurred: dispose of data, print error and exit */
        LOGERR << ex.what() << std::endl;
        LOGERR.Flush(); // temporary fix to ensure that exception messages get published
        return 1;
    }
    /* All done (should never get here) */
    return 0;
}