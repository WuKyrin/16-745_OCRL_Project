/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mobile_manipulator_ros/MobileManipulatorDummyVisualization.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_msgs/msg/MpcFlattenedController>

#include "rclcpp/rclcpp.hpp"

using namespace ocs2;
using namespace mobile_manipulator;


rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

void topic_callback(const ocs2_msgs::msg::MpcFlattenedController::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->init_observation.state.value);
}

int main(int argc, char** argv) {
    const std::string robotName = "mobile_manipulator";

    // Initialize ros node
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(
        robotName + "_mrt",
        rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true));
    // Get node parameters
    std::string taskFile = node->get_parameter("taskFile").as_string();
    std::string libFolder = node->get_parameter("libFolder").as_string();
    std::string urdfFile = node->get_parameter("urdfFile").as_string();
    std::cerr << "Loading task file: " << taskFile << std::endl;
    std::cerr << "Loading library folder: " << libFolder << std::endl;
    std::cerr << "Loading urdf file: " << urdfFile << std::endl;
    // Robot Interface
    //   mobile_manipulator::MobileManipulatorInterface interface(taskFile, libFolder,
    //                                                            urdfFile);

    // MRT
    // MRT_ROS_Interface mrt(robotName);
    // mrt.initRollout(&interface.getRollout());
    // mrt.launchNodes(node);

    // Visualization
    // auto dummyVisualization =
    //     std::make_shared<mobile_manipulator::MobileManipulatorDummyVisualization>(
    //         node, interface);

    // Dummy MRT
    // MRT_ROS_Dummy_Loop dummy(mrt, interface.mpcSettings().mrtDesiredFrequency_,
    //                         interface.mpcSettings().mpcDesiredFrequency_);
    // dummy.subscribeObservers({dummyVisualization});

    // initial state
    // SystemObservation initObservation;
    // initObservation.state = interface.getInitialState();
    // initObservation.input.setZero(interface.getManipulatorModelInfo().inputDim);
    // initObservation.time = 0.0;

    // initial command
    //   vector_t initTarget(7);
    //   initTarget.head(3) << 1, 0, 1;
    //   initTarget.tail(4) << Eigen::Quaternion<scalar_t>(1, 0, 0, 0).coeffs();
    //   const vector_t zeroInput =
    //       vector_t::Zero(interface.getManipulatorModelInfo().inputDim);
    //   const TargetTrajectories initTargetTrajectories({initObservation.time},
    //                                                   {initTarget}, {zeroInput});

    // Run dummy (loops while ros is ok)
    //   dummy.run(initObservation, initTargetTrajectories);

    subscription_ = node->create_subscription<ocs2_msgs::msg::MpcFlattenedController>(
    "mobile_manipulator_mpc_policy", 10, std::bind(topic_callback, node));

    rclcpp::spin(node);
    rclcpp::shutdown();
    // Successful exit
    return 0;
}