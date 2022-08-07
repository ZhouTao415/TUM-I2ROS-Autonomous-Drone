#ifndef SIMULATION_QUADROTOR_LCM_H
#define SIMULATION_QUADROTOR_LCM_H

#include <lcm/lcm-cpp.hpp>
#include <Eigen/Dense>
#include "lcmtypes/drake/lcmt_quadrotor_input_t.hpp"
#include "lcmtypes/drake/lcmt_quadrotor_output_t.hpp"

class QuadrotorLCMInterface {

public:
    QuadrotorLCMInterface(std::string const& input_channel, std::string const& output_channel)
        : input_channel(input_channel), output_channel(output_channel) {
      if (Good()) {
        std::cout << "Subcribing to: " << output_channel << std::endl;
        lcm.subscribe(output_channel, &QuadrotorLCMInterface::OnOutput, this);
      }
    }

    bool Good() const {
      return lcm.good();
    }

    void Spin() {
      lcm.handle();
    }

    void PublishControls(Eigen::Vector4d const& motors) {
      drake::lcmt_quadrotor_input_t input_msg;
      Eigen::Map<Eigen::Vector4d> motor_map(input_msg.motors);
      motor_map = motors;
      lcm.publish(input_channel, &input_msg);
    }

    void RegisterIMUCallback(std::function<void(Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f)> callback) {
      imu_callback = callback;
    }

    void RegisterStateCallback(std::function<void(Eigen::Vector3f, Eigen::Vector4f, Eigen::Vector3f, Eigen::Vector3f)> callback) {
      state_callback = callback;
    }

private:
    void OnOutput(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan,
                       const  drake::lcmt_quadrotor_output_t* msg) {
      if(imu_callback) {
        //converted to PX4 frame
        imu_callback(Eigen::Vector3f(msg->accelerometer[0], -msg->accelerometer[1], -msg->accelerometer[2]),
                     Eigen::Vector3f(msg->gyroscope[0], -msg->gyroscope[1], -msg->gyroscope[2]),
                     Eigen::Vector3f(msg->magnetometer[0], -msg->magnetometer[1], -msg->magnetometer[2])
        );
      }

      if(state_callback) {
        Eigen::Vector4f orientation = Eigen::Vector4f(msg->orientation[0], msg->orientation[1], msg->orientation[2], msg->orientation[3]);
        orientation.normalize();
        state_callback(
            Eigen::Vector3f(msg->position[0], msg->position[1], msg->position[2]),
            orientation,
            Eigen::Vector3f(msg->twist[0], msg->twist[1], msg->twist[2]),
            Eigen::Vector3f(msg->twist[3], msg->twist[4], msg->twist[5])
        );
      }
    }

    std::string input_channel, output_channel;
    std::function<void(Eigen::Vector3f, Eigen::Vector3f, Eigen::Vector3f)> imu_callback;
    std::function<void(Eigen::Vector3f, Eigen::Vector4f, Eigen::Vector3f, Eigen::Vector3f)> state_callback;
    lcm::LCM lcm;
};

#endif
