/**
 * @file    RttImuStatePublisher.cpp
 * @author  your name (you@domain.com)
 * @brief   ロボットに搭載されたImuの情報をPublish
 * @version 0.1
 * @date    2021-11-19
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <cnoid/SimpleController>
#include <cnoid/AccelerationSensor>
#include <cnoid/RateGyroSensor>
#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>

class RttImuStatePublisher : public cnoid::SimpleController{
private:
    std::unique_ptr<ros::NodeHandle> node;
    ros::Publisher imuPublisher;
    sensor_msgs::Imu imu;
    cnoid::AccelerationSensorPtr accelSensor;
    cnoid::RateGyroSensorPtr gyro;
    cnoid::Vector3 dv_sum;
    cnoid::Vector3 w_sum;

    int sensingSteps;
    double time;
    double timeStep;
    double cycleTime;
    double timeCounter;

public:
    virtual bool configure(cnoid::SimpleControllerConfig* config) override {
        node.reset(new ros::NodeHandle(config->body()->name()));
        imuPublisher = node->advertise<sensor_msgs::Imu>("imu", 1);
        return true;
    }


    virtual bool initialize(cnoid::SimpleControllerIO* io){
        accelSensor = io->body()->findDevice<cnoid::AccelerationSensor>();
        gyro = io->body()->findDevice<cnoid::RateGyroSensor>();

        io->enableInput(accelSensor);
        io->enableInput(gyro);

        dv_sum.setZero();
        w_sum.setZero();
        sensingSteps = 0;

        for(int i=0; i<9; ++i){
            imu.orientation_covariance[i] = 0.0;
            imu.angular_velocity_covariance[i] = 0.0;
            imu.linear_acceleration_covariance[i] = 0.0;
        }
        imu.orientation_covariance[0] = -1.0;
        imu.orientation.x = 0.0;
        imu.orientation.y = 0.0;
        imu.orientation.z = 0.0;
        imu.orientation.w = 0.0;

        time = 0.0;
        timeStep = io->timeStep();
        const double frequency = 20.0;
        cycleTime = 1.0 / frequency;
        timeCounter = 0.0;
        return true;
    }

    virtual bool control() override{
        dv_sum += accelSensor->dv();
        w_sum += gyro->w();
        ++sensingSteps;

        time += timeStep;
        timeCounter += timeStep;

        if(timeCounter >= cycleTime){
            imu.header.stamp.fromSec(time);

            // 合計値の平均値を出力する
            auto dv = dv_sum / sensingSteps;
            imu.linear_acceleration.x = dv.x();
            imu.linear_acceleration.y = dv.y();
            imu.linear_acceleration.z = dv.z();
            dv_sum.setZero();

            // 合計値の平均値を出力する
            auto w = w_sum / sensingSteps;
            imu.angular_velocity.x = w.x();
            imu.angular_velocity.y = w.y();
            imu.angular_velocity.z = w.z();
            w_sum.setZero();

            sensingSteps = 0;

            imuPublisher.publish(imu);
            timeCounter -= cycleTime;
        }
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(RttImuStatePublisher)