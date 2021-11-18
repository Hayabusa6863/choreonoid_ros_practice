/**
 * @file    RttJointStatePublisher.cpp
 * @author  your name (you@domain.com)
 * @brief   ロボットの関節の状態をJointState型のROStopicとしてPublish
 * @version 0.1
 * @date    2021-11-18
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <cnoid/SimpleController>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>

class RttJointStatePublisher : public cnoid::SimpleController{
private:
    cnoid::BodyPtr ioBody;
    std::unique_ptr<ros::NodeHandle> node;
    ros::Publisher jointStatePublisher;
    sensor_msgs::JointState jointState;
    double time;
    double timeStep;
    double cycleTime;
    double timeCounter;

public:
    virtual bool configure(cnoid::SimpleControllerConfig* config) override {
        node.reset(new ros::NodeHandle(config->body()->name()));
        jointStatePublisher = node->advertise<sensor_msgs::JointState>("joint_state", 1);
        return true;
    }

    virtual bool initialize(cnoid::SimpleControllerIO* io) override {
        ioBody = io->body();

        int nj = ioBody->numJoints();
        jointState.name.resize(nj);
        jointState.position.resize(nj);
        jointState.velocity.resize(nj);
        jointState.effort.resize(nj);

        for(int i=0; i<nj; i++){
            auto joint = ioBody->joint(i);
            io->enableInput(joint, cnoid::Link::JointDisplacement | JointVelocity | JointEffort);
            jointState.name[i] = joint->name();
        }

        time = 0.0;
        timeStep = io->timeStep();
        double frequency = 50.0;
        cycleTime = 1.0 / frequency;
        timeCounter = 0.0;
        
        return true;
    }


    virtual bool control() override {
        time += timeStep;
        timeCounter += timeStep;

        /**
         * timeCounterが周期に対応するcycleTimeに達した場合のみ，状態の出力を行うようにしている．
         * 一般的にcontrol関数はロボットの制御周期で実行されるが，それは状態出力の周期としては
         * 短すぎる場合が多いので，別途このような周期を設けている．
         * 
         */
        if(timeCounter >= cycleTime){
            jointState.header.stamp.fromSec(time);

            for(int i=0; i<ioBody->numJoints(); ++i){
                auto joint = ioBody->joint(i);
                jointState.position[i] = joint->q();
                jointState.velocity[i] = joint->dq();
                jointState.effort[i]   = joint->u();
            }
            jointStatePublisher.publish(jointState);
            timeCounter -= cycleTime;
        }
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(RttJointStatePublisher);