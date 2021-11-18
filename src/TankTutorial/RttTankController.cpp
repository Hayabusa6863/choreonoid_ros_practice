/**
 * @file    RttTankController.cpp
 * @author  your name (you@domain.com)
 * @brief   外部でPublishされているROSトピックをコントローラでSubscribeして受信し，モデルを動かす
 * @version 0.1
 * @date    2021-11-18
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <cnoid/SimpleController>
#include <cnoid/Joystick>
#include <ros/node_handle.h>
#include <sensor_msgs/Joy.h>
#include <mutex>

using namespace cnoid;

// Rtt = 'R'OS 'T'ank 'T'utorial
class RttTankController : public SimpleController
{
    std::unique_ptr<ros::NodeHandle> node;
    ros::Subscriber subscriber;
    sensor_msgs::Joy latestJoystickState;
    std::mutex joystickMutex;

    Link* trackL;
    Link* trackR;
    Link* turretJoint[2];
    double qref[2];
    double qprev[2];
    double dt;

public:
    /* コントローラがプロジェクトに導入された時点で呼ばれる */
    virtual bool configure(SimpleControllerConfig* config) override{
        node.reset(new ros::NodeHandle);
        return true;
    }

    /* シミュレーション開始の直前に呼ばれる */
    virtual bool initialize(SimpleControllerIO* io) override{
        std::ostream& os = io->os();
        Body* body = io->body();
        dt = io->timeStep();

        trackL = body->link("TRACK_L");
        trackR = body->link("TRACK_R");
        io->enableOutput(trackL, JointVelocity);
        io->enableOutput(trackR, JointVelocity);

        turretJoint[0] = body->link("TURRET_Y");
        turretJoint[1] = body->link("TURRET_P");
        for(int i=0; i < 2; ++i){
            Link* joint = turretJoint[i];
            qref[i] = qprev[i] = joint->q();
            joint->setActuationMode(JointTorque);
            io->enableIO(joint);
        }

        subscriber = node->subscribe(
            "joy", 1, &RttTankController::joystickCallback, this);

        return true;
    }

    void joystickCallback(const sensor_msgs::Joy& msg){
        std::lock_guard<std::mutex> lock(joystickMutex);
        latestJoystickState = msg;
    }

    virtual bool control() override{
        sensor_msgs::Joy joystick;

        // 排他処理
        {
            std::lock_guard<std::mutex> lock(joystickMutex);
            joystick = latestJoystickState;
        }
        joystick.axes.resize(Joystick::NUM_STD_AXES, 0.0f);
        joystick.buttons.resize(Joystick::NUM_STD_BUTTONS, 0);

        static const int trackAxisID[] =
            { Joystick::L_STICK_H_AXIS, Joystick::L_STICK_V_AXIS };
        static const int turretAxisID[] =
            { Joystick::R_STICK_H_AXIS, Joystick::R_STICK_V_AXIS };

        double pos[2];
        for(int i=0; i < 2; ++i){
            pos[i] = joystick.axes[trackAxisID[i]];
            if(fabs(pos[i]) < 0.2){
                pos[i] = 0.0;
            }
        }
        // set the velocity of each tracks
        trackL->dq_target() = -2.0 * pos[1] + pos[0];
        trackR->dq_target() = -2.0 * pos[1] - pos[0];

        static const double P = 200.0;
        static const double D = 50.0;

        for(int i=0; i < 2; ++i){
            Link* joint = turretJoint[i];
            double pos = joystick.axes[turretAxisID[i]];
            if(fabs(pos) < 0.15){
                pos = 0.0;
            }
            double q = joint->q();
            double dq = (q - qprev[i]) / dt;
            double dqref = 0.0;
            double deltaq = 0.002 * pos;
            qref[i] += deltaq;
            dqref = deltaq / dt;
            joint->u() = P * (qref[i] - q) + D * (dqref - dq);
            qprev[i] = q;
        }

        return true;
    }

    /* コントローラ停止時 */
    virtual void stop() override{
        subscriber.shutdown();
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(RttTankController)