/**
 * @file RttImuStateSubscriber.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-19
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <cnoid/SimpleController>
#include <cnoid/BodyItem>
#include <cnoid/AccelerationSensor>
#include <cnoid/RateGyroSensor>
#include <cnoid/LazyCaller>
#include <cnoid/Timer>
#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>
#include <random>

class RttImuStateSubscriber : public cnoid::SimpleController{
private:
    std::unique_ptr<ros::NodeHandle> node;
    ros::Subscriber subscriber;
    cnoid::BodyItemPtr bodyItem;
    cnoid::AccelerationSensorPtr accelSensor;
    cnoid::RateGyroSensorPtr gyro;
    cnoid::Timer bodyShakeTimer;    // QtのQTimerクラスでChoreonoidのシグナル型を使えるようにしたもの
    double bodyShakeDuration;
    cnoid::Isometry3 bodyPosition;
    std::mt19937 mt;
    std::uniform_real_distribution<> rand;

public:
    virtual bool configure(cnoid::SimpleControllerConfig* config) override {
        bodyItem = static_cast<cnoid::BodyItem*>(config->bodyItem());

        auto body = bodyItem->body();

        accelSensor = body->findDevice<cnoid::AccelerationSensor>();
        if(accelSensor){
            // タイマが指定した時間間隔に達した（タイムアウトした）時に送出されるシグナル
            auto sigTimeout = bodyShakeTimer.sigTimeout();
            if(!sigTimeout.hasConnections()){
                sigTimeout.connect([this](){ onBodyShakeTimerTimeout(); });
                bodyShakeTimer.setInterval(20);
            }
        }

        gyro = body->findDevice<cnoid::RateGyroSensor>();

        node.reset(new ros::NodeHandle(bodyItem->name()));
        subscriber = node->subscribe(std::string("/") + bodyItem->name() + "/imu", 1, &RttImuStateSubscriber::imuStateCallback, this);
        return true;
    }

    void imuStateCallback(const sensor_msgs::Imu& state){
        cnoid::callLater([this, state](){ updateImuState(state); });
    }


    void updateImuState(const sensor_msgs::Imu& state){
        if(accelSensor){
            auto& dv = state.linear_acceleration;
            accelSensor->dv() << dv.x, dv.y, dv.z;
            accelSensor->notifyStateChange();
            if(accelSensor->dv().head<2>().norm() > 20.0){
                startBodyShake();
            }
        }

        if(gyro){
            auto& w = state.angular_velocity;
            gyro->w() << w.x, w.y, w.z;
            gyro->notifyStateChange();
        }
    }

    void startBodyShake(){
        bodyShakeDuration = 0.5;
        if(!bodyShakeTimer.isActive()){
            bodyPosition = bodyItem->body()->rootLink()->position();
            rand.param(std::uniform_real_distribution<>::param_type(-0.02, 0.02));
            bodyShakeTimer.start(); // タイマの開始
        }
    }


    void onBodyShakeTimerTimeout(){
        if(bodyShakeDuration > 0.0){
            auto T = bodyPosition;
            T.translation() += cnoid::Vector3(rand(mt), rand(mt), rand(mt));
            bodyItem->body()->rootLink()->setPosition(T);
        }
        // bodyShakeDurationが0以下になるとアニメーションを終了
        else{
            bodyShakeTimer.stop();
            bodyItem->notifyKinematicStateChange();
            bodyShakeDuration -= 0.02;
        }
    }

    virtual void unconfigure() override {
        node.reset();
        subscriber = ros::Subscriber();
        bodyItem.reset();
        accelSensor.reset();
        gyro.reset();
        bodyShakeTimer.stop();
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(RttImuStateSubscriber)