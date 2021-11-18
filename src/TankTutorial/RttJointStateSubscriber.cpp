/**
 * @file RttJointStateSubscriber.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2021-11-18
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <cnoid/SimpleController>

#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>

/**
 * BodyItemはBodyプラグインで定義されているクラスで，ロボットのモデルに対応するBodyオブジェクトをChoreonoidのGUI上で操作できるようにするためのもの．
 * 通常コントローラは特定のGUIには依存しないように実装されるが，今回はGUI上のモデルを直接更新することが目的なので，BodyItemを使用するようにしている．
 * 
 */ 
#include <cnoid/BodyItem>

/**
 * ChoreonoidのBaseモジュールで定義されているもの．
 * 
 */
#include <cnoid/LazyCaller>

class RttJointStateSubscriber : public cnoid::SimpleController{
private:
    std::unique_ptr<ros::NodeHandle> node;
    ros::Subscriber subscriber;
    cnoid::BodyItemPtr bodyItem;    // BodyItemに対応するポインタ変数

public:
    virtual bool configure(cnoid::SimpleControllerConfig* config) override {
        /* 状態更新の対象となるBodyアイテムを取得する */
        bodyItem = static_cast<cnoid::BodyItem*>(config->bodyItem());
        node.reset(new ros::NodeHandle(bodyItem->name()));
        subscriber = node->subscribe(std::string("/") + bodyItem->name() + "/joint_state", 1, &RttJointStateSubscriber::jointStateCallback, this);
        return true;
    }

    /**
     * コールバックが呼ばれるスレッドは通常のスレッド（メインスレッド）とは異なるため，更新処理をこの関数内で直接行ってはいけない！
     * Subscribeは受信用ポートへの入力によってトリガーされる非同期処理であるため，そのための専用のスレッドで処理される．
     * 一方，可視化用モデルはGUIを稼働しているメインスレッド内で管理されている．
     * この場合，Subscribe用のスレッドからメインスレッドのオブジェクトに直接アクセスすることはできない．
     * その為，メインスレッド内でGUIを稼働するためのイベントループを使用し，イベントを投じることで
     * 別スレッドからメインスレッドへの処理の転送を実現できる．
     * これを行う関数がcallLater関数で，この関数はどのスレッドからも実行することができ，引数として与えた関数はイベントループを経由して
     * メインスレッド上で実行される．
     * 
     * なお，SubscribeされたJointState型のデータは，callLaterに与えたラムダ式でキャプチャしており，この際データがキャプチャ後の別変数にコピーされている．
     * このコピー処理により，JointStateデータに関しては排他制御を行う必要が無い． 
     */
    void jointStateCallback(const sensor_msgs::JointState& state){
        cnoid::callLater([this, state](){ updateJointState(state); });
    }


    /**
     * メインスレッドから実行されるモデル状態更新の処理を行う． 
     */
    void updateJointState(const sensor_msgs::JointState& state){
        auto body = bodyItem->body();   // 更新対象のBodyオブジェクトを取得
        auto& names = state.name;       
        auto& positions = state.position;
        int size = std::min(names.size(), positions.size());
        int nj = std::min(body->numJoints(), size);
        
        /* 各関節の関節角度の現在値をモデルにセット */
        for(int i=0; i<nj; ++i){
            auto joint = body->joint(i);
            if(joint->jointName() == names[i]){
                joint->q() = positions[i];
            }
        }

        /** 
         * モデルの状態が更新されたことをChoreonoidのGUIに通知する．
         * これを行うことで，シーン描画を含むChoreonoid上の各種GUIコンポーネントがモデルの更新を反映するようになる．
         * 引数のtrueにより，通知前に順運動学計算も適用される．
         * これは以下のコードと同意
         * body->calcForwardKinematics();
         * bodyItem->notifyKinematicStateChange();
         */
        bodyItem->notifyKinematicStateChange(true);
    }

    virtual void unconfigure() override {
        bodyItem.reset();
        node.reset();
        subscriber = ros::Subscriber(); // Subscriberをクリア
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(RttJointStateSubscriber)