#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo
{
  class ModelFriction : public WorldPlugin
  {
    // Pointer to the terrain
    private: physics::ModelPtr terrain;

    // Pointer to the vehicle model
    private: physics::ModelPtr vehicle;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    // A ROS subscriber
    private: ros::Subscriber rosSub;

    // A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    // A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
    

    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      // Store the pointer to terrain model
      this->terrain = _world->ModelByName("my_mesh");

      // Store the pointer to vehicle model
      this->vehicle = _world->ModelByName("pioneer3at");

      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "friction_sub",
            ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("friction_sub"));

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            "/gazebo/cmd_mu",
            1,
            boost::bind(&ModelFriction::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&ModelFriction::QueueThread, this));

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      // this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      //     std::bind(&ModelFriction::OnUpdate, this));
    }

    // Handle an incoming message from ROS
    // A float value that is used to set the velocity
    /// of the Velodyne.
    public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
    {
      // printf("connected to ros, cmd_mu = %f\n", _msg->data);

      // Pointers to terrain friction 
      physics::LinkPtr link = terrain->GetLink("body");
      physics::CollisionPtr col = link->GetCollision("collision");
      physics::SurfaceParamsPtr m_surface = col->GetSurface();
      physics::FrictionPyramidPtr friction_pyramid = m_surface->FrictionPyramid();

      // Change friction
      friction_pyramid->SetMuPrimary(_msg->data);
      friction_pyramid->SetMuSecondary(_msg->data);
      printf("Mu changed to %f\n", friction_pyramid->MuPrimary());
    }

    // ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    // Called by the world update event (only used for checking)
    // public: void OnUpdate()
    // {
    //   // Pointers to terrain friction 
    //   physics::LinkPtr link = terrain->GetLink("body");
    //   physics::CollisionPtr col = link->GetCollision("collision");
    //   physics::SurfaceParamsPtr m_surface = col->GetSurface();
    //   physics::FrictionPyramidPtr friction_pyramid = m_surface->FrictionPyramid();
    //   printf("MuPrimary = %f\n", friction_pyramid->MuPrimary());
    //   printf("MuSeconday = %f\n", friction_pyramid->MuSecondary());

    // }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(ModelFriction)
}