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
#include "geometry_msgs/Pose.h"
#include "sdpp_gazebo/set_goal.h"


#include <iostream>

namespace gazebo
{
	class ModelWalk : public ModelPlugin
	{
		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
		{
			// store the pointer to the model
			this->model = _parent;

			// Listen to the update event. This event is broadcast every
			// simulation iteration/
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
					std::bind(&ModelWalk::OnUpdate, this));



			if (!ros::isInitialized())
			{
			  int argc = 0;
			  char **argv = NULL;
			  ros::init(argc, argv, "gazebo_walker_node",
			      ros::init_options::NoSigintHandler);
			}

			// Create our ROS node. This acts in a similar manner to
			// the Gazebo node
			this->rosNode.reset(new ros::NodeHandle("gazebo_walker_node"));


			this->rosServ = n.advertiseService("test", &ModelWalk::onService);
			/*
			// Create a named topic, and subscribe to it.
			ros::SubscribeOptions so =
				ros::SubscribeOptions::create<std_msgs::Float32>(
			    "/" + this->model->GetName() + "/walk_goal",
				1,
			    boost::bind(&ModelWalk::OnRosMsg, this, _1),
			    ros::VoidPtr(), &this->rosQueue);

			this->rosSub = this->rosNode->subscribe(so);

			// Spin up the queue helper thread.
			this->rosQueueThread =
			  std::thread(std::bind(&ModelWalk::QueueThread, this));*/
		}

		public: void OnUpdate()
		{	

			const math::Pose curr_pose = this->model->GetWorldPose();
			
			//std::cout << curr_pose.pos << std::endl;
			
			if(curr_pose.pos[0] <= 10 ){
				//appply a small linear velocity to the model.
				this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
			}
			else{
				this->model->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
			}

			math::Box bounding = this->model->GetBoundingBox();
			//std::cout << bounding << std::endl;
		}

		public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
		{
		  std::cout << _msg->data << std::endl;
		}

		public: void onService(sdpp_gazebo::set_goal::Request &req,
							   sdpp_gazebo::set_goal::Request &req){

			res.plan_success = "test";
		}

		private: void QueueThread()
		{
		  static const double timeout = 0.01;
		  while (this->rosNode->ok())
		  {
		    this->rosQueue.callAvailable(ros::WallDuration(timeout));
		  }
		}



		// POinter to the model
		private: physics::ModelPtr model;

		// Pointer to the udpate event connection
		private: event::ConnectionPtr updateConnection;

		// desired goal pose
		public: std_msgs::Float32 goal_pose;
		
		/// \brief A node use for ROS transport
		private: std::unique_ptr<ros::NodeHandle> rosNode;

		//a ros service brief
		private: ros::ServiceServer rosServ;

		/// \brief A ROS subscriber
		private: ros::Subscriber rosSub;

		/// \brief A ROS callbackqueue that helps process messages
		private: ros::CallbackQueue rosQueue;

		/// \brief A thread the keeps running the rosQueue
		private: std::thread rosQueueThread;

	};
	GZ_REGISTER_MODEL_PLUGIN(ModelWalk)
}