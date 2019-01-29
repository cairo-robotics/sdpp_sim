#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include "ros/ros.h"

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
			std::cout << bounding << std::endl;

		}

		// POinter to the model
		private: physics::ModelPtr model;

		// Pointer to the udpate event connection
		private: event::ConnectionPtr updateConnection;

		// desired goal pose
		public: math::Pose goal_pose;


	};
	GZ_REGISTER_MODEL_PLUGIN(ModelWalk)
}