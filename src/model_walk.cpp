#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

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
			//appply a small linear velocity to the model.
			this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
		}

		// POinter to the model
		private: physics::ModelPtr model;

		// Pointer to the udpate event connection
		private: event::ConnectionPtr updateConnection;
	};
	GZ_REGISTER_MODEL_PLUGIN(ModelWalk)
}