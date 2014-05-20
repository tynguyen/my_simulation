//In this version, apply force to front two wheels also => 4 wheels driven.
/* Version 7.2
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef _GAZEBO_CART_DEMO_PLUGIN_HH_
#define _GAZEBO_CART_DEMO_PLUGIN_HH_

#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/transport/transport.hh"
//#include "gazebo/util/system.hh"

#define NUM_JOINTS 3

namespace gazebo
{
  /// \brief This plugin drives a four wheeled cart model forward and back
  /// by applying a small wheel torque.  Steering is controlled via
  /// a position pid.
  /// this is a test for general rolling contact stability.
  /// should refine the test to be more specific in the future.
  class CartDemoPlugin : public ModelPlugin
  {
    public: CartDemoPlugin();
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    public: virtual void Init();
		public: enum pidType_t {GASFLAT = 1, GASUP = 3, GASDOWN = 5, 
		BRAKEFLAT = 2, BRAKEUP = 4, BRAKEDOWN = 6, TRACK = 7}; //to choose PID type
    private: void OnUpdate();
		public: double pidUpdate(pidType_t index, double error, common::Time stepTime );
    private: transport::NodePtr node;

    private: event::ConnectionPtr updateConnection;

    private: physics::ModelPtr model;

    private: physics::JointPtr joints[NUM_JOINTS];
    private: common::PID jointPIDs[8]; //Add 4 more for different cases and TRACK pid (to plan using PID)
    private: double jointPositions[NUM_JOINTS];
    private: double jointVelocities[NUM_JOINTS];
    private: double jointMaxEfforts[NUM_JOINTS];
    
    private: physics::JointPtr gasJoint, brakeJoint;
    private: double aeroLoad;
    private: double frontPower;
    private: double tireAngleRange;
    private: double steeringRatio;
    private: double maxGas, maxBrake;
    private: double swayForce;
    private: double maxSpeed;
    private: double rearPower;
    private: double wheelRadius;
	  private: double gas_force, brake_force;
    private: common::Time prevUpdateTime;
    private: double y_prev; //Original y value
    private: double x_prev;
    
    //The following variables are used for naive PID setpoint controller
    private: double time_orig, time_end;
    private: double vel_end;
    private: double d, x_orig;
     
    //SubscriberPtr to 
  };
}
#endif
