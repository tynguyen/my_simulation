//Version 9.0 
/*
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


#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "/home/tynguyen/my_plugin/CartDemoPlugin.hh"
#include <fstream> //ofstream
#include <math.h> //round(), sqrt()
#include <stdlib.h> // abs function
using namespace std;


using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(CartDemoPlugin)

/////////////////////////////////////////////////
CartDemoPlugin::CartDemoPlugin()
{
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    this->jointPIDs[i] = common::PID(1, 0.1, 0.01, 1, -1);
    this->jointPositions[i] = 0;
    this->jointVelocities[i] = 0;
    this->jointMaxEfforts[i] = 100;
    this->aeroLoad = 0.1;
	  this->swayForce = 10;
     
 	  this->maxSpeed = 10;
  	this->frontPower = 50;
 		this->rearPower = 50;
    //Max wheel angle
    this->wheelRadius = 0.2;
  }
}

/////////////////////////////////////////////////
void CartDemoPlugin::Load(physics::ModelPtr _model,
                           sdf::ElementPtr _sdf)
{
  this->model = _model;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  if (!_sdf->HasElement("steer"))
    gzerr << "CartTest plugin missing <steer> element\n";

  // get all joints
  this->joints[0] = _model->GetJoint(
    _sdf->GetElement("steer")->Get<std::string>());
  this->jointPIDs[0] = common::PID(
    _sdf->GetElement("steer_pid")->Get<math::Vector3>().x,
    _sdf->GetElement("steer_pid")->Get<math::Vector3>().y,
    _sdf->GetElement("steer_pid")->Get<math::Vector3>().z,
    _sdf->GetElement("steer_ilim")->Get<math::Vector2d>().y,
    _sdf->GetElement("steer_ilim")->Get<math::Vector2d>().x);
  this->jointPositions[0] =
    _sdf->GetElement("steer_pos")->Get<double>();
  this->jointVelocities[0] =
    _sdf->GetElement("steer_vel")->Get<double>();
  this->jointMaxEfforts[0] =
    _sdf->GetElement("steer_eff")->Get<double>();

  this->joints[1] = _model->GetJoint(
    _sdf->GetElement("right")->Get<std::string>());
  this->jointPIDs[1] = common::PID(
    _sdf->GetElement("gas_pid")->Get<math::Vector3>().x,
    _sdf->GetElement("gas_pid")->Get<math::Vector3>().y,
    _sdf->GetElement("gas_pid")->Get<math::Vector3>().z,
    _sdf->GetElement("gas_ilim")->Get<math::Vector2d>().y,
    _sdf->GetElement("gas_ilim")->Get<math::Vector2d>().x);
  this->jointPositions[1] =
    _sdf->GetElement("right_pos")->Get<double>();
  this->jointVelocities[1] =
    _sdf->GetElement("right_vel")->Get<double>();
  this->jointMaxEfforts[1] =
    _sdf->GetElement("right_eff")->Get<double>();

  this->joints[2] = _model->GetJoint(
    _sdf->GetElement("left")->Get<std::string>());
  this->jointPIDs[2] = common::PID(
    _sdf->GetElement("brake_pid")->Get<math::Vector3>().x,
    _sdf->GetElement("brake_pid")->Get<math::Vector3>().y,
    _sdf->GetElement("brake_pid")->Get<math::Vector3>().z,
    _sdf->GetElement("brake_ilim")->Get<math::Vector2d>().y,
    _sdf->GetElement("brake_ilim")->Get<math::Vector2d>().x);
  this->jointPositions[2] =
    _sdf->GetElement("left_pos")->Get<double>();
  this->jointVelocities[2] =
    _sdf->GetElement("left_vel")->Get<double>();
  this->jointMaxEfforts[2] =
    _sdf->GetElement("left_eff")->Get<double>();
  this->gasJoint = this->model->GetJoint(_sdf->Get<std::string>("gas"));
  this->brakeJoint = this->model->GetJoint(_sdf->Get<std::string>("brake"));
  
  this->maxSpeed = _sdf->Get<double>("max_speed");
  this->aeroLoad = _sdf->Get<double>("aero_load");
  this->tireAngleRange = _sdf->Get<double>("tire_angle_range");
  this->frontPower = _sdf->Get<double>("front_power");
  this->rearPower = _sdf->Get<double>("rear_power");
  
  
  
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&CartDemoPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void CartDemoPlugin::Init()
{
  // physics::EntityPtr parent = boost::dynamic_pointer_cast<physics::Entity>(
  //   this->joints[0]->GetChild());
  // The total range the steering wheel can rotate
  double steeringRange = this->joints[0]->GetHighStop(0).Radian() -
                         this->joints[0]->GetLowStop(0).Radian();
  // Compute the angle ratio between the steering wheel and the tires
  this->steeringRatio = steeringRange / this->tireAngleRange;
  // Maximum gas is the upper limit of the gas joint
  this->maxGas = this->gasJoint->GetHighStop(0).Radian();

  // Maximum brake is the upper limit of the gas joint
  this->maxBrake = this->gasJoint->GetHighStop(0).Radian();
  //   A little force to push back on the pedals
  this->gasJoint->SetForce(0, -0.1);
  this->brakeJoint->SetForce(0, -0.1);

  printf("SteeringRation[%f] MaxGa[%f]\n", this->steeringRatio, this->maxGas);
  
  //Create a file to ouput parameters
  ofstream myfile;
  myfile.open ("pidOut.csv", ios::out| ios::trunc); 
}

/////////////////////////////////////////////////
void CartDemoPlugin::OnUpdate()
{
  common::Time currTime = this->model->GetWorld()->GetSimTime();
  common::Time stepTime = currTime - this->prevUpdateTime;
  this->prevUpdateTime = currTime;
  
  // Get the normalized gas and brake amount
  double gas = this->gasJoint->GetAngle(0).Radian() / this->maxGas;
  double brake = this->brakeJoint->GetAngle(0).Radian() / this->maxBrake;



  // Get the steering angle
  double steeringAngle = this->joints[0]->GetAngle(0).Radian();

  // Compute the angle of the front wheels.
  double wheelAngle = steeringAngle / this->steeringRatio;

  // double idleSpeed = 0.5;

  
  
  
  for (int i = 0; i < 1; i++)
  {
    // first joint, set position
    double pos_target = this->jointPositions[i];
    double pos_curr = this->joints[i]->GetAngle(0).Radian();
    double max_cmd = this->jointMaxEfforts[i];
    double steerVel = 0;
    double pos_err = pos_curr - pos_target;
    
		
		
		
    double effort_cmd = this->jointPIDs[i].Update(pos_err, stepTime);
    effort_cmd = effort_cmd > max_cmd ? max_cmd :
      (effort_cmd < -max_cmd ? -max_cmd : effort_cmd);
    this->joints[i]->SetForce(0, effort_cmd);
//    gzdbg << "steer [" << pos_curr << "] [" << pos_target << "]"<<"steer_effort:"<<effort_cmd;
  }
  
    
/*Forward => backward => stop*/
    double tmp_t = currTime.Double();
    double vel_target;
    
    // custom test
    
    
    
    
    /* Pid to velocity */
    if (tmp_t < 10)      
      vel_target = 0;
    else if (tmp_t<35)
    	vel_target = 8.0;
//    else if (tmp_t<40)
//    	vel_target = 2.0;
    else if (tmp_t<50)
    	vel_target = 8.0;
    else if (tmp_t<60)
    	vel_target = 8.0;
    else if (tmp_t<90)
    	vel_target = 8.0;
    else if (tmp_t<95)
    	vel_target = 8.0;
//    else if (tmp_t<100)
//    	vel_target = 4.0;
//    else if (tmp_t<105)
//    	vel_target = 2.0;
    else if (tmp_t<120)
    	vel_target = 8.0;
    	else if (tmp_t<200)
    	vel_target = 4.0;
    else vel_target = 0.0;
    
    double vel_curr = this->joints[1]->GetVelocity(0);
    double vel_err = vel_curr - vel_target;
    double max_cmd = 1300.0;
    double eff;
    
    
    
    //Why with vel_target=1, brake = gas = 0.3678? while with target 2, vehicle move with gas and brake = zero
   	//jointVel is always zeros, 
    if(vel_err <= 0.05)
    {
     this->brake_force = 0;	
     this->gas_force = this->jointPIDs[1].Update(vel_err, stepTime.Double());
      this->gas_force = this->gas_force > max_cmd ? max_cmd :
        (this->gas_force < -max_cmd ? -max_cmd : this->gas_force);
    }
    
    else if (vel_err > 0.05)
    {
     this->gas_force = 0;
     this->brake_force = this->jointPIDs[1].Update(vel_err, stepTime.Double());
     this->brake_force = this->brake_force > max_cmd ? max_cmd :
        (this->brake_force < -max_cmd ? -max_cmd : this->brake_force);
    }
    
    else 
    	{
    		this->brake_force = 0;
    		this->gas_force = 0;
    	}
    
    if(vel_target == 0)
    {
    	gas_force = 0;
    	brake_force = 0;
    }
     //Force applied to wheels = this->gas_force + this->brake_force + airResistantForce + FrictionForce 
     //Slope resistance force seems to be applied automatically by gazebo, so not need to include in this   
     //formula   
//     eff =  this->gas_force + this->brake_force*abs(vel_curr)/abs(vel_curr + 0.0001) - 
//	      vel_curr*0.002745 - vel_curr*0.01*9.8*25.5/abs(vel_curr + 0.00001);
		//Apply static friction force (max = 25.5*9.8 = 249.9 N)
		
		
    
    
    //Current coordinates of vehicle
    math::Pose orig_pose = this->model->GetWorldPose();
		//X,Z coordinate:
		double current_z = orig_pose.pos.z;
		double current_x = orig_pose.pos.x;
		//Calculate cAlpha:
    double cAlpha = 1.0;
    double sAlpha = 0.0;
    if(current_x >-30.0 && current_z <= 5.72 )
    {
    	cAlpha = 1.0;
    	sAlpha = 0;
    }
    	
    else if(current_x >-30.0 && 5.72<current_z<10.7)
    {
    	cAlpha = 16/sqrt(16*16 + 4.804*4.804);
    	sAlpha = sqrt(1-cAlpha*cAlpha);
    }
    else if(current_z >= 10.7)
    	{
    	cAlpha = 1.0;
    	sAlpha = 0;
    }
    else if(current_x < -30.0 && 5.72<current_z<10.7)
    {
    	cAlpha = 26/sqrt(26*26 + 4.804*4.804);
    	sAlpha = -sqrt(1-cAlpha*cAlpha);
    }
    
    //Set minimum vel_curr to apply static force
    if(abs(vel_curr) < 0.009) 
    	vel_curr = 0;
    //Define direction to which brake force resists movement.
		double brake_direct = -1; 
		if(vel_curr != 0 )
			brake_direct = -vel_curr/abs(vel_curr);
		else 
			brake_direct = -(gas_force - sAlpha*9.8*25.5)/abs(gas_force - sAlpha*9.8*25.5 + 0.001); 
		//when car stops, there are only gas force and slope resistance force
		double statFric = 0;
		if(vel_curr == 0)
			{
				statFric = -(this->gas_force - sAlpha*9.8*25.5);   
				statFric = statFric > 9.8*25.5 ? 9.8*25.5 :
        (eff < -9.8*25.5 ? -9.8*25.5 : statFric);
			}
		else
				statFric = 0.0;
    
    eff = this->gas_force + this->brake_force*brake_direct
    			- vel_curr*0.002745 - vel_curr*0.01*9.8*25.5*cAlpha/abs(vel_curr + 0.00001)  
    			- vel_curr*abs(vel_curr)*0.35 + statFric ;
 			
   	gzdbg <<"Current Position x, y , z: "<< orig_pose.pos.x<<"	"
   				<<orig_pose.pos.y<<"	"<<orig_pose.pos.z<<"	";
    	
//    }   
//      eff = eff > max_cmd ? max_cmd :
//        (eff < -max_cmd ? -max_cmd : eff);
  
//      double max_cmd = 100.0;  // this->jointMaxEfforts[i];

//      double vel_err = vel_curr - vel_target;

//      eff = this->jointPIDs[i].Update(vel_err, stepTime);
//      eff = eff > max_cmd ? max_cmd :
//        (eff < -max_cmd ? -max_cmd : eff);
//    }
//    else
//    {
//      // hold wheel positions
//      double pos_target = this->jointPositions[i];
//      double pos_curr = this->joints[i]->GetAngle(0).Radian();
//      double max_cmd = 100;  // this->jointMaxEfforts[i];

//      double pos_err = pos_curr - pos_target;

//      eff = this->jointPIDs[i].Update(pos_err, stepTime);
//      eff = eff > max_cmd ? max_cmd :
//        (eff < -max_cmd ? -max_cmd : eff);
//      // gzdbg << "wheel pos [" << pos_curr << "] tar [" << pos_target << "]\n";
//    }




///*Going around 0*/
//  for (int i = 1; i < NUM_JOINTS; i++)
//  {
//    double tmp_t = currTime.Double();
//    double eff;
//    // custom test
//    if (tmp_t < 10)      eff = 0;
//    else if (tmp_t < 20) eff = this->jointMaxEfforts[i];
//    else if (tmp_t < 30) eff = -this->jointMaxEfforts[i];
//    else if (tmp_t < 40) eff = -this->jointMaxEfforts[i];
//    else if (tmp_t < 50) eff = this->jointMaxEfforts[i];
//    else if (tmp_t < 60)
//    {
//      /* pid to velocity */
//      double vel_target = this->jointVelocities[i];
//      double vel_curr = this->joints[i]->GetVelocity(0);
//      double max_cmd = 100.0;  // this->jointMaxEfforts[i];

//      double vel_err = vel_curr - vel_target;

//      eff = this->jointPIDs[i].Update(vel_err, stepTime);
//      eff = eff > max_cmd ? max_cmd :
//        (eff < -max_cmd ? -max_cmd : eff);
//    }
//    else
//    {
//      // hold wheel positions
//      double pos_target = this->jointPositions[i];
//      double pos_curr = this->joints[i]->GetAngle(0).Radian();
//      double max_cmd = 100;  // this->jointMaxEfforts[i];

//      double pos_err = pos_curr - pos_target;

//      eff = this->jointPIDs[i].Update(pos_err, stepTime);
//      eff = eff > max_cmd ? max_cmd :
//        (eff < -max_cmd ? -max_cmd : eff);
//      // gzdbg << "wheel pos [" << pos_curr << "] tar [" << pos_target << "]\n";
//    }

//    for (int i = 1; i < NUM_JOINTS; i++)
//    {
//    gzdbg << " wheel pos ["
//          << this->joints[i]->GetAngle(0).Radian()
//          << "] vel ["
//          << this->joints[i]->GetVelocity(0)<<"] JointVel"<<jointVel<<"] maxSpeed ["<<maxSpeed<<"] wheelRadius["<<wheelRadius<<"] Gas, brake [{"<<gas<<"}{"<<brake<<"}]"<<"vel_target["<<vel_target;
//    
//    this->joints[i]->SetVelocity(1, jointVel);
//    this->joints[i]->SetMaxForce(1, MaxForce);
////this->joints[i]->SetMaxForce(1, 0.5); // with this value, good figure
//    }
//  gzdbg << "\n";
  
   		 
   	for (int i = 1; i < NUM_JOINTS; i++)
    {
//    gzdbg << " wheel pos ["
//          << this->joints[i]->GetAngle(0).Radian()
//          << "] vel ["
//          << this->joints[i]->GetVelocity(0)<<"] JointVel"<<jointVel<<"] maxSpeed ["<<maxSpeed<<"] wheelRadius["<<wheelRadius<<"] Gas, brake [{"<<gas<<"}{"<<brake<<"}]"<<"vel_target["<<vel_target;
    //Initialize concrete condition to avoid auto flow at the beggining
    
    this->joints[i]->SetForce(0, eff );
//this->joints[i]->SetMaxForce(1, 0.5); // with this value, good figure
    }
    gzdbg << "\n";
 
  ////////////////////////////////////////////////
	/* Out put parameters to file "pidOut.csv"*/
	if(tmp_t*10 == round(tmp_t*10) ) //sample each 0.1 second
  {
  ofstream myfile;
 	myfile.open ("pidOut.csv", ios::out| ios::app);  //Append to existing file
 	myfile << tmp_t<<"\tvel\t"<< this->joints[1]->GetVelocity(0)<<"\teff\t"<<eff
  		<<"\tGas_force\t"<<gas_force<<"\tbrake_force\t"<<brake_force<<"\tvel_target\t"
  		<<vel_target<<"\tx\t"<<current_x<<"\tz\t"<<current_z<<"\n";
  }
}

