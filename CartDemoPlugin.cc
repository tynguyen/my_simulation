/*--Version 17.3 map road2-v6.4 normal vehicle-->
//This version replace the old car_model and change completely the way apply force to wheels. 
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


#include "gazebo/common/common.hh"    //for gazebo 3.0, need to add extra part of path
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/transport/transport.hh"
#include "/home/tynguyen/my_plugin/CartDemoPlugin.hh"
#include <fstream> //ofstream
#include <math.h> //round()
#include <cmath> //pow(), acos()
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
    this-> u_e = 0;
    this->a = 0;
    this->b = 0;
    this->b_g = 0;
    this->kp = 0.08;
    this->ki = 0.06;
    this->gas_force = 0;
    this->brake_force = 0;
//    this->brake_kp = 0.02939;
//    this->brake_ki = 0.00588;
    this->brake_kp = 0.05143;
    this->brake_ki = 0.018;
    this->ITerm = 0;
    this->theta_prev = 0;
    this->x_orig = 0;
    this->v_prev = 0;
    this->i_store = 0;
    this->brake_i_store = 0;
    this->target_prev = 0; //Vel-target-previous
    this->u_prev = 0;
    this->brake_u_prev = 0;
    this->vel_set = 0;
  }
}

/////////////////////////////////////////////////
void CartDemoPlugin::pidCal(double &theta_e, double &v_e, double &a, double &b, double &b_g, double &kp, double &ki)
{
	//Compute the trottle u_e required to keep velocity ve
	//at slope thetae, velocity ve, and gear

	double A0=Tm*an*(1-bbeta*pow(an*v_e/wm-1,2.0));
	double A1=m*g*Cr+rho*Cd*A*v_e*v_e/2+m*g*sin(theta_e);
	u_e=A1/A0;

	//Compute linearized model
	double w=an*v_e;
	double T=Tm*(1-bbeta*pow((w/wm-1),2.0));
	double pT=-2*bbeta*Tm*(w/wm-1)/wm;
	a=(pow(an,2)*u_e*pT)/m;
	b=A0/m;
	b_g=g*cos(theta_e);

//	cout<<"a: "<<a<<" b:"<<b<<" b_g:"<<b_g<<" u_e:"<<u_e<<endl;
	double w0=0.5,z=1;
//	kp=(2*z*w0+a)/b ;
//	ki=w0*w0/b;
	kp = 0.08;
	ki = 0.12;
	cout<<"u_e:"<<u_e<<" a:"<<a<<" b:"<<b<<" kp:"<<kp<<" ki:"<<ki<<endl;

}
/////////////////////////////////////////////////
void CartDemoPlugin::brakePidCal(double &theta_e, double &kp, double &ki)
{
	double brake_v_e = 2;
	double k = m*g*2/10/(brake_Tm*brake_an);
	double alpha = (m*(2.06-g*sin(theta_e))/(brake_Tm*brake_an) - k)/100;
	double a_e = theta_e <= 0? 1: (0.1+g*sin(theta_e));
	double u_e = (a_e - g*sin(theta_e))*m/(brake_Tm*brake_an*(brake_v_e*brake_v_e*alpha + k));
	double Te = u_e*brake_Tm*(brake_v_e*brake_v_e*alpha + k);
	double w=brake_an*brake_v_e;
	double a=2*u_e*brake_v_e*alpha*brake_Tm*brake_an/m;
	double c = Te*brake_an/m;
	double b_g=g*cos(theta_e);

//	cout<<"a: "<<a<<" b:"<<b<<" b_g:"<<b_g<<" u_e:"<<u_e<<endl;
	double w0=0.5,z=1;
	kp=(2*z*w0-a)/c/10 ;
	ki=w0*w0/c/10;
	cout<<"brake_kp:"<<kp<<" brake_ki:"<<ki<<endl;

}

/////////////////////////////////////////////////
void CartDemoPlugin::Load(physics::ModelPtr _model,
                           sdf::ElementPtr _sdf)
{
  this->model = _model;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());


  this->joints[1] = _model->GetJoint(
    _sdf->GetElement("right")->Get<std::string>());
  this->jointPIDs[1] = common::PID(
    _sdf->GetElement("gas_pid")->Get<math::Vector3>().x,
    _sdf->GetElement("gas_pid")->Get<math::Vector3>().y,
    _sdf->GetElement("gas_pid")->Get<math::Vector3>().z,
    _sdf->GetElement("gas_ilim")->Get<math::Vector2d>().y,
    _sdf->GetElement("gas_ilim")->Get<math::Vector2d>().x);
//	this->jointPIDs[1] = common::PID(
//			this->kp, this->ki, 0, -1, 1);
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
  
  /// Add two more drive wheels
   this->joints[4] = _model->GetJoint(
    _sdf->GetElement("front_right")->Get<std::string>());
   this->joints[3] = _model->GetJoint(
    _sdf->GetElement("front_left")->Get<std::string>());
  
  
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&CartDemoPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void CartDemoPlugin::Init()
{
  // physics::EntityPtr parent = boost::dynamic_pointer_cast<physics::Entity>(
  //   this->joints[0]->GetChild());
  this->gasJoint->SetForce(0, -0.1);
  this->brakeJoint->SetForce(0, -0.1);
  

  
  
  //Create a file to ouput parameters
  ofstream myfile;
  myfile.open ("pidOut.csv", ios::out| ios::trunc); 
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void CartDemoPlugin::OnUpdate()
{
  common::Time currTime = this->model->GetWorld()->GetSimTime();
  common::Time stepTime = currTime - this->prevUpdateTime;
  this->prevUpdateTime = currTime;
  
//////////////////////////////////////////////////
/*Running conditions for the car                */
//////////////////////////////////////////////////
    /* Pid to velocity */
    double tmp_t = currTime.Double();
    double vel_target;
    
    if (tmp_t < 10)  
    	vel_target = 0;

    else if (tmp_t<20)
    	vel_target = 6.0;
    else if (tmp_t<30)
    	vel_target = 2.0;
//    else if (tmp_t<50)
//    	vel_target = 4.0;
    else if (tmp_t<50)
    	vel_target = 3.0;
    else if (tmp_t<90)
    	vel_target = 7.0;
//    else if (tmp_t<95)
//    	vel_target = 7.0;
//    else if (tmp_t<100)
//    	vel_target = 4.0;
//    else if (tmp_t<105)
//    	vel_target = 2.0;
    else if (tmp_t<120)
    	vel_target = 5.0;
    else if (tmp_t<300)
    	vel_target = 7.0;
    
    else vel_target = 0.0;
    
    
    double theta_e = 0, v_e = 0;
    
    //Posistion
    math::Pose orig_pose = this->model->GetWorldPose();
    //X,Z coordinate:
		double current_z = orig_pose.pos.z;
		double current_x = orig_pose.pos.x;
		double current_y = orig_pose.pos.y;
		if(tmp_t < 10)
			x_orig = current_x;
    
    ///Array of terrain height 
    double NET[5] = {4.0, 3.0, 3.8 , 2.5, 3.0};
    
    ///Identify z with respect to x 
    int i_z = 0;
    if(-128 <= current_x && current_x <-64)
    	i_z = 0;
    else if(-64<= current_x && current_x < 0)
    	i_z = 1;
    else if(0<= current_x && current_x< 64)
    	i_z = 2;
    else 
    	i_z = 3;

		///Flat start
	  if(-128+ 64*i_z <= current_x && current_x < -112 +64*i_z)
	  {
	  	theta_e = 0;
	  	v_e = 16;
	  }

	  ///Up slope 
	  else if(-112 + 64*i_z <= current_x && current_x < -97 + 64*i_z)
	  {
	  	v_e = 16;
	  	theta_e = atan(NET[i_z]/10); //Up
	  }

	  ///Flat hight
	  else if(-97 + 64*i_z <= current_x && current_x < -91 + 64*i_z)
	  {
	  	v_e = 16;
	  	theta_e = 0;
	  }

	  ///Down slope
	  else if(-91 + 64*i_z <= current_x && current_x< -71 + 64*i_z)
	  {
	  	v_e = 16;
	  	theta_e = -atan(NET[i_z]/14); //Down
	  }

	  ///Flat end	  
	  else
	  {
	   	v_e = 16;
	  	theta_e = 0;
	  }
	  
//////////////////////////////////////////////////
/*Apply car model and calculate parameters      */
//////////////////////////////////////////////////
		//Nonlinear model
		double vel_curr = this->joints[1]->GetVelocity(0);
		double vel_curr_y = this->joints[1]->GetVelocity(2);
    double eff = 0;
		double vel_err = vel_curr - vel_target;
//		vel_err = vel_err > 10? 10:(vel_err< - 10? -10: vel_err);
    if((theta_e - this->theta_prev) != 0)
    {
    	CartDemoPlugin::pidCal(theta_e, v_e, this->a, this->b, this->b_g, this->kp, this->ki);
    	CartDemoPlugin::brakePidCal(theta_e, this->brake_kp, this->brake_ki);
    }
    gzdbg <<"Start with kp:"<<this->kp<<" ki:"<<this->ki<<endl;
    this->theta_prev = theta_e;
   
    
		this->i_store = this->i_store + stepTime.Double()*vel_err;
		///Set iterm max and min
//		this->i_store = this->i_store > 0.1? 0.1:
//					(this->i_store < -0.1? -0.1:this->i_store);
		
		double uu = -this->kp*vel_err - this->ki*this->i_store;		
		double u= uu > 1? 1: (uu < 0? 0: uu);
		double dI = -this->ki*vel_err + 0.5*(u-uu);
//    this->i_store = this->i_store - dI*stepTime.Double();
		double torque = uu * Tm ;
		this->gas_force = 2*torque;
		this->gas_force = this->gas_force < 0? 0:this->gas_force;
		
		
		/*Resistant forces*/
		double sgn;
		if(abs(vel_curr) <= 0.01)
			sgn = 0;
		else if (vel_curr > 0)
			sgn = 1;
		else
			sgn = -1;
		double Fd=m*g*Cr*sgn + rho*Cd*A*vel_curr*vel_curr/2;
		Fd = 0;



		 /*Brake force*/
    this->brake_i_store = this->brake_i_store + stepTime.Double()*vel_err;
    double brake_uu = this->brake_kp*vel_err + this->brake_ki*this->brake_i_store;		
		double brake_u = brake_uu > 1? 1: (brake_uu < 0? 0: brake_uu);
		double k = m*g*2/10/(0.5*brake_Tm*brake_an);
		double alpha = (m*(5-g*sin(theta_e))/(brake_Tm*brake_an) - k)/100;
		double brake_torque = brake_u *brake_Tm*(vel_curr*vel_curr*alpha + k);
		this->brake_force = brake_an*brake_torque;
    this->brake_force = this->brake_force > 0 ? this->brake_force: 0 ;
//////////////////////////////////////////////////
/*Control strategy for gas and brake            */
//////////////////////////////////////////////////
		/*Switch gas and brake control*/

		if(theta_e > 0)   //Up
			{ 

    				this->brake_force = 0;
    				this->brake_i_store = 0;
			}

		else if(theta_e < 0)  //Down
			{
				// Gas only used when zero brake but not reach target
				if(vel_target >= this->target_prev && vel_err < -0.05 && brake_u == 0 && theta_e > -0.03)
					{
    				this->brake_i_store = 0;
    				this->brake_force = 0;
    			}
    		else if(vel_target < this->target_prev && vel_err < -0.02 && brake_u == 0 && theta_e > -0.03)
					{
    				this->brake_i_store = 0;
    				this->brake_force = 0;
    			}
				else
					{
    				this->gas_force = 0;
    			}
			}
		else
		 	{
		 		    				this->brake_force = 0;
    				this->brake_i_store = 0;
    		
    				 	}
		if (vel_curr < 0.001)
			{
			this->brake_force = 0;
			this->brake_i_store = 0;
			}
		this->target_prev = vel_target;
		
//////////////////////////////////////////////////////////
/*Set calculated parameters to wheels & simulation      */
//////////////////////////////////////////////////////////
		/*Total torque applied to each wheel*/
		///Preset to stop car from running right after gazebo launching
		if (tmp_t < 10)  
    {
      this->gas_force = 0;
      this->brake_force = 0;
    }  
		eff = (this->gas_force  ) - this->brake_force; 

		double acc = (vel_curr - v_prev)/0.001;
		v_prev = vel_curr;
		gzdbg	<<"vel_err:"<<vel_err<<" vel_target:"<<vel_target<<" vel_curr:"<<vel_curr<<endl;
		gzdbg <<"gas_signal:uu:"<<uu<<":u:"<<u<<" gas_force: "<<this->gas_force<<" Fd:"<<Fd
		      <<"\tBrake_signal:uu:"<<brake_uu<<":u:"<<brake_u
    			<<"\tBrake_force\t"<<this->brake_force
    			<<" Effort:"<<eff<<endl;
    gzdbg <<"kp:"<<kp<<" ki:"<<this->ki<<"	dI"<<dI<<endl;
    gzdbg <<"brake_kp:"<<brake_kp<<"\tbrake_ki:\t"<<brake_ki<<endl;
    gzdbg <<"x:"<<current_x<<"\ty:\t"<<current_y<<"\tz:\t"<<current_z<<"\ti_z\t"<<i_z<<"\tTheta_e:\t"<<theta_e<<endl;
    gzdbg	<<"NOW:\t"<<tmp_t
  					<<"\tDistance_by_angle\t"<<this->joints[1]->GetAngle(0).Radian()*radius*5
  					<<"\tDelta x:\t"<<current_x - this->x_orig
  					<<"\tvel_current\t"<<vel_curr<<"\tvel_curr_y:\t"<<vel_curr_y<<"\n";
    
    
    if(abs(vel_err) > 0.5)
    	vel_set = vel_set - vel_err;
    ///Set torque to each wheel
   	for (int i = 4; i >0; i--)
    {
//    this->joints[i]->SetForce(0, eff );
	    this->joints[i]->SetMaxForce(0,360);
			this->joints[i]->SetVelocity(0,vel_set);
		
    }
     
    gzdbg << "\n";
    
    
 
//////////////////////////////////////////////////
/* Out put parameters to file "pidOut.csv"      */
//////////////////////////////////////////////////
//  if(tmp_t*10 == round(tmp_t*10) ) //sample each 0.1 second
//  {
  	ofstream myfile;
  	myfile.open ("pidOut.csv", ios::out| ios::app);  //Append to existing file
  	myfile << tmp_t
  				<<"\tvel\t"<< this->joints[1]->GetVelocity(0)
  				<<"\tGas_force\t"<<this->gas_force
  				<<"\tEffort\t"<<eff
  				<<"\tv_e\t"<<v_e
					<<"\tvel_target\t"<<vel_target
					<<"\tx\t"<<current_x
					<<"\tz\t"<<current_z
					<<"\tCurrent_y\t"<<current_y
					<<"\tTheta_e\t"<<theta_e*180/3.14
					<<"\tAcc:\t"<<acc
					<<"\tkp:\t"<<kp
					<<"\tki:\t"<<ki
					<<"\tGas_actual_sig\t"<<uu
					<<"\tGas_limited_sig\t"<<u
					<<"\tBrake_force\t"<<this->brake_force
					<<"\tThis->i_store\t"<<this->i_store
					<<"\tvel_errorr*stepTime\t"<<vel_err*stepTime.Double()
					<<"\n";
// 	}
}
