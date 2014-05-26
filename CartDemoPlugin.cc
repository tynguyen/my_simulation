//Version 10.0
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


#include "gazebo/physics/physics.hh"
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
    this->kp = 0.5;
    this->ki = 0.01;
    this->ITerm = 0;
    this->theta_prev = 0;
  }
}

/////////////////////////////////////////////////
void CartDemoPlugin::pidCal(double &theta_e, double &v_e, double &u_e, double &a, double &b, double &b_g, double &kp, double &ki)
{
	double gears[5] = {40, 25, 16, 12, 10};	// gear ratios
	double m=1000;                         // mass of car kg
	double Tm = 190;                       // engine torque constant, Nm
	double wm = 420;                       // peak torque rate, rad/sec
	double bbeta = 0.4;                    // torque coefficient
	double Cr = 0.01;                      // coefficient of rolling friction
	double rho = 1.3;                      // density of air, kg/m^3
	double Cd = 0.32;                      // drag coefficient
	double A = 2.4;                        // car area, m^2
	double g = 9.8;                        // gravitational constant
	//Get parameters for car
	double vref=20;         //reference value for velocity m/s
	int gear=4;             //gear

	//Compute the trottle u_e required to keep velocity ve
	//at slope thetae, velocity ve, and gear

	double an=gears[gear];
	double A0=Tm*an*(1-bbeta*pow(an*v_e/wm-1,2.0));
	double A1=m*g*Cr+rho*Cd*A*v_e*v_e/2+m*g*sin(theta_e);
	u_e=A1/A0;

	//Compute linearized model
	double w=an*v_e;
	double T=Tm*(1-bbeta*pow((w/wm-1),2.0));
	double pT=-2*bbeta*Tm*(w/wm-1)/wm;
	a=(pow(an,2)*u_e*pT-rho*Cd*A*v_e)/m;
	b=A0/m;
	b_g=g*cos(theta_e);

//	cout<<"a: "<<a<<" b:"<<b<<" b_g:"<<b_g<<" u_e:"<<u_e<<endl;
	double w0=0.4,z=1;
	kp=(2*z*w0+a)/b ;
	ki=w0*w0/b;
	cout<<"kp:"<<kp<<" ki:"<<ki<<endl;

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
  }
  
    /* Pid to velocity */
    double tmp_t = currTime.Double();
    double vel_target;
    
    if (tmp_t < 10)      
      vel_target = 0;
    else if (tmp_t<35)
    	vel_target = 8.0;
//    else if (tmp_t<40)
//    	vel_target = 2.0;
//    else if (tmp_t<50)
//    	vel_target = 4.0;
    else if (tmp_t<50)
    	vel_target = 5.0;
    else if (tmp_t<90)
    	vel_target = 5.0;
//    else if (tmp_t<95)
//    	vel_target = 7.0;
//    else if (tmp_t<100)
//    	vel_target = 4.0;
//    else if (tmp_t<105)
//    	vel_target = 2.0;
    else if (tmp_t<120)
    	vel_target = 7.0;
    else vel_target = 0.0;
    
    
    double theta_e = 0, v_e = 0;
    
    //Posistion
    math::Pose orig_pose = this->model->GetWorldPose();
    //X,Z coordinate:
		double current_z = orig_pose.pos.z;
		double current_x = orig_pose.pos.x;
		double current_y = orig_pose.pos.y;
	
    if(current_x >-30.0 && current_z <= 5.72 )
    {
    	theta_e = 0;
    	v_e = 25;
    }
    	
    else if(current_x >-30.0 && 5.72<current_z<10.7)
    {
    	v_e = 30;
    	theta_e = acos(16/sqrt(16*16 + 4.804*4.804));//Down
    }
    else if(current_z >= 10.7)
    	{
    	v_e = 25; 
    	theta_e = 0;
    }
    else if(current_x < -30.0 && 5.72<current_z<10.7)
    {
    	v_e = 20;
    	theta_e = acos(26/sqrt(26*26 + 4.804*4.804)); //Up
    }
    
    else
     {
     	v_e = 25;
    	theta_e = 0;
    }
    
//    double vel_target_error = vel_target - v_e;
    //Update PID: 
    //a: -0.0215773 b:1.77549
//    //Calculate gas control signal (value = 0 to 1)
//    this->gas_force = this->jointPIDs[1].Update(vel_err, stepTime);
//    double ul = this->gas_force > 1 ? 1 :
//        (this->gas_force < 0 ? 0 : this->gas_force);
//    cout<<"gas_force: "<<this->gas_force<<endl;
//    double target_force = this->jointPIDs[1].Update(vel_target_error, stepTime);
//    double ul_target = target_force > 1? 1: (target_force < 0? 0: target_force);
//    //Calculate force applied to wheels except slope force (gazebo applies by default)
//    eff = a*(vel_err - vel_target_error) + b*(ul - ul_target) + b_g*theta_e;
//    cout<<"Theta_e: "<<theta_e<<" a: "<<a<<" b:"<<b<<" b_g:"<<b_g<<" u_e:"<<u_e<<endl;
//    gzdbg	<<"v_e: "<<v_e<<" vel_target: "<<vel_target<<" vel_curr: "<<vel_curr<<" gas_signal:"
//    			<<ul<<" Effort: "<<eff<<endl;
//    double dI = 0.09*(vel_target_error) + 2*(ul - this->gas_force);
//    cout<<"dI: "<<dI<<endl;
//    ki = ki + dI;
//    this->jointPIDs[1].SetIGain(ki);

		//Nonlinear model
		double vel_curr = this->joints[1]->GetVelocity(0);
    double eff = 0;
		double vel_err = vel_curr - vel_target;
		double m=1000;                         // mass of car kg
		double Tm = 190;                       // engine torque constant, Nm
		double wm = 420;                       // peak torque rate, rad/sec
		double bbeta = 0.4;                    // torque coefficient
		double Cr = 0.01;                      // coefficient of rolling friction
		double rho = 1.3;                      // density of air, kg/m^3
		double Cd = 0.32;                      // drag coefficient
		double A = 2.4;                        // car area, m^2
		double g = 9.8;                        // gravitational constant
    double an = 12;
    double u_e = 0;
    if((theta_e - this->theta_prev) != 0)
    {
    	CartDemoPlugin::pidCal(theta_e, v_e, u_e, this->a, this->b, this->b_g, this->kp, this->ki);
    }
    gzdbg <<"Start with kp:"<<kp<<" ki:"<<this->ki<<endl;
    this->theta_prev = theta_e;
    this->jointPIDs[1].SetIGain(this->ki);
    this->jointPIDs[1].SetPGain(this->kp);
//    this->a = this->a + stepTime.Double()*vel_err;
    double uu = this->jointPIDs[1].Update(vel_err, stepTime);
//		double uu = -this->kp*vel_err - this->ki*this->a; 
		
		double u= uu > 1? 1: (uu < 0? 0: uu);
		double dI = -this->ki*vel_err + 0.5*(u-uu);
//		this->ki = this->ki + dI*stepTime.Double();
		double omega = an*vel_curr;
		double torque = u * Tm * ( 1 - bbeta * pow((omega/wm - 1),2) );
		double F = an * torque;
		double sgn;
		if(abs(vel_curr) <= 0.02)
			sgn = 0;
		else if (vel_curr > 0)
			sgn = 1;
		else
			sgn = -1;
		double Fd=m*g*Cr*sgn + sgn*rho*Cd*A*vel_curr*vel_curr/2;
//		eff = (F - Fd)/m*5;
		
//		this->jointPIDs[1].SetIGain(this->ki);
		gzdbg	<<"vel_err:"<<vel_err<<" vel_target:"<<vel_target<<" vel_curr:"<<vel_curr<<" gas_signal:"
    			<<uu<<" Effort:"<<eff<<" F: "<<F<<" Fd:"<<Fd<<endl;
    gzdbg <<"kp:"<<kp<<" ki:"<<this->ki<<"	dI"<<dI<<endl;
    gzdbg <<"z: "<<current_z<<endl;
    
//    if(vel_curr < 0.02) 
//    	vel_curr = 0;
    
//    //Why with vel_target=1, brake = gas = 0.3678? while with target 2, vehicle move with gas and brake = zero
//   	//jointVel is always zeros, 
//    if(vel_err <= 0.05)
//    {
//     this->brake_force = 0;	
//     this->gas_force = this->jointPIDs[1].Update(vel_err, stepTime.Double());
//      this->gas_force = this->gas_force > max_cmd ? max_cmd :
//        (this->gas_force < -max_cmd ? -max_cmd : this->gas_force);
//    }
//    
//    else if (vel_err > 0.05)
//    {
//     this->gas_force = 0;
//     this->brake_force = this->jointPIDs[1].Update(vel_err, stepTime.Double());
//     this->brake_force = this->brake_force > max_cmd ? max_cmd :
//        (this->brake_force < -max_cmd ? -max_cmd : this->brake_force);
//    }
//    
//    else 
//    	{
//    		this->brake_force = 0;
//    		this->gas_force = 0;
//    	}
//    
//    if(vel_target == 0)
//    {
//    	gas_force = 0;
//    	brake_force = 0;
//    }
//     //Force applied to wheels = this->gas_force - this->brake_force - airResistantForce - FrictionForce    
//     eff =  this->gas_force + this->brake_force*abs(vel_curr)/abs(vel_curr + 0.0001) - 
//	      vel_curr*0.002745 - vel_curr*0.01*9.8*25.5/abs(vel_curr + 0.00001);
        
 
  
   	eff = 6.0;
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
  	myfile << tmp_t<<"\tvel\t"<< this->joints[1]->GetVelocity(0)<<"\tGas_contr_sig\t"
  				<<this->gas_force<<"\tEffort\t"<<eff
  				<<"\tv_e\t"<<v_e
					<<"\tvel_target\t"<<vel_target<<"\tx\t"<<current_x<<"\tz\t"<<current_z
					<<"\tCurrent_y\t"<<current_y
					<<"\tTheta_e\t"<<theta_e
					<<"\n";
 	}
}

