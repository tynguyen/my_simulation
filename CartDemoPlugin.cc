//Version 7.3
#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "/home/tynguyen/my_plugin/CartDemoPlugin.hh"
#include <fstream> //ofstream
#include <math.h> //round()
#include <stdlib.h> // abs function
using namespace std;


using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(CartDemoPlugin)

/////////////////////////////////////////////////
CartDemoPlugin::CartDemoPlugin()
{
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    
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
    this->vel_end = 6.0;
    this->time_end = 30; //70s
    this->time_orig = 10;
    this->d = 120.0; 
    this->x_orig = 0 ;
    this->d_sum = 0;
  }
  for (int i = 0; i < 6; i++)
  {
  this->jointPIDs[i] = common::PID(1, 0.1, 0.01, 1, -1);
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
    
  //Add two more PID for going up and down
  this->jointPIDs[3] = common::PID(40,1, 0); //GASUp
  this->jointPIDs[5] = common::PID(5,0.3,0);	//GASDown
  this->jointPIDs[4] = common::PID(5,0.3, 0);	//BRAKEUP
  this->jointPIDs[6] = common::PID(20,0.6, 0);	//BRAKEDOWN
  this->jointPIDs[7] = common::PID(1.8,0, 0.1);	//TRACK
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
double CartDemoPlugin::pidUpdate(enum pidType_t index, double error, common::Time stepTime)
{
	return this->jointPIDs[index].Update(error, stepTime);
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
	enum pidType_t gasType, brakeType;


  // Get the steering angle
  double steeringAngle = this->joints[0]->GetAngle(0).Radian();

  // Compute the angle of the front wheels.
  double wheelAngle = steeringAngle / this->steeringRatio;

  
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
  
    
    //Define pidType to calculate gas and brake forces
    math::Pose orig_pose = this->model->GetWorldPose();
    //X,Z coordinate:
		double current_z = orig_pose.pos.z;
		double current_x = orig_pose.pos.x;
		double cAlpha = 1.0;
    double sAlpha = 0.0;
		double vel_curr = this->joints[1]->GetVelocity(0);
    double tmp_t = currTime.Double();
    double vel_target;  //vel_target = this->vel_end + vel_complement
    double vel_time = 0; //vel_time = d/(t_end - t)
    double vel_complement = 0; // = PID(this->vel_end - vel_time)
    
    
    //Calculate v_target
    //First, calculate vel_time
    if (tmp_t < 10)      
      {	
      	vel_time = 0;
      	vel_target = 0;
      	this->x_orig = current_x;
      }
    
    else if(tmp_t < this->time_end && this->d_sum < this->d)
  	{
  		vel_time = (this->d - this->d_sum)/(this->time_end - tmp_t);
  		
  		vel_complement = CartDemoPlugin::pidUpdate(TRACK, vel_time - this->vel_end,stepTime);
  		vel_complement = vel_complement > 10 ? 10 :
        (this->gas_force < -10 ? -10 : vel_target);
  		vel_target = this->vel_end - vel_complement;
  		
  		gzdbg	<<"\tNOW:\t"<<tmp_t
  					<<"\td\t"<<d
  					<<"\tx_orig\t"<<this->x_orig
  					<<"Distance_by_angle"<<d_sum
  					<<"\tDelta x:\t"<<current_x - this->x_orig
  					<<"\tDelta Time:\t"<<this->time_end - tmp_t<<"\n"
  					<<"vel_time:\t"<<vel_time<<"\tvel_complement\t"<<vel_complement
  					<<"\tvel_target:\t"<<vel_target
  					<<"\tvel_current\t"<<vel_curr<<"\n";
  	}
	   
	  //Abort system if over ending
	  else if (tmp_t > this->time_end || this->d_sum > (this->d + 0.01))
	  {
	  	vel_target = 0;
	  }
	  //If finish planning, write down result and abort system 
	  else
	  {
	  	
	  	ofstream myfile;
	  	myfile.open ("trackOut.csv", ios::out| ios::trunc);  //New or replace existing file
  		myfile << tmp_t<<"\tvel_end in real\t"<<vel_curr<<"\tDistance:\t"
  			<<this->d_sum<<"\tDistance_target:\t"<<d<<"\tTime end:\t"
  			<<tmp_t<<"\ttime_end_target\t"<<this->time_end
	  		<<"\tvel_target\t"<<vel_target<<"\n";
	  	
	  	
	  }    
    
    /*Now, compute setpoints for gas and brake pids*/
    double vel_err = vel_curr - vel_target;
    double max_cmd = 20.0;
    double eff;
    if(current_x >-30.0 && current_z <= 5.72 )
    {
    	gasType = GASFLAT;
    	brakeType = BRAKEFLAT;
    	cAlpha = 1.0;
    	sAlpha = 0;
    }
    	
    else if(current_x >-30.0 && 5.72<current_z<10.7)
    {
    	gasType = GASDOWN;
    	brakeType = BRAKEDOWN;
    	cAlpha = 16/sqrt(16*16 + 4.804*4.804);
    	sAlpha = -sqrt(1-cAlpha*cAlpha); //Down
    }
    else if(current_z >= 10.7)
    	{
    	gasType = GASFLAT;
    	brakeType = BRAKEFLAT;
    	cAlpha = 1.0;
    	sAlpha = 0;
    }
    else if(current_x < -30.0 && 5.72<current_z<10.7)
    {
    	gasType = GASUP;
    	brakeType = BRAKEUP;
    	cAlpha = 26/sqrt(26*26 + 4.804*4.804);
    	sAlpha = sqrt(1-cAlpha*cAlpha);  //UP
    }
    
    else
     {
    	gasType = GASFLAT;
    	brakeType = BRAKEFLAT;
    	cAlpha = 1.0;
    	sAlpha = 0;
    }
    
    if(vel_err <= 0.05)
    {
     this->brake_force = 0;	
     this->gas_force = CartDemoPlugin::pidUpdate(gasType, vel_err, stepTime.Double());
      this->gas_force = this->gas_force > max_cmd ? max_cmd :
        (this->gas_force < -max_cmd ? -max_cmd : this->gas_force);
    }
    
    else if (vel_err > 0.05)
    {
     this->gas_force = 0;
     this->brake_force = CartDemoPlugin::pidUpdate(brakeType, vel_err, stepTime.Double());
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
    
    if(vel_curr < 0.001) 
    	vel_curr = 0;
     //Force applied to wheels = this->gas_force - this->brake_force - airResistantForce - FrictionForce    
     eff =  this->gas_force + this->brake_force*abs(vel_curr)/abs(vel_curr + 0.0001) - 
	      vel_curr*0.002745 - vel_curr*0.01*9.8*25.5/abs(vel_curr + 0.00001);
	      
	  //Sum up traveled distance
	  this->d_sum = this->joints[1]->GetAngle(0).Radian()*0.2*5;
        
   	for (int i = 1; i < NUM_JOINTS; i++)
    {
    
    this->joints[i]->SetForce(0, eff );
//this->joints[i]->SetMaxForce(1, 0.5); 
    }
    gzdbg << "\n";
 
  ////////////////////////////////////////////////
	/* Out put parameters to file "pidOut.csv"*/
  if(tmp_t*10 == round(tmp_t*10) ) //sample each 0.1 second
  {
  	ofstream myfile;
  	myfile.open ("pidOut.csv", ios::out| ios::app);  //Append to existing file
  	myfile << tmp_t<<"\tvel\t"<< this->joints[1]->GetVelocity(0)<<"\tEffort\t"<<eff
  		<<"\tvel_target\t"<<vel_target<<"\n";
 	}
}
