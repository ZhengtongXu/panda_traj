#include "traj_following.cpp"
#include <boost/date_time.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <math.h>
#include <stdio.h>
#include <iostream>

using namespace Eigen;
class motion_primitive{
	private:
		double t_total;
		Vector3d end_p, start_p;
		Quaterniond end_q, start_q;
		double curr_t;
		Vector3d target_p;
		Quaterniond target_q;
	
	public:
		motion_primitive(double t_total_, const Vector3d &start_p_, const Vector3d &end_p_, const Quaterniond &start_q_, const Quaterniond &end_q_);
		double scalar_generation(double t_);
		const Vector3d get_target_p(){return target_p;};
		const Quaterniond get_target_q(){return target_q;};
		void position_traj_generation();
		void orientation_traj_generation();
		void time_update(double t_){curr_t = t_;};
};

motion_primitive::motion_primitive(double t_total_, const Vector3d &start_p_, const Vector3d &end_p_, const Quaterniond &start_q_, const Quaterniond &end_q_){
	t_total = t_total_;
	end_p = end_p_;
	start_p = start_p_;
	end_q = end_q_;
	start_q = start_q_;
	curr_t = 0;
}

double motion_primitive::scalar_generation(double t_){
	//Polynomial trajectory.
	if (t_<=(t_total)){
		double t = t_total;
		double a0 = 0;
		double a1 = 0;
		double a2 = 0;
		double a3 = (1/(2*t*t*t))*(20);
		double a4 = (1/(2*t*t*t*t))*(-30);
		double a5 = (1/(2*t*t*t*t*t))*(12);
		return a0 + a1*t_ + a2*t_*t_ +a3*t_*t_*t_ + a4*t_*t_*t_*t_ + a5*t_*t_*t_*t_*t_;
	}else if (t_>(t_total)){
		return 1;
	}
}

void motion_primitive::position_traj_generation(){
	target_p = start_p + scalar_generation(curr_t)*(end_p-start_p);
}

void motion_primitive::orientation_traj_generation(){
	target_q = start_q.slerp(scalar_generation(curr_t), end_q);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "talker");
	traj_following trajFollowing(argc,argv);
	int frequency = 150;
	trajFollowing.set_frequency(frequency);
	ros::Rate loop_rate(frequency);
	
	double ini_time = 2;
	for( int i = 0; i < int(ini_time*frequency); i ++ ){
       	ros::spinOnce();
       	loop_rate.sleep();
   	}
	geometry_msgs::PoseStamped ini_joint_angle = trajFollowing.get_joint_angle();
        KDL::JntArray ini_joint_target(7);
	ini_joint_target(0) = ini_joint_angle.pose.orientation.w;
	ini_joint_target(1) = ini_joint_angle.pose.orientation.x;
	ini_joint_target(2) = ini_joint_angle.pose.orientation.y;
	ini_joint_target(3) = ini_joint_angle.pose.orientation.z;
	ini_joint_target(4) = ini_joint_angle.pose.position.x;
	ini_joint_target(5) = ini_joint_angle.pose.position.y;
	ini_joint_target(6) = ini_joint_angle.pose.position.z;
	
	trajFollowing.ini_solver(ini_joint_target);
	KDL::Frame ini_end;
	ini_end = trajFollowing.fk(ini_joint_target);

	double qua_w,qua_x,qua_y,qua_z,vec_x,vec_y,vec_z;

	ini_end.M.GetQuaternion(qua_x,qua_y,qua_z,qua_w);
	vec_x = ini_end.p.x();
	vec_y = ini_end.p.y();
	vec_z = ini_end.p.z();

   	Quaterniond q1(qua_w,qua_x,qua_y,qua_z);
	Quaterniond q2(1,0,0,0);
	Vector3d p1(vec_x,vec_y,vec_z);
	Vector3d p2(vec_x-0.15,vec_y,vec_z+0.1);
	
	motion_primitive motionPrimitive(3,p1,p2,q1,q2);

	double start_sec = 0;
	double end_sec = 0;
	double n = 1;
	while (ros::ok()){
		start_sec = ros::Time::now().toSec();
		motionPrimitive.time_update(n/frequency);
		motionPrimitive.position_traj_generation();
		motionPrimitive.orientation_traj_generation();
	
		KDL::Vector p_target(motionPrimitive.get_target_p()(0),motionPrimitive.get_target_p()(1),motionPrimitive.get_target_p()(2));
		KDL::Rotation r_target = KDL::Rotation::Quaternion(motionPrimitive.get_target_q().normalized().x(),
														   motionPrimitive.get_target_q().normalized().y(),
														   motionPrimitive.get_target_q().normalized().z(),
														   motionPrimitive.get_target_q().normalized().w());
		KDL::Frame end_effe_target(r_target,p_target);
		trajFollowing.one_loop(end_effe_target);
		n++;
		ros::spinOnce();	
		
		end_sec = ros::Time::now().toSec();
		//std::cout<<"time per loop: "<<end_sec-start_sec<<std::endl;
		loop_rate.sleep();		
	}	
  	return 0;
}
