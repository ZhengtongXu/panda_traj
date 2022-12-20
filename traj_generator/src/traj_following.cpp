#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/WrenchStamped.h"
#include "sensor_msgs/JointState.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <math.h>
#include <stdio.h>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

using namespace Eigen;
class traj_following{
	private:
        ros::NodeHandle nh;

        ros::Subscriber end_state_sub;
		ros::Subscriber joint_state_sub;
		ros::Publisher joint_velocity_pub;
		ros::Publisher joint_angle_pub;

		//Solver parameters.
  		std::string chain_start, chain_end, urdf_param;
  		double timeout;

		//Commands.
		VectorXd joint_angle_target_last;
		VectorXd joint_angle_target_current;
		VectorXd joint_velocity_target;

		KDL::JntArray solver_result;

		int frequency;
		bool ini_flag;

        void end_state_sub_cb(const geometry_msgs::PoseStamped& msg){
         	end_pose_state = msg;
        };
		void joint_state_sub_cb(const geometry_msgs::PoseStamped& msg){
         	joint_angle_state = msg;
        };
	public:

		geometry_msgs::PoseStamped joint_angle_state;
		geometry_msgs::PoseStamped end_pose_state;

		traj_following(int, char**){
  	        end_state_sub = nh.subscribe("/joint_velocity_interface/end_state", 1, &traj_following::end_state_sub_cb, this);
			joint_state_sub = nh.subscribe("/joint_velocity_interface/joint_state", 1, &traj_following::joint_state_sub_cb, this);
		    joint_velocity_pub = nh.advertise<geometry_msgs::PoseStamped>("/joint_velocity_interface/joint_velocity_target", 1, true);
			joint_angle_pub = nh.advertise<geometry_msgs::PoseStamped>("/joint_velocity_interface/joint_angle_target", 1, true);

  			nh.param("chain_start", chain_start, std::string("panda_link0"));
  			nh.param("chain_end", chain_end, std::string("panda_link8"));
  			if (chain_start == "" || chain_end == ""){
    			ROS_FATAL("Missing chain info in launch file");
    			exit(-1);
  			}
  			nh.param("timeout", timeout, 0.005);
  			nh.param("urdf_param", urdf_param, std::string("/robot_description"));
			ini_flag =  false;
		};
		void ini_solver(const KDL::JntArray& solver_ini){
			solver_result = solver_ini;
			joint_angle_target_last = solver_result.data;
			ini_flag = true;
		};
		void one_loop(const KDL::Frame& end_effe_target){
			if (ini_flag){
				//Use last result as warm start.
				solver_result = ik(end_effe_target,solver_result);
				joint_angle_target_current = solver_result.data;
				joint_velocity_target = (joint_angle_target_current - joint_angle_target_last)*frequency;
				joint_angle_target_last = joint_angle_target_current;
				
				//Publish commands.
				geometry_msgs::PoseStamped joint_position_target_msgs;
				joint_position_target_msgs.pose.orientation.w = joint_angle_target_current(0);
				joint_position_target_msgs.pose.orientation.x = joint_angle_target_current(1);
				joint_position_target_msgs.pose.orientation.y = joint_angle_target_current(2);
				joint_position_target_msgs.pose.orientation.z = joint_angle_target_current(3);
				joint_position_target_msgs.pose.position.x = joint_angle_target_current(4);
				joint_position_target_msgs.pose.position.y = joint_angle_target_current(5);
				joint_position_target_msgs.pose.position.z = joint_angle_target_current(6);

				joint_angle_pub.publish(joint_position_target_msgs);

				geometry_msgs::PoseStamped joint_velocity_target_msgs;
				joint_velocity_target_msgs.pose.orientation.w = joint_velocity_target(0);
				joint_velocity_target_msgs.pose.orientation.x = joint_velocity_target(1);
				joint_velocity_target_msgs.pose.orientation.y = joint_velocity_target(2);
				joint_velocity_target_msgs.pose.orientation.z = joint_velocity_target(3);
				joint_velocity_target_msgs.pose.position.x = joint_velocity_target(4);
				joint_velocity_target_msgs.pose.position.y = joint_velocity_target(5);
				joint_velocity_target_msgs.pose.position.z = joint_velocity_target(6);

				joint_velocity_pub.publish(joint_velocity_target_msgs);
			}
		};


		void set_frequency(int frequency_){ frequency = frequency_;};
		geometry_msgs::PoseStamped get_end_pose(){ return end_pose_state;};
		geometry_msgs::PoseStamped get_joint_angle(){ return joint_angle_state;};

		KDL::Frame fk(const KDL::JntArray& nominal){
			double eps = 1e-6;
			TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, TRAC_IK::Distance);
			KDL::Chain chain;
			KDL::JntArray ll, ul; //lower joint limits, upper joint limits
			bool valid = tracik_solver.getKDLChain(chain);
			if (!valid){
				ROS_ERROR("There was no valid KDL chain found");
			}
			valid = tracik_solver.getKDLLimits(ll, ul);
			if (!valid){
				ROS_ERROR("There were no valid KDL joint limits found");
			}
			std::array<double, 7> _max_joint_positions {
				{2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973}
			};
			std::array<double, 7> _min_joint_positions {
				{-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973}
			};
			for (int i = 0; i<7; i++) 
			{
				ul(i) = _max_joint_positions[i];
				ll(i) = _min_joint_positions[i];
			}
			assert(chain.getNrOfJoints() == ll.data.size());
			assert(chain.getNrOfJoints() == ul.data.size());
			KDL::ChainFkSolverPos_recursive fk_solver(chain);
			KDL::Frame result;
			fk_solver.JntToCart(nominal, result);
			return result;
		};

		KDL::JntArray ik(const KDL::Frame& end_effector_pose,const KDL::JntArray& nominal){
			double eps = 1e-5;
			TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);
			KDL::Chain chain;
			KDL::JntArray ll, ul; //lower joint limits, upper joint limits
			bool valid = tracik_solver.getKDLChain(chain);
			if (!valid){
				ROS_ERROR("There was no valid KDL chain found");
			}
			valid = tracik_solver.getKDLLimits(ll, ul);
			if (!valid){
				ROS_ERROR("There were no valid KDL joint limits found");
			}
			std::array<double, 7> _max_joint_positions {
				{2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973}
			};
			std::array<double, 7> _min_joint_positions {
				{-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973}
			};
			for (int i = 0; i<7; i++) 
			{
				ul(i) = _max_joint_positions[i];
				ll(i) = _min_joint_positions[i];
			}
			assert(chain.getNrOfJoints() == ll.data.size());
			assert(chain.getNrOfJoints() == ul.data.size());
			KDL::ChainFkSolverPos_recursive fk_solver(chain);
			KDL::JntArray nominal_(7);
			if (nominal.data.size() == 8){
				for (uint j = 0; j < nominal_.data.size(); j++){
					nominal_(j) = (ll(j) + ul(j)) / 2.0;
				}
			}else{
				nominal_ = nominal;
			}
			boost::posix_time::ptime start_time;
			boost::posix_time::time_duration diff;
			KDL::JntArray result;
			int rc;
			double total_time = 0;
			double elapsed = 0;
			start_time = boost::posix_time::microsec_clock::local_time();
			rc = tracik_solver.CartToJnt(nominal_, end_effector_pose, result);
			diff = boost::posix_time::microsec_clock::local_time() - start_time;
			elapsed = diff.total_nanoseconds() / 1e9;
			total_time += elapsed;
			//ROS_INFO_STREAM("TRAC-IK found " << total_time << " secs per sample"<<rc<<"!");
			return result;
		};
};