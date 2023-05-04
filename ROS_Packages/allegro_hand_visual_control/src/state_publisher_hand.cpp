/*
This ROS-node (state_publisher_hand) publishes joint positions for an Allegro hand (right hand). 
The two dimensional pixel positions of the hand's joints in live video are constantly read through a subscription to "ros_openpose". 
The video is provided by a ZED camera. 
Calculated joint angles, based on the monitored joint positions are first sent through a Kalman filter and the to the Allegro hand. 
This allows the hand to imitate different grasps, using 2, 3, and 4 fingers.
Program can be shut down by demostrating a peace sign (only index and middle finger are extended and spread apart in a V-pose).
*/

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <ros_openpose/Frame.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64MultiArray.h> 
using namespace std;

// Constants
const int size_moving_average = 5;  // Size of the arrays which are used for building the average values
const int DOF_JOINTS = 16;          // Degrees of freedom of the Allegro hand
const double pi = 3.14159265359;

// Class for publishing joint positions for the Allegro hand.
class State_Publisher_Hand{
    public:
        void initialize_system(ros::NodeHandle *n);
        void hand_startup(ros::Rate loop_rate);
        void loop(void);
        int get_classifier(void);
    private:
        ros::Publisher joint_prekalman_pub;
        ros::Publisher joint_pub;
        ros::Subscriber sub;
        ros::Subscriber sub_kalman;
        ros::Subscriber measured_joints_sub;
        ros_openpose::Pixel recorded_positions [21];
        float recorded_accuracy [21] = {0};
        float kalman_filtered_angle_positions [16] = {0.0, 0.1, 0.4, 0.1, 0.0, 0.1, 0.4, 0.1, 0.0, 0.1, 0.4, 0.1, pi/4, pi/4, pi/4, -pi/4};
        float data_index [size_moving_average][10] = {0};
        float data_middle [size_moving_average][10] = {0};
        float data_ring [size_moving_average][10] = {0};
        float data_pinkie [size_moving_average][10] = {0};
        float data_thumb [size_moving_average][10] = {0};
        int array_filled_counter = 0;
        float data_accuracy [size_moving_average][24] = {0};
        int classifier = 0;

        sensor_msgs::JointState joint_prekalman_state;  // calculated joint states (angles), which are sent to a Kalman filter
        sensor_msgs::JointState joint_state;            // final joint states (angles) which are sent to the Allegro hand
        sensor_msgs::JointState measured_joints;        // measured angles sent by Allegro hand
        std::string jointNames[DOF_JOINTS] =
        {
                "joint_0.0", "joint_1.0", "joint_2.0", "joint_3.0",
                "joint_4.0", "joint_5.0", "joint_6.0", "joint_7.0",
                "joint_8.0", "joint_9.0", "joint_10.0", "joint_11.0",
                "joint_12.0", "joint_13.0", "joint_14.0", "joint_15.0"
        };
        std::vector<float> calculated_joint_angles;     // finger angles which are calculated in function calculate_joints

        void FrameOpenposeCallback(const ros_openpose::Frame& msg);
        void KalmanCallback(const std_msgs::Float64MultiArray& msg);
        void MeasuredJointsCallback(const sensor_msgs::JointState& msg);
        std::vector<float> calculate_joints(void);
};


// Function to initialize publishers, subscribers and joint positions of the Allegro hand
void State_Publisher_Hand::initialize_system(ros::NodeHandle *n) 
{   
    // Create a publisher for joint states before passing through the Kalman filter
    joint_prekalman_pub = n->advertise<sensor_msgs::JointState>("joint_prekalman_states", 1000);
    // Create a publisher for final joint states to be sent to the Allegro hand
    joint_pub = n->advertise<sensor_msgs::JointState>("allegroHand/joint_cmd", 1000);

    // Subscribe to frame data from rosOpenpose
    sub = n->subscribe("/frame", 1, &State_Publisher_Hand::FrameOpenposeCallback, this);
    // Subscribe to data from Kalman filter
    sub_kalman = n->subscribe("/joint_postkalman_states", 1, &State_Publisher_Hand::KalmanCallback, this);
    // Subscribe to measured joint states of the Allegro hand
    measured_joints_sub = n->subscribe("/allegroHand/joint_states", 1, &State_Publisher_Hand::MeasuredJointsCallback, this);

    // Resize and set the timestamp for the joint_prekalman_state (angles in rad)
    joint_prekalman_state.position.resize(DOF_JOINTS);
    joint_prekalman_state.name.resize(DOF_JOINTS);
    joint_prekalman_state.header.stamp = ros::Time::now();

    // Resize and set the timestamp for the joint_state (angles in rad)
    joint_state.position.resize(DOF_JOINTS);
    joint_state.name.resize(DOF_JOINTS);
    joint_state.header.stamp = ros::Time::now();

    // Resize and set the timestamp for the measured_joints (angles in rad), which are sent by the allegro_hand
    measured_joints.position.resize(DOF_JOINTS);
    measured_joints.velocity.resize(DOF_JOINTS);
    measured_joints.name.resize(DOF_JOINTS);
    measured_joints.header.stamp = ros::Time::now();
    
    // Assign names to each joint in the pre-Kalman states, final joint states and measured joint states
    for (int i = 0; i < DOF_JOINTS; i++) 
    {
        joint_prekalman_state.name[i] = jointNames[i];
        joint_state.name[i] = jointNames[i];
        measured_joints.name[i] = jointNames[i];
    }
}

// Function for updating the pixel positions of each joint received by ros_openpose
// The pixel positions in the video of the ZED camera are saved
void State_Publisher_Hand::FrameOpenposeCallback(const ros_openpose::Frame& msg)
{ 
    if (msg.persons.empty())
    {
        ROS_WARN("No person was detected.");
    }
    else
    {
        for(int i = 0; i < 21; i++)
        {
            recorded_positions[i] = msg.persons[0].rightHandParts[i].pixel;
            recorded_accuracy[i] = msg.persons[0].rightHandParts[i].score;
        }
    }
}

// Callback function for the kalman-filtered joint positions (angles in rad)
void State_Publisher_Hand::KalmanCallback(const std_msgs::Float64MultiArray& msg)
{
    for (int i = 0; i<16; i++)
    {
        kalman_filtered_angle_positions[i] = msg.data[i]; 
    }  
}

// Callback function for the current joint position (angles in rad), which are sent by the Allegro hand
void State_Publisher_Hand::MeasuredJointsCallback(const sensor_msgs::JointState& msg)
{
    for (int i = 0; i<16; i++)
    {
        measured_joints.position[i] = msg.position[i];
    }
    for (int i = 0; i<16; i++)
    {
        measured_joints.velocity[i] = msg.velocity[i];
    }
}

//Function, which slowly brings the hand into a relaxed position at startup of the program
void State_Publisher_Hand::hand_startup(ros::Rate loop_rate)
{
    
    int i = 0;
    while(i<10)  //wait a short while to measure actual start_position of joints
    {
        loop_rate.sleep();
        ros::spinOnce();
        i++;
    } 
    
    float desired_starting_positions [16] = {0.0, 0.1, 0.4, 0.1, 0.0, 0.1, 0.4, 0.1, 0.0, 0.1, 0.4, 0.1, pi/4, pi/4, pi/4, -pi/3};
    float actual_starting_positions [16] = {0.0};
    
    float increment = 0.0;
    joint_state.header.stamp = ros::Time::now();
    
    for (int i = 0; i < DOF_JOINTS; i++)
    {
        actual_starting_positions[i] = measured_joints.position[i];
    } 

    while (increment<1)
    {
        increment+=0.025;
        for (int i = 0; i < DOF_JOINTS; i++)
        {
            joint_state.position[i] = actual_starting_positions[i]+(desired_starting_positions[i]-actual_starting_positions[i])*increment;
        }
        joint_pub.publish(joint_state);
        loop_rate.sleep();
        ros::spinOnce();
    }

    for (int i = 0; i < DOF_JOINTS; i++)
    {
        joint_prekalman_state.position[i] = measured_joints.position[i];
    }
    
}

/* This function publishes the joint positions for the simulation.
First it is being classified which grasp is being executed (2,3,4-finger grasp or none)
and then the angles are calculated accordingly. */
void State_Publisher_Hand::loop(void)
{
    joint_state.header.stamp = ros::Time::now();
    calculated_joint_angles = calculate_joints();

    double distance_tips_thumb_index = sqrt(pow(recorded_positions[4].x-recorded_positions[8].x,2)+pow(recorded_positions[4].y-recorded_positions[8].y,2));
    double distance_knuckle_joint1_index = sqrt(pow(recorded_positions[5].x-recorded_positions[6].x,2)+pow(recorded_positions[5].y-recorded_positions[6].y,2));
    double distance_tips_thumb_middle = sqrt(pow(recorded_positions[4].x-recorded_positions[12].x,2)+pow(recorded_positions[4].y-recorded_positions[12].y,2));
    double distance_knuckle_joint1_middle = sqrt(pow(recorded_positions[9].x-recorded_positions[10].x,2)+pow(recorded_positions[9].y-recorded_positions[10].y,2));
    double distance_tips_thumb_ring = sqrt(pow(recorded_positions[4].x-recorded_positions[16].x,2)+pow(recorded_positions[4].y-recorded_positions[16].y,2));
    double distance_tips_thumb_pinkie = sqrt(pow(recorded_positions[4].x-recorded_positions[20].x,2)+pow(recorded_positions[4].y-recorded_positions[20].y,2));
    
    if(calculated_joint_angles[1]<0.05 && calculated_joint_angles[3]<0.05 && calculated_joint_angles[5]<0.05)
    {
        classifier = 0;
        ROS_INFO("No grasp detected.");
        
    }
    else if(abs(calculated_joint_angles[1]-calculated_joint_angles[3])<0.2 && abs(calculated_joint_angles[1]-calculated_joint_angles[5])<0.2  
        && calculated_joint_angles[1]>0.1 && calculated_joint_angles[3]>0.1 && calculated_joint_angles[5]>0.1
        && distance_tips_thumb_pinkie < 10*distance_tips_thumb_index)
    { 
        classifier = 3;
        ROS_INFO("4-finger enclosing grasp detected.");
    }
    
    else if(distance_tips_thumb_index*3 < distance_tips_thumb_pinkie && distance_tips_thumb_middle*3 < distance_tips_thumb_pinkie &&
        distance_tips_thumb_index < distance_knuckle_joint1_index*2 && calculated_joint_angles[1]>0.3 && calculated_joint_angles[3]>0.3 
        && calculated_joint_angles[5]<0.4)
    /*
    else if(distance_tips_thumb_index*2 < distance_tips_thumb_pinkie && distance_tips_thumb_middle < distance_tips_thumb_pinkie
        && calculated_joint_angles[1]>pi/8 && calculated_joint_angles[3]>pi/8)
        */
    {
        classifier = 2;
        ROS_INFO("3-fingers pinch grasp detected.");
    } 
    else if(distance_tips_thumb_index*3 < distance_tips_thumb_middle && distance_tips_thumb_index < distance_knuckle_joint1_index*2
        && calculated_joint_angles[1]>0.3 && calculated_joint_angles[5]<0.2 && calculated_joint_angles[3]<0.3)
    {
        classifier = 1;
        ROS_INFO("2-fingers pinch grasp detected.");
    } 
    else if(distance_tips_thumb_ring < distance_knuckle_joint1_index && distance_tips_thumb_pinkie < distance_knuckle_joint1_index 
        && distance_tips_thumb_index > 2*distance_knuckle_joint1_index && distance_tips_thumb_middle > 2*distance_knuckle_joint1_index
        && calculated_joint_angles[5] > 0.5)
    {
        classifier = 4; //peace gesture
        ROS_INFO("Peace gesture detected. => Shutting down system.");
    }
    
    
    if (classifier == 0)
    {
        joint_prekalman_state.position[3] = 0.1;
        joint_prekalman_state.position[2] = 0.4;
        joint_prekalman_state.position[1] = 0.1;
        joint_prekalman_state.position[0] = 0.0;
        joint_prekalman_state.position[7] = 0.1;
        joint_prekalman_state.position[6] = 0.4;
        joint_prekalman_state.position[5] = 0.1;
        joint_prekalman_state.position[4] = 0.0;
        joint_prekalman_state.position[11] = 0.1;
        joint_prekalman_state.position[10] = 0.4;
        joint_prekalman_state.position[9] = 0.1;
        joint_prekalman_state.position[8] = 0.0;
        joint_prekalman_state.position[12] = pi/4;
        joint_prekalman_state.position[13] = pi/4;
        joint_prekalman_state.position[14] = pi/4;
        joint_prekalman_state.position[15] = -pi/3;
    }
    else if (classifier == 1)
    {
        if (calculated_joint_angles[1]>0.6)
        {
            calculated_joint_angles[1] = 0.6;
        }
        joint_prekalman_state.position[0] = 0.1;
        joint_prekalman_state.position[3] = 1.5*calculated_joint_angles[1]*1.2;
        joint_prekalman_state.position[2] = 2*calculated_joint_angles[1]*1.2;
        joint_prekalman_state.position[1] = 1*calculated_joint_angles[1]*1.2;
        joint_prekalman_state.position[0] = 0.0;
        joint_prekalman_state.position[7] = 0.1;
        joint_prekalman_state.position[6] = 0.4;
        joint_prekalman_state.position[5] = 0.1;
        joint_prekalman_state.position[4] = 0.0;
        joint_prekalman_state.position[11] = 0.1;
        joint_prekalman_state.position[10] = 0.4;
        joint_prekalman_state.position[9] = 0.1;
        joint_prekalman_state.position[8] = 0.0;
        joint_prekalman_state.position[12] = 5*pi/16;
        joint_prekalman_state.position[13] = 0.3;
        joint_prekalman_state.position[14] = 1.4*calculated_joint_angles[1]*1.2;
        joint_prekalman_state.position[15] = -pi/2+1.7*calculated_joint_angles[1]*1.2;
    }
    else if (classifier == 2)
    {
        if (calculated_joint_angles[1]>0.9)
        {
            calculated_joint_angles[1] = 0.9;
        }
        joint_prekalman_state.position[0] = 0.1-pi/20;;
        joint_prekalman_state.position[4] = 0.1+pi/20;;
        joint_prekalman_state.position[3] = 1.5*calculated_joint_angles[1]*0.9;
        joint_prekalman_state.position[2] = 2*calculated_joint_angles[1]*0.9;
        joint_prekalman_state.position[1] = 1*calculated_joint_angles[1]*0.9;
        joint_prekalman_state.position[7] = 1.5*calculated_joint_angles[1]*0.9;
        joint_prekalman_state.position[6] = 2*calculated_joint_angles[1]*0.9;
        joint_prekalman_state.position[5] = 1*calculated_joint_angles[1]*0.9;
        joint_prekalman_state.position[11] = 0.1;
        joint_prekalman_state.position[10] = 0.4;
        joint_prekalman_state.position[9] = 0.1;
        joint_prekalman_state.position[8] = 0.0;
        joint_prekalman_state.position[12] = 7.0*pi/16;;
        joint_prekalman_state.position[13] = 0.2;
        joint_prekalman_state.position[14] = 1.6*calculated_joint_angles[1]*0.9;
        joint_prekalman_state.position[15] = -pi/2+1.6*calculated_joint_angles[1]*0.9;
    }
    else if (classifier == 3)
    {
        if (calculated_joint_angles[1]>1.0)
        {
            calculated_joint_angles[1] = 1.0;
        }
        joint_prekalman_state.position[3] = 0.8*calculated_joint_angles[1];
        joint_prekalman_state.position[2] = 1.6*calculated_joint_angles[1];
        joint_prekalman_state.position[1] = 1.6*calculated_joint_angles[1];
        joint_prekalman_state.position[0] = 0.1;
        joint_prekalman_state.position[7] = 0.8*calculated_joint_angles[1];
        joint_prekalman_state.position[6] = 1.6*calculated_joint_angles[1];
        joint_prekalman_state.position[5] = 1.6*calculated_joint_angles[1];
        joint_prekalman_state.position[4] = 0.1;
        joint_prekalman_state.position[11] = 0.8*calculated_joint_angles[1];
        joint_prekalman_state.position[10] = 1.6*calculated_joint_angles[1];
        joint_prekalman_state.position[9] = 1.6*calculated_joint_angles[1];
        joint_prekalman_state.position[8] = 0.1;
        
        joint_prekalman_state.position[12] = 7*pi/16;
        joint_prekalman_state.position[13] = 0.0;
        
        if (calculated_joint_angles[1]>0.4)
        {
            calculated_joint_angles[1] = 0.4;
        }
        joint_prekalman_state.position[14] = 0.7*calculated_joint_angles[1];
        joint_prekalman_state.position[15] = -pi/2+1.6*calculated_joint_angles[1];
    }
    else if (classifier == 4)
    {
        joint_prekalman_state.position[3] = 0.0;
        joint_prekalman_state.position[2] = 0.0;
        joint_prekalman_state.position[1] = 0.0;
        joint_prekalman_state.position[0] = pi/16;
        joint_prekalman_state.position[7] = 0.0;
        joint_prekalman_state.position[6] = 0.0;
        joint_prekalman_state.position[5] = 0.0;
        joint_prekalman_state.position[4] = -pi/16;
        joint_prekalman_state.position[11] = 0.8*0.78;
        joint_prekalman_state.position[10] = 1.6*0.78;
        joint_prekalman_state.position[9] = 1.6*0.78;
        joint_prekalman_state.position[8] = 0.3;
        
        joint_prekalman_state.position[12] = 7*pi/16;
        joint_prekalman_state.position[13] = 3*pi/8;
        joint_prekalman_state.position[14] = 1.0*0.4;
        joint_prekalman_state.position[15] = -pi/2+3.0*0.4;
    }

    joint_prekalman_pub.publish(joint_prekalman_state);
    
    for (int i = 0; i < DOF_JOINTS; i++)
    {
        joint_state.position[i] = kalman_filtered_angle_positions[i];
    }

    joint_pub.publish(joint_state);
}

int State_Publisher_Hand::get_classifier(void)
{
    return classifier;
}

// Function for calculating joint angles by using the pixel positions of the joints obtained from ros_openpose.
// A moving average filter is being used, with the input being weighted according to their accuracy declared by ros_openpose 
std::vector<float> State_Publisher_Hand::calculate_joints(void) 
{
    std::vector<float> result(8);

    // Only do something if data for the right hand is received from Openpose with a minimum certainty 
    if(recorded_accuracy[0]>0.05)
    {
        // Writing current positions received from Openpose into the data arrays.
        // A moving average filter is created by writing the points into the position [array_filled_counter % size_moving_average] 
        // of the data array and incrementing the array_filled_counter afterwards.
        // The accuracy of those positions (also sent by ros-openpose) is also saved.
        data_index[array_filled_counter % size_moving_average][0] = recorded_positions[0].x;
        data_index[array_filled_counter % size_moving_average][1] = recorded_positions[0].y;
        data_accuracy[array_filled_counter % size_moving_average][4] = recorded_accuracy[0]; //accuracy of the wrist
        for(int i = 1; i < 5; i++)
            {
                data_accuracy[array_filled_counter % size_moving_average][i+4] = recorded_accuracy[i+4];
                data_index[array_filled_counter % size_moving_average][2*i] = recorded_positions[i+4].x;
                data_index[array_filled_counter % size_moving_average][2*i+1] = recorded_positions[i+4].y;
            }
        data_middle[array_filled_counter % size_moving_average][0] = recorded_positions[0].x;
        data_middle[array_filled_counter % size_moving_average][1] = recorded_positions[0].y;
        data_accuracy[array_filled_counter % size_moving_average][9] = recorded_accuracy[0]; //accuracy of the wrist
        for(int i = 1; i < 5; i++)
            {
                data_accuracy[array_filled_counter % size_moving_average][i+9] = recorded_accuracy[i+8];
                data_middle[array_filled_counter % size_moving_average][2*i] = recorded_positions[i+8].x;
                data_middle[array_filled_counter % size_moving_average][2*i+1] = recorded_positions[i+8].y;
            }
        data_ring[array_filled_counter % size_moving_average][0] = recorded_positions[0].x;
        data_ring[array_filled_counter % size_moving_average][1] = recorded_positions[0].y;
        data_accuracy[array_filled_counter % size_moving_average][14] = recorded_accuracy[0];
        for(int i = 1; i < 5; i++)
            {
                data_accuracy[array_filled_counter % size_moving_average][i+14] = recorded_accuracy[i+12];
                data_ring[array_filled_counter % size_moving_average][2*i] = recorded_positions[i+12].x;
                data_ring[array_filled_counter % size_moving_average][2*i+1] = recorded_positions[i+12].y;
            }

        data_pinkie[array_filled_counter % size_moving_average][0] = recorded_positions[0].x;
        data_pinkie[array_filled_counter % size_moving_average][1] = recorded_positions[0].y;
        data_accuracy[array_filled_counter % size_moving_average][19] = recorded_accuracy[0];
        for(int i = 1; i < 5; i++)
            {
                data_accuracy[array_filled_counter % size_moving_average][i+19] = recorded_accuracy[i+16];
                data_pinkie[array_filled_counter % size_moving_average][2*i] = recorded_positions[i+16].x;
                data_pinkie[array_filled_counter % size_moving_average][2*i+1] = recorded_positions[i+16].y;
            }
        data_thumb[array_filled_counter % size_moving_average][0] = recorded_positions[0].x;
        data_thumb[array_filled_counter % size_moving_average][1] = recorded_positions[0].y;
        data_accuracy[array_filled_counter % size_moving_average][0] = recorded_accuracy[0];
        for(int i = 2; i < 5; i++)
        {
            data_accuracy[array_filled_counter % size_moving_average][i-1] = recorded_accuracy[i-1];
            data_thumb[array_filled_counter % size_moving_average][2*i-2] = recorded_positions[i].x;
            data_thumb[array_filled_counter % size_moving_average][2*i-1] = recorded_positions[i].y;
        }        
        array_filled_counter += 1;
        
        //Start calculating only once array is filled all the way
        if (array_filled_counter > size_moving_average)
        {   
            // Here, the joint angles are calculated for index, middle and ring finger and the thumb, using the above filled 
            // data arrays and their accuracies.

            //------------------Index finger---------------------------------------------------------------------------------------------------
            double avg_values_index[10] = {0};
            for(int i = 0; i < 10; i++)
            {
                float addedup_accuracy = 0;

                // The accuracy of all the points of a certain joint (i-th joint) in the data array are added up.
                for (int k = 0; k < size_moving_average; k++)
                {
                    if (i%2 == 0)
                    {
                        addedup_accuracy = addedup_accuracy + data_accuracy[k][i/2+4];
                    }
                    else
                    {
                        addedup_accuracy = addedup_accuracy + data_accuracy[k][(i-1)/2+4];
                    }
                }
                

                // The points of the data array are weighted according to their accuracy and from that an average value is calculated.
                for(int j = 0; j < size_moving_average; j++)
                {
                    if (i%2 == 0)
                    {
                        avg_values_index[i] = avg_values_index[i] + data_index[j][i]*data_accuracy[j][i/2+4]/addedup_accuracy;  
                    }
                    else
                    {
                        avg_values_index[i] = avg_values_index[i] + data_index[j][i]*data_accuracy[j][(i-1)/2+4]/addedup_accuracy;  
                    } 
                }
            }

            //Calculating joint_knuckle by using positions of wrist, knuckle and joint1
            double b2_joint_knuckle = pow(avg_values_index[4]-avg_values_index[2],2)+pow(avg_values_index[5]-avg_values_index[3],2);
            double c2_joint_knuckle = pow(avg_values_index[2]-avg_values_index[0],2)+pow(avg_values_index[3]-avg_values_index[1],2);
            double a2_joint_knuckle = pow(avg_values_index[4]-avg_values_index[0],2)+pow(avg_values_index[5]-avg_values_index[1],2);
            result[0] = 3.14 -acos((b2_joint_knuckle+c2_joint_knuckle-a2_joint_knuckle)/(2*sqrt(b2_joint_knuckle)*sqrt(c2_joint_knuckle)));
            

            //Calculating joint2 by using positions of knuckle, joint1 and tip of index
            double b2_joint2 = pow(avg_values_index[8]-avg_values_index[4],2)+pow(avg_values_index[9]-avg_values_index[5],2);
            double c2_joint2 = pow(avg_values_index[4]-avg_values_index[2],2)+pow(avg_values_index[5]-avg_values_index[3],2);
            double a2_joint2 = pow(avg_values_index[8]-avg_values_index[2],2)+pow(avg_values_index[9]-avg_values_index[3],2);
            result[1] = (3.14-acos((b2_joint2+c2_joint2-a2_joint2)/(2*sqrt(b2_joint2)*sqrt(c2_joint2))))/3;
            //Calculating joint2 by using positions of joint1, joint2 and tip of index
            b2_joint2 = pow(avg_values_index[8]-avg_values_index[6],2)+pow(avg_values_index[9]-avg_values_index[7],2);
            c2_joint2 = pow(avg_values_index[6]-avg_values_index[4],2)+pow(avg_values_index[7]-avg_values_index[5],2);
            a2_joint2 = pow(avg_values_index[8]-avg_values_index[4],2)+pow(avg_values_index[9]-avg_values_index[5],2);
            //using the average of both calculation methods of angle of joint2
            result[1] = (result[1] + (3.14 -acos((b2_joint2+c2_joint2-a2_joint2)/(2*sqrt(b2_joint2)*sqrt(c2_joint2)))))/2;

            //------------------Middle finger---------------------------------------------------------------------------------------------------
            double avg_values_middle[10] = {};

            for(int i = 0; i < 10; i++)
            {
                float addedup_accuracy = 0;
                for (int k = 0; k < size_moving_average; k++)
                {
                    if (i%2 == 0)
                    {
                        addedup_accuracy = addedup_accuracy + data_accuracy[k][i/2+9];
                    }
                    else
                    {
                        addedup_accuracy = addedup_accuracy + data_accuracy[k][(i-1)/2+9];
                    }
                }

                for(int j = 0; j < size_moving_average; j++)
                {
                    if (i%2 == 0)
                    {
                        avg_values_middle[i] = avg_values_middle[i] + data_middle[j][i]*data_accuracy[j][i/2+9]/addedup_accuracy;  
                    }
                    else
                    {
                        avg_values_middle[i] = avg_values_middle[i] + data_middle[j][i]*data_accuracy[j][(i-1)/2+9]/addedup_accuracy;  
                    }  
                }
            }

            //Calculating joint_knuckle by using positions of wrist, knuckle and joint1
            b2_joint_knuckle = pow(avg_values_middle[4]-avg_values_middle[2],2)+pow(avg_values_middle[5]-avg_values_middle[3],2);
            c2_joint_knuckle = pow(avg_values_middle[2]-avg_values_middle[0],2)+pow(avg_values_middle[3]-avg_values_middle[1],2);
            a2_joint_knuckle = pow(avg_values_middle[4]-avg_values_middle[0],2)+pow(avg_values_middle[5]-avg_values_middle[1],2);
            result[2] = 3.14 -acos((b2_joint_knuckle+c2_joint_knuckle-a2_joint_knuckle)/(2*sqrt(b2_joint_knuckle)*sqrt(c2_joint_knuckle)));

            //Calculating joint2 by using positions of knuckle, joint1 and tip of middle
            b2_joint2 = pow(avg_values_middle[8]-avg_values_middle[4],2)+pow(avg_values_middle[9]-avg_values_middle[5],2);
            c2_joint2 = pow(avg_values_middle[4]-avg_values_middle[2],2)+pow(avg_values_middle[5]-avg_values_middle[3],2);
            a2_joint2 = pow(avg_values_middle[8]-avg_values_middle[2],2)+pow(avg_values_middle[9]-avg_values_middle[3],2);
            result[3] = (3.14-acos((b2_joint2+c2_joint2-a2_joint2)/(2*sqrt(b2_joint2)*sqrt(c2_joint2))))/3;
            //Calculating joint2 by using positions of joint1, joint2 and tip of middle
            b2_joint2 = pow(avg_values_middle[8]-avg_values_middle[6],2)+pow(avg_values_middle[9]-avg_values_middle[7],2);
            c2_joint2 = pow(avg_values_middle[6]-avg_values_middle[4],2)+pow(avg_values_middle[7]-avg_values_middle[5],2);
            a2_joint2 = pow(avg_values_middle[8]-avg_values_middle[4],2)+pow(avg_values_middle[9]-avg_values_middle[5],2);
            //using the average of both calculation methods of angle of joint2
            result[3] = (result[3] + (3.14 -acos((b2_joint2+c2_joint2-a2_joint2)/(2*sqrt(b2_joint2)*sqrt(c2_joint2)))))/2;

            //------------------Ring finger---------------------------------------------------------------------------------------------------
            double avg_values_ring[10] = {};

            for(int i = 0; i < 10; i++)
            {

                float addedup_accuracy = 0;
                for (int k = 0; k < size_moving_average; k++)
                {
                    if (i%2 == 0)
                    {
                        addedup_accuracy = addedup_accuracy + data_accuracy[k][i/2+14];
                    }
                    else
                    {
                        addedup_accuracy = addedup_accuracy + data_accuracy[k][(i-1)/2+14];
                    }
                }

                for(int j = 0; j < size_moving_average; j++)
                {
                    if (i%2 == 0)
                    {
                        avg_values_ring[i] = avg_values_ring[i] + data_ring[j][i]*data_accuracy[j][i/2+14]/addedup_accuracy;  
                    }
                    else
                    {
                        avg_values_ring[i] = avg_values_ring[i] + data_ring[j][i]*data_accuracy[j][(i-1)/2+14]/addedup_accuracy;  
                    }  
                }  
            }

            //Calculating joint_knuckle by using positions of wrist, knuckle and joint1
            b2_joint_knuckle = pow(avg_values_ring[4]-avg_values_ring[2],2)+pow(avg_values_ring[5]-avg_values_ring[3],2);
            c2_joint_knuckle = pow(avg_values_ring[2]-avg_values_ring[0],2)+pow(avg_values_ring[3]-avg_values_ring[1],2);
            a2_joint_knuckle = pow(avg_values_ring[4]-avg_values_ring[0],2)+pow(avg_values_ring[5]-avg_values_ring[1],2);
            result[4] = 3.14 -acos((b2_joint_knuckle+c2_joint_knuckle-a2_joint_knuckle)/(2*sqrt(b2_joint_knuckle)*sqrt(c2_joint_knuckle)));

            //Calculating joint2 by using positions of knuckle, joint1 and tip of ring
            b2_joint2 = pow(avg_values_ring[8]-avg_values_ring[4],2)+pow(avg_values_ring[9]-avg_values_ring[5],2);
            c2_joint2 = pow(avg_values_ring[4]-avg_values_ring[2],2)+pow(avg_values_ring[5]-avg_values_ring[3],2);
            a2_joint2 = pow(avg_values_ring[8]-avg_values_ring[2],2)+pow(avg_values_ring[9]-avg_values_ring[3],2);
            result[5] = (3.14-acos((b2_joint2+c2_joint2-a2_joint2)/(2*sqrt(b2_joint2)*sqrt(c2_joint2))))/3;
            //Calculating joint2 by using positions of joint1, joint2 and tip of ring
            b2_joint2 = pow(avg_values_ring[8]-avg_values_ring[6],2)+pow(avg_values_ring[9]-avg_values_ring[7],2);
            c2_joint2 = pow(avg_values_ring[6]-avg_values_ring[4],2)+pow(avg_values_ring[7]-avg_values_ring[5],2);
            a2_joint2 = pow(avg_values_ring[8]-avg_values_ring[4],2)+pow(avg_values_ring[9]-avg_values_ring[5],2);
            //using the average of both calculation methods of angle of joint2
            result[5] = (result[5] + (3.14 -acos((b2_joint2+c2_joint2-a2_joint2)/(2*sqrt(b2_joint2)*sqrt(c2_joint2)))))/2;

            //------------------Pinkie finger---------------------------------------------------------------------------------------------------
            double avg_values_pinkie[10] = {};

            for(int i = 0; i < 10; i++)
            {
                float addedup_accuracy = 0;
                for (int k = 0; k < size_moving_average; k++)
                {
                    if (i%2 == 0)
                    {
                        addedup_accuracy = addedup_accuracy + data_accuracy[k][i/2+19];
                    }
                    else
                    {
                        addedup_accuracy = addedup_accuracy + data_accuracy[k][(i-1)/2+19];
                    }
                }

                for(int j = 0; j < size_moving_average; j++)
                {
                    if (i%2 == 0)
                    {
                        avg_values_pinkie[i] = avg_values_pinkie[i] + data_pinkie[j][i]*data_accuracy[j][i/2+19]/addedup_accuracy;  
                    }
                    else
                    {
                        avg_values_pinkie[i] = avg_values_pinkie[i] + data_pinkie[j][i]*data_accuracy[j][(i-1)/2+19]/addedup_accuracy;  
                    }  
                } 
            }

            //Calculating joint_knuckle by using positions of wrist, knuckle and joint1
            b2_joint_knuckle = pow(avg_values_pinkie[4]-avg_values_pinkie[2],2)+pow(avg_values_pinkie[5]-avg_values_pinkie[3],2);
            c2_joint_knuckle = pow(avg_values_pinkie[2]-avg_values_pinkie[0],2)+pow(avg_values_pinkie[3]-avg_values_pinkie[1],2);
            a2_joint_knuckle = pow(avg_values_pinkie[4]-avg_values_pinkie[0],2)+pow(avg_values_pinkie[5]-avg_values_pinkie[1],2);
            result[6] = 3.14 -acos((b2_joint_knuckle+c2_joint_knuckle-a2_joint_knuckle)/(2*sqrt(b2_joint_knuckle)*sqrt(c2_joint_knuckle)));

            //Calculating joint2 by using positions of knuckle, joint1 and tip of pinkie
            b2_joint2 = pow(avg_values_pinkie[8]-avg_values_pinkie[4],2)+pow(avg_values_pinkie[9]-avg_values_pinkie[5],2);
            c2_joint2 = pow(avg_values_pinkie[4]-avg_values_pinkie[2],2)+pow(avg_values_pinkie[5]-avg_values_pinkie[3],2);
            a2_joint2 = pow(avg_values_pinkie[8]-avg_values_pinkie[2],2)+pow(avg_values_pinkie[9]-avg_values_pinkie[3],2);
            result[7] = (3.14-acos((b2_joint2+c2_joint2-a2_joint2)/(2*sqrt(b2_joint2)*sqrt(c2_joint2))))/3;
            //Calculating joint2 by using positions of joint1, joint2 and tip of pinkie
            b2_joint2 = pow(avg_values_pinkie[8]-avg_values_pinkie[6],2)+pow(avg_values_pinkie[9]-avg_values_pinkie[7],2);
            c2_joint2 = pow(avg_values_pinkie[6]-avg_values_pinkie[4],2)+pow(avg_values_pinkie[7]-avg_values_pinkie[5],2);
            a2_joint2 = pow(avg_values_pinkie[8]-avg_values_pinkie[4],2)+pow(avg_values_pinkie[9]-avg_values_pinkie[5],2);
            //using the average of both calculation methods of angle of joint2
            result[7] = (result[7] + (3.14 -acos((b2_joint2+c2_joint2-a2_joint2)/(2*sqrt(b2_joint2)*sqrt(c2_joint2)))))/2;

            //------------------Thumb---------------------------------------------------------------------------------------------------
            double avg_values_thumb[8] = {};

            for(int i = 0; i < 8; i++)
            {
                float addedup_accuracy = 0;
                for (int k = 0; k < size_moving_average; k++)
                {
                    if (i%2 == 0)
                    {
                        addedup_accuracy = addedup_accuracy + data_accuracy[k][i/2];
                    }
                    else
                    {
                        addedup_accuracy = addedup_accuracy + data_accuracy[k][(i-1)/2];
                    }
                }

                for(int j = 0; j < size_moving_average; j++)
                {
                    if (i%2 == 0)
                    {
                        avg_values_thumb[i] = avg_values_thumb[i] + data_thumb[j][i]*data_accuracy[j][i/2]/addedup_accuracy;  
                    }
                    else
                    {
                        avg_values_thumb[i] = avg_values_thumb[i] + data_thumb[j][i]*data_accuracy[j][(i-1)/2]/addedup_accuracy;  
                    }  
                }
            }

            //Calculating joint_knuckle by using positions of wrist, knuckle and joint1
            b2_joint_knuckle = pow(avg_values_thumb[4]-avg_values_thumb[2],2)+pow(avg_values_thumb[5]-avg_values_thumb[3],2);
            c2_joint_knuckle = pow(avg_values_thumb[2]-avg_values_thumb[0],2)+pow(avg_values_thumb[3]-avg_values_thumb[1],2);
            a2_joint_knuckle = pow(avg_values_thumb[4]-avg_values_thumb[0],2)+pow(avg_values_thumb[5]-avg_values_thumb[1],2);
            result[8] = 3.14 -acos((b2_joint_knuckle+c2_joint_knuckle-a2_joint_knuckle)/(2*sqrt(b2_joint_knuckle)*sqrt(c2_joint_knuckle)));

            //Calculating joint1 by using positions of knuckle, joint1 and tip of thumb
            double b2_joint1 = pow(avg_values_thumb[6]-avg_values_thumb[4],2)+pow(avg_values_thumb[7]-avg_values_thumb[5],2);
            double c2_joint1 = pow(avg_values_thumb[4]-avg_values_thumb[2],2)+pow(avg_values_thumb[5]-avg_values_thumb[3],2);
            double a2_joint1 = pow(avg_values_thumb[6]-avg_values_thumb[2],2)+pow(avg_values_thumb[7]-avg_values_thumb[3],2);
            result[9] = (3.14-acos((b2_joint1+c2_joint1-a2_joint1)/(2*sqrt(b2_joint1)*sqrt(c2_joint1))))/3;
        }
    }
    return  result;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher_hand");
    ros::NodeHandle n;
    ros::Rate loop_rate(12);
    State_Publisher_Hand state_publisher_hand;
    state_publisher_hand.initialize_system(&n);
    state_publisher_hand.hand_startup(loop_rate);

    while (ros::ok()) {
        state_publisher_hand.loop();
        // If a peace sign (only index and middle finger are extended and spread apart in a V-pose) was detected by the demonstrating person
        // then the program is shut down after 20 loops
        if (state_publisher_hand.get_classifier()==4)
        {
            for(int i = 0; i < 20; i++)
            {
                state_publisher_hand.loop();
                ros::spinOnce();
                loop_rate.sleep();  
            }
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}