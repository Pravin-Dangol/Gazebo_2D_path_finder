
#include <iostream>
#include <math.h>
#include <algorithm>  

#include "robot.hh"

using std::cout;
using std::endl;

const double goal_x = 20.0;
const double goal_y = 0.0;
bool done = false;

void
callback(Robot* robot)
{
	//===============================================================================
    // PARAMETER INIT AND DECLARATION

	// Odometry from sensor:
    double dx = goal_x - robot->pos_x;
    double dy = goal_y - robot->pos_y;
    double alpha = robot->pos_t; 			// robot's heading, yaw angle (in rads)
    double theta = atan2(dy, dx) - alpha;	// angle between robot's heading and goal (in rads)

    // States:
    int state = 0;
    bool turn = false;
    bool stay_on_course = false;
    bool transition= false;

    // Internal parameters for actuator command and decisions
    double hit_range = 1.0;
    double range_threshold = 4.0;
    double turn_angle = 30*3.141/180;
    double vel_des = 10.0;
    double vel_cmd = 0.0;
    double turn_cmd = 0.0;

    // Norm distance to goal
    double dist = sqrt( pow(dx,2) + pow(dy,2) );
    // the norm distance acts as proportional gain, might be unstable if value dips below 1, saturate to 1
    dist = std::max(dist , 1.0);

    //===============================================================================
    // GOAL AND OBSTACLE DETECTION

    // Check if goal reached
    if (abs(dx) < 0.75 && abs(dy) < 0.75) {
        cout << "we win!" << endl;
        robot->set_vel(0.0);
        robot->set_turn(0.0);
        robot->done();
        return;
    }

    // Check obstacle distance and angle
    for (LaserHit hit : robot->hits) {
    	if (hit.range < range_threshold) {
    		if ( hit.angle < turn_angle && hit.angle > -turn_angle ) {
    			hit_range = hit.range;
                turn = true;
            } else if ( hit.angle < turn_angle*1.7 && hit.angle > -turn_angle*1.7 ) {
                stay_on_course = true;
            } else if ( hit.angle < turn_angle*2.2 && hit.angle > -turn_angle*2.2 ) {
                transition = true;
            }
        }
    }

    //===============================================================================
    // CONTROL LOGIC AND FINITE STATES MACHINE (sort of, very crude)

    // States: 	0) drive to goal, 1) avoid obstacle, 2) keep heading (no turns), 
    // 			3) transition change in angle between 2 and 0
    if (turn) {
    	// if osbstacle within a narrow range, turn away in the direction away from goal
    	// at inverse rate of distance to obstacle  
        vel_cmd = vel_des*0.8; 										// reduce speed while turning to avoid collison
        hit_range = std::max(hit_range, 1.0);				// turn slower when further away
        // turn angle based robot's location
        if (robot->pos_y < -0.1){
        	turn_cmd = (1/hit_range)*turn_angle*1;
        } else {
        	turn_cmd = (1/hit_range)*turn_angle*-1;
        }
        state = 1; 
    } else if (stay_on_course) {
    	// if osbstacle within a wider but outside narrow range, don't turn wheel 
    	// keep moving until obstacle is cleared
    	vel_cmd = vel_des*0.8; 	
        turn_cmd = 0;										// keep current heading	
        state = 2;
    } else if (transition) {
    	// if osbstacle within an even wider range, turn wheel halfway to where they should be
    	// this is to smoothen wheel turning 
    	vel_cmd = vel_des*0.9; 	
        turn_cmd = -0.5*theta*pow(dist,-1/2);				// set turn to halfway between state 0 and state 2
        state = 3;
    }
    else {
    	// if no osbstacle in range, turn wheel to goal w.r.t. heading
    	// rate of change is norm distace raised to power -1/2, 
    	// this promotes the wheel angle to slowly converge to goal rather than always pointing towards it 
        vel_cmd = vel_des;
        turn_cmd = -theta*pow(dist,-1/2);					// keep turning towards goal, slower when further away
        state = 0;
    }

    //===================================================================
    // SIGNAL CONDITIONING AND DEBUGGING:

    // saturate desired turn angle to prevent extreme turns and joint locks
    turn_cmd = std::min( std::max(turn_cmd, -50*3.14/180), 50*3.14/180);

    /*cout << endl;
    cout << "robot x =" << robot->pos_x << endl;
    cout << "robot y =" << robot->pos_y << endl;
    cout << "State: " << state << endl;
    cout << "commanded turn: " << turn_cmd*180/3.14 << " deg" << endl;
    cout << "heading: " << alpha*180/3.14 << " deg" << endl;
    cout << "goal angle w.r.t. heading: " << theta*180/3.14 << " deg" << endl;*/

    //===================================================================
    // SET ACTUATOR COMMANDS:

    robot->set_vel(vel_cmd);
    robot->set_turn(turn_cmd);
}

int
main(int argc, char* argv[])
{
    cout << "making robot" << endl;
    Robot robot(argc, argv, callback);
    robot.do_stuff();

    return 0;
}
