/* 
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Hanna Müller, Vlad Niculescu, Tommaso Polonelli, Iman Ostovar
 */

#include "ToF_process.h"
#include "commander.h"
#include "param.h"
#include "log.h"
#include "math.h"
#include "debug.h" //TODO: remove afterwards, just for debugging now

// zones and position parameters
zone_t DRONE_ZONE = {{3.0f, 3.0f}, {5.0f, 4.0f}};
zone_t CARE_ZONE = {{2.0f, 2.0f}, {5.0f, 5.0f}};
const pos_t MIDDLE_POS = {3.0f, 3.5f};

bool object_warning = false;

uint32_t counter111 = 0;
float velx111 = 0;
float dis_react = 1.4; //m
float dis_stop = 0.15; //m


uint16_t DIS_REACT = 1400; // mm
uint16_t DIS_SLOW = 700; // mm
uint16_t DIS_STOP = 400; // mm
uint16_t DIS_FEAR = 150; // mm

float command_velocity_x = 0.0f;
float command_velocity_z = 0.0f;
float command_turn = 0.0f;
float min_distance = 0.0f;
float gain_dist_x = 0.0f;
float vel_x = 0.0f;
float turn_command = 0.0f;
float z_vel = 0;//Z_VEL
float gain_dist_tof_x_function = 0.0f;
float gain_dist_us_x_function = 0.0f;
float vel_x_old_2 = 0.8; //initialize with something not too low otherwise it will take some time to speed up of no object is in front
uint16_t border_right = 0;
uint16_t border_top = 0;
uint16_t border_bottom = 0;
uint16_t border_left = 0;

#ifdef IMAGE_PROCESS_DEBUG_PRINT
#include "debug.h"
                                       { 0, 0, 0, 0, 1, 1, 1, 1} };
#endif
// C++ Program to count islands in boolean 2D matrix

// A function to check if a given cell (row, col) can be included in DFS
bool isSafe(bool M[][COL], uint8_t row, uint8_t col, bool visited[][COL])
{
    // row number is in range, column number is in range and value is 1 and not yet visited
    return (row >= 0) && (row < ROW) && (col >= 0) && (col < COL) && (M[row][col] && !visited[row][col]);
}

// A utility function to do DFS for a 2D boolean matrix. It only considers the 8 neighbours as adjacent vertices
void DFS(bool M[][COL], uint8_t row, uint8_t col, bool visited[][COL])
{
    // These arrays are used to get row and column numbers of 8 neighbours of a given cell
    const static uint8_t neighbour_num = 8;
    const static int8_t rowNbr[] = { -1, -1, -1, 0, 0, 1, 1, 1 };
    const static int8_t colNbr[] = { -1, 0, 1, -1, 1, -1, 0, 1 };

    // Mark this cell as visited
    visited[row][col] = true;

    // Recur for all connected neighbours
    for (int k = 0; k < neighbour_num; ++k)
        if (isSafe(M, row   + rowNbr[k], col + colNbr[k], visited))
            DFS(M, row + rowNbr[k], col + colNbr[k], visited);
}

// The main function that returns count of islands in a given boolean 2D matrix
uint8_t countIslands(bool M[][COL], bool objects[][ROW][COL])
{
    // Make a bool array to mark visited cells.
    // Initially all cells are unvisited
    bool visited[ROW][COL];
    bool old_visited[ROW][COL];

    memset(visited, 0, ROW*COL); //sizeof(visited)

    int count = 0;
    for (int i = 0; i < ROW; ++i)
        for (int j = 0; j < COL; ++j)
            if (M[i][j] && !visited[i][j]) {
                //save old visited status
                memcpy(old_visited,visited, sizeof(visited));

                // If a cell with value 1 is not visited yet, then new island found Visit all cells in this island.
                DFS(M, i, j, visited);

                //save new Island positions
                if (count < MAX_TARGET_NUM)
                {   
                    for (int x = 0; x<ROW; ++x)
                        for (int y = 0; y<COL; ++y)
                            objects[count][x][y] = old_visited[x][y] ^ visited[x][y];
                }

                // and increment island count
                ++count;
            }

    return count;
}


target_t Process_ToF_Image(VL53L5CX_ResultsData* p_tof_results)
{
    // decode results
    uint16_t ToF_distances[ToF_DISTANCES_LEN/2]; // 64 array (8*8 pixels)
    uint8_t ToF_targets[ToF_TARGETS_DETECTED_LEN];  //64
    uint8_t ToF_status[ToF_TARGETS_STATUS_LEN]; //64
    memcpy(ToF_distances, (uint8_t *)(&p_tof_results->distance_mm[0]), ToF_DISTANCES_LEN); //dist in mm
    memcpy(ToF_targets, (uint8_t *)(&p_tof_results->nb_target_detected[0]), ToF_TARGETS_DETECTED_LEN); //target detected in that region
    memcpy(ToF_status, (uint8_t *)(&p_tof_results->target_status[0]), ToF_TARGETS_STATUS_LEN); //detect if pixel is valid or not
    
    // found invalid pixels and binarize the 
    bool invalid_mask[ROW*COL]; //invalid pixels mask
    for (int i= 0; i<ROW*COL; ++i)
        invalid_mask[i] = (ToF_status[i] != 5 && ToF_status[i] != 9) || ToF_targets[i] != 1;
    
    
    // binarize the image
    bool binary_matrix[ROW][COL];
    uint16_t ToF_distances_matrix[ROW][COL]; 

    for (int i= 0; i<ROW; ++i)
        for (int j= 0; j<COL; ++j)
        {
            if(! invalid_mask[j+COL*i]) // check if the pixel is valid
            {
                ToF_distances_matrix[i][j] = ToF_distances[j+COL*i];
                binary_matrix[i][j] = (ToF_distances_matrix[i][j] <= MAX_DISTANCE_TO_PROCESS);
            }
            else
            {
                ToF_distances_matrix[i][j] = UINT16_MAX; //this is invalid Value;  should care about not to be used later
                binary_matrix[i][j] = false;
            }
        }


    // detect objects
    static bool objects_matrixes[MAX_TARGET_NUM][ROW][COL];
    uint8_t object_num = countIslands(binary_matrix, objects_matrixes);

    // objects feature extraction
    target_t targets[MAX_TARGET_NUM] ;
    for (int k=0; k<object_num; ++k)
    {
        uint16_t dis_min = UINT16_MAX;
        uint16_t x_sum = 0;
        uint16_t y_sum = 0;
        uint8_t ones_count = 0;
        borders_t tar_borders = {INT8_MAX, INT8_MAX, INT8_MIN, INT8_MIN}; //top left right bottom

        for (int i= 0; i<ROW; ++i)
        {
            for (int j= 0; j<COL; ++j)
            {
                if (objects_matrixes[k][i][j])
                {
                    if(ToF_distances_matrix[i][j] < dis_min)
                        dis_min = ToF_distances_matrix[i][j];

                    x_sum += i;
                    y_sum += j;
                    ones_count ++;

                    //update borders 
                    if (i < tar_borders.top) //min i
                        tar_borders.top = i;
                    if (i > tar_borders.bottom) // max i
                        tar_borders.bottom = i;
                    if (j < tar_borders.left) // min j
                        tar_borders.left = j;
                    if (j > tar_borders.right) // max j
                        tar_borders.right = j;
                }
            }
        }
        targets[k].min_distance = dis_min;
        targets[k].position.x = (float)x_sum/ones_count;
        targets[k].position.y = (float)y_sum/ones_count;
        targets[k].pixels_number = ones_count;
        targets[k].borders = tar_borders; 
    }
    
    #ifdef IMAGE_PROCESS_DEBUG_PRINT
        // DEBUG_PRINT("ToFDeck P.I. N.Targets: %d\n",object_num); 
    #endif
    return Decision_Making(targets, object_num);

}


target_t Decision_Making(target_t* targets, uint8_t object_num) //only does obstacle avoidance for the closest object detected
{  
    // find the highest priority target 
    uint16_t dis_global_min = UINT16_MAX;
    int8_t selected_target = -1;
    for (int k= 0; k<object_num; ++k)
    {
        if (targets[k].min_distance < dis_global_min && targets[k].pixels_number > MIN_PIXEL_NUMBER )
        {
            dis_global_min = targets[k].min_distance;
            selected_target = k;
        }
    }
    if (selected_target < 0) // no big target in front, set min_distance to UINT16_MAX and check in the main file for this!!
        return (target_t){{0, 0}, {0, 0, 0, 0}, UINT16_MAX, UINT16_MAX, UINT16_MAX, 0};

    return targets[selected_target]; //return the closest object detected
}


FlyCommand_t Decision_Making_ToF(target_t selected_target) { //use this if only ToF sensor connected

    command_velocity_x = 0.0f;
    command_velocity_z = 0.0f;
    command_turn = 0.0f;
    //float current_pitch = logGetFloat(logGetVarId("stateEstimate", "pitch"));    //------------------Use this for decision making-------------
    // float current_vx = logGetFloat(logGetVarId("stateEstimate", "vx"));
    //targets[selected_target].min_distance *= cos(current_pitch/180*3.14f);
    min_distance = (float)(selected_target.min_distance/1000.0f); // just for logging
    border_right = selected_target.borders.right;
    border_bottom = selected_target.borders.bottom;
    border_left = selected_target.borders.left;
    border_top = selected_target.borders.top;

    if (selected_target.min_distance <= DIS_FEAR) //If very close, fly backwards to avoid obstacle (so close the sensor only sees in front of it, so in the danger zone for sure)
    {
        command_velocity_x = VEL_FEAR;
        command_turn = TURN_NOT;
    }
    else if (selected_target.borders.top >= GROUND_BORDER &&   
        selected_target.min_distance < DIS_GROUND_MIN && 
        ( selected_target.position.x >= MIDDLE_POS.x  )
        )//check for ground lower //If we see an obstacle close to the ground which is closer than the accepted minimal distance, and in the region where the drone flies, we fly higher
    {
        //ground should be avoided
        // decision--> "/increase height"
        // target-->  '-G'
        command_velocity_x = VEL_STOP;
        command_velocity_z = VEL_UP;
    }
    else if (selected_target.borders.bottom <= CELLING_BORDER &&
        selected_target.min_distance < DIS_CEILING_MIN &&
        (selected_target.position.x < MIDDLE_POS.x  )
        ) //check for celling upper //If too close to the ceiling 
    {
        // celling should be avoided
        // decision--> "/Decrease height"
        // target-->  '-C'
        command_velocity_x = VEL_STOP;
        command_velocity_z = VEL_DOWN;
    }
    else if (selected_target.min_distance < DIS_REACT) //check for front object
    {
        // scale the distance to the object to be in between 0 and 1 (negative values are zeroed later on)
        command_velocity_x = (float)(selected_target.min_distance - DIS_STOP)/(float)(DIS_REACT - DIS_STOP); //scale the velocity to the distance you are still away from the object


        if (selected_target.borders.right >= DRONE_ZONE.top_left.y  &&
            selected_target.borders.bottom >= DRONE_ZONE.top_left.x &&
            selected_target.borders.left <= DRONE_ZONE.bottom_righ.y &&
            selected_target.borders.top <= DRONE_ZONE.bottom_righ.x ) // object is in drone zone
        {
            if (selected_target.min_distance <= DIS_STOP)
            {
                command_velocity_x = VEL_STOP;
                command_turn = TURN_MAX;
            }
            else if (selected_target.min_distance <= DIS_SLOW ) 
            {
                command_velocity_x *= VEL_SCALE_SLOW; // velocity gets slower by 30%
                command_turn = TURN_MAX;
            }
            else //If object is in drone zone but still far away
            {
                command_velocity_x = ((float)(selected_target.min_distance - DIS_SLOW)/(float)(DIS_REACT - DIS_STOP))*VEL_SCALE_MEDIUM + ((float)(DIS_SLOW - DIS_STOP)/(float)(DIS_REACT - DIS_STOP))*VEL_SCALE_SLOW;
                // make sure there are no "bumps" in velocity scaling
                // float tmp_slow_factor = (float)(DIS_REACT - targets[selected_target].min_distance)/(float)(DIS_REACT - DIS_STOP);
                // command_velocity_x -= (VEL_SCALE_MEDIUM - VEL_SCALE_SLOW)*tmp_slow_factor;
                command_turn = TURN_SLOW;
            }
        }
        else if (selected_target.borders.right >= CARE_ZONE.top_left.y  &&
                 selected_target.borders.bottom >= CARE_ZONE.top_left.x &&
                 selected_target.borders.left <= CARE_ZONE.bottom_righ.y &&
                 selected_target.borders.top <= CARE_ZONE.bottom_righ.x) // object is in care zone
        {
            if (selected_target.min_distance <= DIS_STOP)
            {
                command_velocity_x = VEL_STOP;
                command_turn = TURN_MAX;
            }
            else
            {
                command_velocity_x *= VEL_SCALE_MEDIUM;
                command_turn = TURN_SLOW;
            }
        }
        else
        {
            command_velocity_x = VEL_SCALE_MEDIUM; // the objects are all in the outer regions of the FoV, we should not reduce the speed based on the distance to them
            command_turn = TURN_NOT;
        }
        if (selected_target.position.y < MIDDLE_POS.y)
        {
            command_turn *= -1.0f;
        }

         
    }
    else
    {
        command_velocity_x = VEL_SCALE_MEDIUM;
        command_turn = TURN_NOT;
    }  

    // Check for special situations
    command_turn = Handle_Exception_Commands(command_turn);

    FlyCommand_t fly_command = {command_velocity_x, command_velocity_z, command_turn};
    return fly_command;
}


float Handle_Exception_Commands(float current_turn_command)
{
    float refine_command = current_turn_command;
    #define HISTORY_LENGTH 5
    static float previous_command[HISTORY_LENGTH] = {CommandError} ; //have access to the previous commands in the array previous_command (5 last)
    static uint8_t previous_command_index = 0;
    uint8_t left_commands = 0, right_commands=0;
    
    //handle convex situation
    for (int i =0; i< HISTORY_LENGTH; i++)
    {
        if (previous_command[i] < -TURN_MAX + EPSILON) //we turned maximum to the right
            right_commands++;
        else if (previous_command[i] > TURN_MAX - EPSILON) //we turned maximum to the left
            left_commands++; 
    }
    if ((current_turn_command < -TURN_MAX + EPSILON || current_turn_command > TURN_MAX - EPSILON) 
        && right_commands+left_commands >= (MAX_TURN_RATIO* HISTORY_LENGTH) ) //if we turned a lot right or left -> means we are stuck -> turn fast to get out
    {
            // negativ means turn to the right 
            // (we always want to turn right if we are stuck, to avoid switching between right/left and to maximize explored area)
            if (right_commands > left_commands)
            {
                refine_command = - TURN_FAST; 
            }
            else{
                refine_command = TURN_FAST;
            }
    }
    previous_command[previous_command_index++] = current_turn_command; //previous_command_index will get incremented every time the function gets executed
    previous_command_index %= HISTORY_LENGTH;
    //
    return refine_command;
}

#define HISTORY_LENGTH 10
static float previous_command[HISTORY_LENGTH] = {CommandError} ; //have access to the previous commands in the array previous_command (5 last)
static uint8_t previous_command_index = 0;

//adjust our turning command based on the HISTORY_LENGTH last turning commands
float Handle_Exception_Commands_us_ToF(float current_turn_command) 
{
    float refine_command = current_turn_command;
    
    uint8_t left_commands = 0, right_commands=0;
    
    //hanlde convex situation
    for (int i =0; i< HISTORY_LENGTH; i++)
    {
        if (previous_command[i] < -TURN_MAX + EPSILON) //we turned maximum to the right
            right_commands++;
        else if (previous_command[i] > TURN_MAX - EPSILON) //we turned maximum to the left
            left_commands++; 
    }
    if ((current_turn_command < -TURN_MAX + EPSILON || current_turn_command > TURN_MAX - EPSILON) && previous_command[(previous_command_index+HISTORY_LENGTH-1)%HISTORY_LENGTH] != TURN_180 && previous_command[(previous_command_index+HISTORY_LENGTH-1)%HISTORY_LENGTH] != -TURN_180
        && right_commands+left_commands >= (MAX_TURN_RATIO* HISTORY_LENGTH) ) 
    {//if we turned a lot right or left -> means we are stuck -> turn fast to get out, but if we already turned fast we do not want to turn again, since 180 degrees should have solved th issue
            // negativ means turn to the right 
            // (we always want to turn right if we are stuck, to avoid switching between right/left and to maximize explored area)
            if (right_commands > left_commands)
            {
                refine_command = - TURN_180; 
            }
            else{
                refine_command = TURN_180;
            }
    }
    previous_command[previous_command_index++] = current_turn_command; //previous_command_index will get incremented every time the function gets executed
    previous_command_index %= HISTORY_LENGTH;
    
    return refine_command;
}

//gain function that scales the distance to the object to a range of [0, 1]
float gain_linear_dist_function(float distance) {
    float scaled_dist;
    if (distance <= dis_stop) {
        scaled_dist = 0;
    }
    else if (distance <= dis_react) {
        scaled_dist = ((distance - dis_stop)/(dis_react-dis_stop)); //*((distance - dis_stop)/(dis_react-dis_stop)); //uncomment for square gain //scales between 0 and 1 squared
    }
    else {
        scaled_dist = 1;
    }
    return scaled_dist;
}

//give most of the weight to the most recent measurement of the ultrasound
float weighting_us_meas(float measurements[NB_LAST_US_MEAS]) {
    return (measurements[0]*0.45 + measurements[1]*0.3 + measurements[2]*0.15 + measurements[3]*0.075 + measurements[4]*0.025);
}

//have a more balanced "average"
float weighting_more_avg_meas(float measurements[NB_LAST_US_MEAS]) {
    return (measurements[0]*0.3 + measurements[1]*0.25 + measurements[2]*0.25 + measurements[3]*0.15 + measurements[4]*0.05);
}

//turning velocity based on the computed forward velocity
float turn_based_on_vel_func(float vel_x) {
    float turning = 0;
    if (vel_x >= 0.6) {
        turning = TURN_NOT;
    }
    else if (vel_x >= 0.1) {
        turning = TURN_SLOW;
    }
    else if (vel_x >= -0.08) { //-0.08 instead of 0 otherwise we get stuck in a corner eventually
        turning = TURN_MAX;
    }
    else {
        turning = TURN_NOT; //if we fly backwards we do not want to turn..
    }
    return turning;
}


FlyCommand_t Decision_Making_ToF_and_US(float ultrasound_meas_ff[NB_LAST_US_MEAS], target_t nearest_ToF_target, uint8_t most_rec_index) {
	//DEBUG_PRINT("ultrasound_meas[0]: %f\n", ultrasound_meas_ff[0]); //for debugging
	//DEBUG_PRINT("distance ToF: %d\n", nearest_ToF_target.min_distance);
	//DEBUG_PRINT("I'm working\n");
    // for (int i = 0; i<5; i++) {
	// 	DEBUG_PRINT("us_array[%d]: %f", i, ultrasound_meas_ff[i]);
	// }
    z_vel = 0;
    vel_x = 1;
    ///////////////////////////////////////////gain_dist function scaling//////////////////////////////////////////
	float meas_scale_us_1 = gain_linear_dist_function(ultrasound_meas_ff[most_rec_index]);
	float meas_scale_us_2 = gain_linear_dist_function(ultrasound_meas_ff[(most_rec_index+NB_LAST_US_MEAS-1)%NB_LAST_US_MEAS]);
	float meas_scale_us_3 = gain_linear_dist_function(ultrasound_meas_ff[(most_rec_index+NB_LAST_US_MEAS-2)%NB_LAST_US_MEAS]);
    float meas_scale_us_4 = gain_linear_dist_function(ultrasound_meas_ff[(most_rec_index+NB_LAST_US_MEAS-3)%NB_LAST_US_MEAS]);
    float meas_scale_us_5 = gain_linear_dist_function(ultrasound_meas_ff[(most_rec_index+NB_LAST_US_MEAS-4)%NB_LAST_US_MEAS]);
    float us_meas[NB_LAST_US_MEAS] = {meas_scale_us_1, meas_scale_us_2, meas_scale_us_3, meas_scale_us_4, meas_scale_us_5};

    /////////////////////////////////////////normal weighting////////////////////////////////
    gain_dist_us_x_function = weighting_us_meas(us_meas);
    /////////////////////////////////////////normal weighting////////////////////////////////

    float float_tof_dist = (float)(nearest_ToF_target.min_distance)/1000;
    gain_dist_tof_x_function = gain_linear_dist_function(float_tof_dist);
     ///////////////////////////////////////////gain_dist function scaling//////////////////////////////////////////

    ///////////////////////////////////////////scale based on ToF zone/////////////////////////////////////////////
    float tof_weight_x_gain_scale = 1;
    if (nearest_ToF_target.borders.top >= GROUND_BORDER &&   
        nearest_ToF_target.min_distance < DIS_GROUND_MIN && 
        ( nearest_ToF_target.position.x >= MIDDLE_POS.x )) //check for ground lower
	{
            vel_x = 0; //put the velocity to 0 to not get unstable
            z_vel = VEL_UP; //Z_VEL
            DEBUG_PRINT("\n1\n");
	}
    else if (nearest_ToF_target.borders.bottom <= CELLING_BORDER &&
        nearest_ToF_target.min_distance < DIS_CEILING_MIN &&
        (nearest_ToF_target.position.x < MIDDLE_POS.x  )
        ) //check for celling upper
    {
        // celling should be avoided
        // decision--> "/Decrease height"
        // target-->  '-C'
        vel_x = 0; // put the velocity to 0 to not get unstable
        z_vel = VEL_DOWN; //Z_VEL
        DEBUG_PRINT("\n2\n");
    }
    else if (nearest_ToF_target.min_distance <= DIS_REACT) {
        if (nearest_ToF_target.borders.right >= DRONE_ZONE.top_left.y  &&
                nearest_ToF_target.borders.bottom >= DRONE_ZONE.top_left.x &&
                nearest_ToF_target.borders.left <= DRONE_ZONE.bottom_righ.y &&
                nearest_ToF_target.borders.top <= DRONE_ZONE.bottom_righ.x ) // object is in drone zone
        {
            tof_weight_x_gain_scale = 0.5; //safety factor
            DEBUG_PRINT("\n3\n");
        }

        else if (nearest_ToF_target.borders.right >= CARE_ZONE.top_left.y  &&
                    nearest_ToF_target.borders.bottom >= CARE_ZONE.top_left.x &&
                    nearest_ToF_target.borders.left <= CARE_ZONE.bottom_righ.y &&
                    nearest_ToF_target.borders.top <= CARE_ZONE.bottom_righ.x) // object is in care zone
        {
            tof_weight_x_gain_scale = 0.8; //safety factor
            DEBUG_PRINT("\n4\n");
        }
    }
    ///////////////////////////////////////////scale based on ToF zone/////////////////////////////////////////////

    ///////////////////////////////////////////x velocity/////////////////////////////////////////////////////////
    if (vel_x == 1) { //means we don't go up or down

        gain_dist_tof_x_function *= tof_weight_x_gain_scale; //scale with safety factor

        float weight_x_us = 1 - weighting_more_avg_meas(us_meas); //take more of an average to better filter out potential wrong measurements
        float weight_x_tof = 1-weight_x_us;


        ///////////////////////////try with the minimum value///////////////////////
        // if (gain_dist_tof_x_function < gain_dist_us_x_function) {
        //     gain_dist_x = gain_dist_tof_x_function;
        // }
        // else {
        //     gain_dist_x = gain_dist_us_x_function;
        // }
        ///////////////////////////try with the minimum value//////////////////////

        //////////////////////////try with weights///////////////////////////////
        gain_dist_x = weight_x_tof*gain_dist_tof_x_function + weight_x_us*gain_dist_us_x_function;
        //////////////////////////try with weights//////////////////////////////

        //Now have to scale to the velocity, we want to scale it between -0.15 and 1.0
        vel_x = gain_dist_x*1.15 - 0.15; //Now between -0.15 and 1.0

        if (vel_x > vel_x_old_2 + 0.05) { //do not accelerate too fast to not get unstable
            vel_x = vel_x_old_2 + 0.05;
        }
        else if (vel_x < vel_x_old_2 - 0.25) { //do not break faster than -0.4 in velocity to not get too unstable
            vel_x = vel_x_old_2 -0.25;
        }
        vel_x_old_2 = vel_x;
    }
    ///////////////////////////////////////////x velocity/////////////////////////////////////////////////////////

    ////////////////////////////////////////////turning based on vel function//////////////////////
    turn_command = turn_based_on_vel_func(vel_x); //adapt the turning command based on the function for the x_vel
    ////////////////////////////////////////////turning based on vel function//////////////////////


    if (nearest_ToF_target.position.y < MIDDLE_POS.y) { //if the object is in the left half plane, turn right
        turn_command *= -1.0f;
    }
    turn_command = Handle_Exception_Commands_us_ToF(turn_command); //handle exceptions, like if we're stuck in a corner

    if (turn_command == TURN_180 || turn_command == -TURN_180) { //if we turn fast, we do not want to fly forward at the same time to not get unstable
        vel_x = 0;
    }

    //////////////////////////////////////////For debugging/////////////////////////////////////////
    DEBUG_PRINT("turn: %f", turn_command);
    //DEBUG_PRINT("tof_loss: %f", (tof_weight_x_vel * (float)(nearest_ToF_target.min_distance)/1000)*(tof_weight_x_vel * (float)(nearest_ToF_target.min_distance)/1000));
    //DEBUG_PRINT("us_loss: %f", ultrasound_x_vel*ultrasound_x_vel);
    DEBUG_PRINT("z_vel: %f", z_vel); //Z_VEL
    //DEBUG_PRINT("vel_x_loss: %f\n", loss_vel_x);
    DEBUG_PRINT("ToF: %f", float_tof_dist);
    //DEBUG_PRINT("tof_weight_x_gain_scale: %f", tof_weight_x_gain_scale);
    //DEBUG_PRINT("gain_dist_tof_x_function: %f", gain_dist_tof_x_function);

    //DEBUG_PRINT("weight_x_us: %f", weight_x_us);
    //DEBUG_PRINT("gain_dist_us_x_function: %f", gain_dist_us_x_function);
    DEBUG_PRINT("gain_dist_x: %f", gain_dist_x);
    DEBUG_PRINT("vel_x: %f\n", vel_x);
    //////////////////////////////////////////For debugging/////////////////////////////////////////

    //////////////////////////////////////////Calculate the whole distance covered during the flight////////////////////////////
    if (vel_x > 0) { //DIST
        velx111 += vel_x; //DIST
    } //DIST
    else { //DIST
        velx111 -= vel_x; //DIST
    } //DIST
    
    //DEBUG_PRINT("velx111: %f", velx111);
    counter111 += 1; //DIST
    //////////////////////////////////////////Calculate the whole distance covered during the flight////////////////////////////

	return (FlyCommand_t) {vel_x, z_vel, turn_command};
}


PARAM_GROUP_START(ToF_FLY_PARAMS)
PARAM_ADD(PARAM_UINT16, React_dis, &DIS_REACT)
PARAM_ADD(PARAM_UINT16, SLOW_DIS, &DIS_SLOW)
PARAM_ADD(PARAM_UINT16, Stop_dis, &DIS_STOP)
PARAM_ADD(PARAM_UINT16, Fear_dis, &DIS_FEAR)
PARAM_GROUP_STOP(ToF_FLY_PARAMS)

LOG_GROUP_START(tof_cmds)
LOG_ADD(LOG_FLOAT, vel_x, &command_velocity_x)
LOG_ADD(LOG_FLOAT, vel_z, &command_velocity_z)
LOG_ADD(LOG_FLOAT, turn, &command_turn)
LOG_ADD(LOG_FLOAT, min_dis, &min_distance)
LOG_ADD(LOG_UINT16, bor_t, &border_top)
LOG_ADD(LOG_UINT16, bor_l, &border_left)
LOG_ADD(LOG_UINT16, bor_r, &border_right)
LOG_ADD(LOG_UINT16, bor_b, &border_bottom)
LOG_GROUP_STOP(tof_cmds)

LOG_GROUP_START(ToF_US_commands)
// LOG_ADD(LOG_FLOAT, tof_w_x, &tof_weight_x_gain_scale)
LOG_ADD(LOG_FLOAT, tof_func, &gain_dist_tof_x_function)
LOG_ADD(LOG_FLOAT, us_func, &gain_dist_us_x_function)
LOG_ADD(LOG_FLOAT, z_vel, &z_vel)
LOG_ADD(LOG_FLOAT, turn, &turn_command)
LOG_ADD(LOG_FLOAT, vel_x, &vel_x)
LOG_ADD(LOG_FLOAT, g_dis_x, &gain_dist_x)
LOG_GROUP_STOP(ToF_US_commands)

PARAM_GROUP_START(SENS_FUS_Fun)
PARAM_ADD(PARAM_FLOAT, dis_reac, &dis_react)
PARAM_ADD(PARAM_FLOAT, dis_stop, &dis_stop)
PARAM_GROUP_STOP(SENS_FUS_Fun)