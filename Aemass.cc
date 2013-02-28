/////////////////////////////////
// File: Aemass.cc
// Desc: robotic life in a petri dish
// Created: 2011-10-17
// Author: Stephen Makonin <smakonin@makonin.com>
// License: GPL
/////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

#include "stage.hh"
using namespace Stg;

const bool verbose = false;

//REPORTING SECTION
typedef struct
{
    usec_t ts;
    int swarm_size;
    int food_amount;
    int waste_amount;
    int charge_amount;
    int stalled_amount;
} report_t;

report_t report;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
    
// ROBOT SECTION
typedef struct
{
    Model* model;
    ModelPosition* position;
    ModelRanger* ranger;
    ModelRanger* laser;
    ModelFiducial* fiducial;
    PowerPack *battery;
    
    ModelFiducial::Fiducial* closest;  
    ModelFiducial::Fiducial* charger;  

    radians_t com_bearing; //centre of mass
    radians_t wall_repulsive;
    int attraction_ticker;
    int avoid_count;
    int stall_stepper;
    int stepper_dir;
    bool found_food;
} robot_t;

//SPEED, ANGLE, DISTANCE LIMITS, ETC
const meters_t SWARM_SPEED = 0.4; // meters per second
const meters_t NON_SWARM_SPEED = 0.4;//8; // meters per second
const meters_t HYPER_SPEED = 2.0; // meters per second
const radians_t EXPAND_WGAIN = 0.3; // turn speed gain
const radians_t FLOCK_WGAIN = 0.3; // turn speed gain
const meters_t SAFE_DIST = 1.0; // meters
const meters_t NOISE_LEN = 0.36;
const meters_t MIN_FRONT_DIST = 0.7;
const meters_t AVOID_SPEED = 0.05; 
const radians_t AVOID_TURN = 0.5;
const meters_t STOP_DIST = 0.5;
const int AVOID_DURATION = 10;
  
const double BATTERY_LVL_MIN = 0.25; // min charge level, need to recharge
const int STEPS_BACKWARDS = 20;
const int LAW_OF_ATTRACTION = 16; // state changes after a repeated amount
const bool CAN_TRY_TO_ESCAPE = true; // allow food to escape joining the swarm

//ROBOT STATES 
const int ROBOT_STATE_DRONE = 1;
const int ROBOT_STATE_FOOD = 3;
const int ROBOT_STATE_ESCAPE = 4;
const int ROBOT_STATE_WASTE = 5;
const int ROBOT_STATE_CHARGE = 6;
const int CHARDER_ID = 99;

bool ObstacleAvoid(robot_t* robot)
{
    bool obstruction = false;
    bool stop = false;

    // find the closest distance to the left and right and check if
    // there's anything in front
    double minleft = 1e6;
    double minright = 1e6;

    // Get the data from the first sensor of the laser 
    const std::vector<meters_t>& scan = robot->laser->GetSensors()[0].ranges;

    uint32_t sample_count = scan.size();

    for(uint32_t i = 0; i < sample_count; i++)
    {
        if((i > (sample_count/4)) && (i < (sample_count - (sample_count/4))) && scan[i] < MIN_FRONT_DIST)
            obstruction = true;
            
        if(scan[i] < STOP_DIST)
            stop = true;

        if(i > sample_count/2)
            minleft = std::min(minleft, scan[i]);
        else      
            minright = std::min(minright, scan[i]);
    }

    if(obstruction || stop || (robot->avoid_count>0))
    {
        robot->position->SetXSpeed(stop ? 0.0 : AVOID_SPEED);      

        //once we start avoiding, select a turn direction and stick with it for a few iterations
        if(robot->avoid_count < 1)
        {
            robot->avoid_count = random() % AVOID_DURATION + AVOID_DURATION;

            if(minleft < minright)
                robot->position->SetTurnSpeed(-AVOID_TURN);
            else
                robot->position->SetTurnSpeed(+AVOID_TURN);
        }			  

        robot->avoid_count--;
        return true; // busy avoding obstacles
    }

    return false; // didn't have to avoid anything
}

void do_reporting(robot_t* robot, bool exit_on_min, bool exit_after_5_days)
{
    pthread_mutex_lock(&mutex);
    World *w = robot->model->GetWorld();
    if(report.ts != w->GetUpdateCount())
    {
        FILE *fp;
        char fname[64];
        char line[80];

        if(CAN_TRY_TO_ESCAPE)
            sprintf(fname, "Batt-%dkJ-Advis.csv", (int)(robot->battery->GetCapacity() / 1000));
        else
            sprintf(fname, "Batt-%dkJ-Coop.csv", (int)(robot->battery->GetCapacity() / 1000));
                
        sprintf(line, "%llu, %d, %d, %d, %d, %d\n", report.ts, report.swarm_size, report.food_amount, report.waste_amount, report.charge_amount, report.stalled_amount);
        
        if(w->GetUpdateCount() == 1)
            fp = fopen(fname, "w");
        else
            fp = fopen(fname, "a");
        fputs(line, fp);
        fclose(fp);
        
        if((report.swarm_size < LAW_OF_ATTRACTION && exit_on_min) || (w->GetUpdateCount() / 3600 >= 120 && exit_after_5_days))
            exit(0);
            
        report.ts = w->GetUpdateCount();
        report.swarm_size = report.food_amount = report.waste_amount = report.charge_amount = report.stalled_amount = 0;
    }
    switch(robot->model->GetFiducialReturn())
    {
        case ROBOT_STATE_DRONE:
            report.swarm_size++;
            break;
            
        case ROBOT_STATE_FOOD:
        case ROBOT_STATE_ESCAPE:
            report.food_amount++;
            break;
            
        case ROBOT_STATE_WASTE:
            report.waste_amount++;
            break;
            
        case ROBOT_STATE_CHARGE:
            report.charge_amount++;
            break;
    }
    if(robot->model->Stalled())
        report.stalled_amount++;
    pthread_mutex_unlock(&mutex);   
}

int RangerUpdate(ModelRanger* rgr, robot_t* robot)
{  	
    double sensor_vector = 0.0;          
    double forward_speed = 0.0;
    double side_speed = 0.0;	   
    double turn_speed = 0.0;
    const std::vector<ModelRanger::Sensor>& sensors = rgr->GetSensors();
    int my_fid = robot->model->GetFiducialReturn();
      
    //backout of stall
    if(robot->stall_stepper > 0)
    {
        robot->position->SetXSpeed(0.02 * robot->stepper_dir);
        robot->stall_stepper--;
        return 0;
    }

    // use front the-facing sensors only
    double dx=0, dy=0;
    for(unsigned int i=0; i < 8; i++ )
    {
        dx += sensors[i].ranges[0] * cos(sensors[i].pose.a);
        dy += sensors[i].ranges[0] * sin(sensors[i].pose.a);
    }
    if(dx != 0 && dy != 0) 
        sensor_vector = atan2(dy, dx);
    
    if(my_fid == ROBOT_STATE_CHARGE && robot->charger != NULL)
    {
        if(robot->charger->range < 0.3 + NOISE_LEN)
        {
            if(verbose) robot->battery->Print("WAS energy level: ");      
            robot->battery->SetStored(robot->battery->GetCapacity());
            if(verbose) robot->battery->Print("NOW energy level: ");                         
        }
        
        if(robot->charger->range < 0.5 + NOISE_LEN)
        {
            robot->position->SetXSpeed( -0.05 );
        }
        else
        {
            robot->position->SetXSpeed( 0.0 );
            robot->model->SetFiducialReturn(ROBOT_STATE_FOOD);
        }
    }   
    else if(my_fid == ROBOT_STATE_WASTE && robot->charger != NULL)
    {
        double a_goal = normalize(robot->charger->bearing);				  
        
        if(robot->charger->range > 0.5 + NOISE_LEN)
        {
            if(!ObstacleAvoid(robot))
            {
                robot->position->SetXSpeed(NON_SWARM_SPEED);	  					 
                robot->position->SetTurnSpeed(a_goal);
            }
        }
        else	
        {			
            robot->position->SetTurnSpeed(a_goal);
            robot->position->SetXSpeed(0.02);
    
            if(robot->charger->range < 0.3 + NOISE_LEN)
            {
                robot->position->Stop();
                robot->model->SetFiducialReturn(ROBOT_STATE_CHARGE);
            }
        }
    }
    else
    {          
        if(my_fid == ROBOT_STATE_DRONE)
        {
            turn_speed = EXPAND_WGAIN * (FLOCK_WGAIN * robot->com_bearing + sensor_vector + FLOCK_WGAIN * robot->wall_repulsive);
            forward_speed = SWARM_SPEED;  
            if(robot->found_food) // drones try to surround the food
                forward_speed = HYPER_SPEED;
        }
        else if(my_fid == ROBOT_STATE_FOOD || my_fid == ROBOT_STATE_ESCAPE)
        {
            turn_speed = EXPAND_WGAIN * (robot->com_bearing + FLOCK_WGAIN * sensor_vector);
            forward_speed = NON_SWARM_SPEED;       
        }
        else if(my_fid == ROBOT_STATE_WASTE)
        {
            turn_speed = EXPAND_WGAIN * (robot->com_bearing - FLOCK_WGAIN * sensor_vector);
            forward_speed = HYPER_SPEED;
        }
        else if(my_fid == ROBOT_STATE_CHARGE)
        {
            turn_speed = forward_speed = 0.0;
        }
        
        if(!ObstacleAvoid(robot))
        {
            if(sensors[3].ranges[0] > SAFE_DIST && sensors[4].ranges[0] > SAFE_DIST && sensors[5].ranges[0] > SAFE_DIST && sensors[6].ranges[0] > SAFE_DIST/2.0 && sensors[2].ranges[0] > SAFE_DIST && sensors[1].ranges[0] > SAFE_DIST/2.0)
            {
                // steer to match the heading of the nearest robot
                if(robot->closest)
                    turn_speed += FLOCK_WGAIN * robot->closest->geom.a;
            }
            else
            {
                forward_speed = 0.0;
            
                // front not clear. we might be stuck, so wiggle a bit
                if(fabs(turn_speed) < 0.1)
                    turn_speed = drand48();
            }

            robot->position->SetSpeed(forward_speed, side_speed, turn_speed);
        }
    }
    
    if(robot->model->Stalled())
    {
        robot->stall_stepper = STEPS_BACKWARDS;
        robot->stepper_dir *= -1;
    }
    
    do_reporting(robot, true, true);
    return 0;
}

int FiducialUpdate(ModelFiducial* fid, robot_t* robot)
{  	
    double dist = 1e6; // find the closest robot
    double cdist = 1e6; // find the closest charger
    double swarm_com_x = 0; //com = centre of mass
    double swarm_com_y = 0;
    double swarm_com_count = 0;
    double food_com_x = 0;
    double food_com_y = 0;
    double food_com_count = 0;
    double wall_com_x = 0;
    double wall_com_y = 0;
    double wall_com_count = 0;
    int my_fid = robot->model->GetFiducialReturn();
    
    robot->closest = NULL;
    robot->charger = NULL;
    robot->found_food = false;
    robot->wall_repulsive = 0.0;
    
    FOR_EACH(it, fid->GetFiducials())
    {
        ModelFiducial::Fiducial* other = &(*it);
        
        Model* their_model = it->mod;
        int their_fid = their_model->GetFiducialReturn();

        if(other->range < dist && their_fid == ROBOT_STATE_DRONE)
        {
            dist = other->range;
            robot->closest = other;
        }
        
        // I am looking for and see a charging station
        if((my_fid == ROBOT_STATE_WASTE || my_fid == ROBOT_STATE_CHARGE) && their_fid == CHARDER_ID && other->range < cdist)
        {
            robot->charger = other;
            cdist = other->range;
        }
        
        if(my_fid == ROBOT_STATE_DRONE && CAN_TRY_TO_ESCAPE)
        {
            if(their_fid == ROBOT_STATE_FOOD && other->bearing >= -2.0 && other->bearing <= 2.0)
            {
                food_com_count += (LAW_OF_ATTRACTION << 2);
                food_com_x += other->range * cos(other->bearing) * (LAW_OF_ATTRACTION << 2);
                food_com_y += other->range * sin(other->bearing) * (LAW_OF_ATTRACTION << 2);
                robot->found_food = true;
            }
        }
        
        if(their_fid == CHARDER_ID)
        {
            wall_com_count++;
            wall_com_x += other->range * cos(other->bearing);
            wall_com_y += other->range * sin(other->bearing);
        }        
        
        if(their_fid == ROBOT_STATE_DRONE)
        {
            swarm_com_count++;
            swarm_com_x += other->range * cos(other->bearing);
            swarm_com_y += other->range * sin(other->bearing);
        }        
    }
    
    // centre of mass for walls
    if(wall_com_count > 0)
        robot->wall_repulsive = atan2(-wall_com_y, -wall_com_x);

    // centre of mass for food
    if(food_com_count > 0)
    {
        
        swarm_com_count = 1;
        if(swarm_com_count > 0)
        {        
            swarm_com_x /= swarm_com_count;
            swarm_com_y /= swarm_com_count;
            swarm_com_count++;
        }

        swarm_com_x += (food_com_x / food_com_count);
        swarm_com_y += (food_com_y / food_com_count);        
    }
    
    // centre of mass for swarm
    if(swarm_com_count > 0)
    {        
        swarm_com_x /= swarm_com_count;
        swarm_com_y /= swarm_com_count;
        robot->com_bearing = atan2(swarm_com_y, swarm_com_x);
    }
    
    // process any state changes
    switch(my_fid)
    {
        case ROBOT_STATE_DRONE:
            if(swarm_com_count == 0)
            {
                if(++robot->attraction_ticker >= LAW_OF_ATTRACTION << 2)
                {
                    my_fid = ROBOT_STATE_WASTE;
                    robot->attraction_ticker = 0;
                }
            }
            else
            {
                robot->attraction_ticker = 0;
            }
            break;
                        
        case ROBOT_STATE_FOOD:
            if(swarm_com_count >= LAW_OF_ATTRACTION)
            {
                if(CAN_TRY_TO_ESCAPE)
                    my_fid = ROBOT_STATE_ESCAPE;
                else
                    my_fid = ROBOT_STATE_DRONE;
                robot->attraction_ticker = 0;
            }
            break;
            
        case ROBOT_STATE_ESCAPE:
            if(swarm_com_count >= LAW_OF_ATTRACTION)
            {
                if(++robot->attraction_ticker >= LAW_OF_ATTRACTION)
                {
                    my_fid = ROBOT_STATE_DRONE;                    
                    robot->attraction_ticker = 0;
                }
            }
            else
            {
                if(swarm_com_count == 0)
                    my_fid = ROBOT_STATE_FOOD;
                
                robot->attraction_ticker = 0;
            }
            break;            
    }
    
    if(my_fid < ROBOT_STATE_WASTE && robot->battery->ProportionRemaining() < BATTERY_LVL_MIN)
    {
        my_fid = ROBOT_STATE_WASTE;
        robot->attraction_ticker = 0;
    }
    
    // update according to state
    switch(my_fid)
    {
        case ROBOT_STATE_DRONE:
            robot->model->SetColor(Color::red);
            if(robot->found_food)
                robot->model->SetColor(Color::magenta);
            break;
            
        case ROBOT_STATE_FOOD:            
            if(swarm_com_x == 0 || swarm_com_y == 0)
                robot->com_bearing = normalize(drand48() * (2.0 * M_PI));
            else
                robot->com_bearing = atan2(swarm_com_y, swarm_com_x);
            robot->closest = NULL;            
            robot->model->SetColor(Color::green);
            break;

        case ROBOT_STATE_ESCAPE:
            robot->com_bearing = atan2(-swarm_com_y, -swarm_com_x);
            robot->closest = NULL;
            robot->model->SetColor(Color::yellow);
            break;
        
        case ROBOT_STATE_WASTE:            
        case ROBOT_STATE_CHARGE:
            robot->closest = NULL;
            robot->com_bearing = 0.0;                               
            robot->model->SetColor(Color(0xA6/255.0, 0x4B/255.0, 0.0, 1.0));
            break;
    }
    
    robot->model->SetFiducialReturn(my_fid);    
    return 0;
}

// Stage calls this when the model starts up
extern "C" int Init( Model* mod )
{
    robot_t* robot = new robot_t;
    robot->model = mod;
    robot->position = (ModelPosition*)mod;
    robot->battery = robot->model->FindPowerPack();
    robot->closest = robot->charger = NULL;
    robot->com_bearing = robot->wall_repulsive = robot->attraction_ticker = robot->stall_stepper = robot->avoid_count = 0;
    robot->stepper_dir = 1;
    
    report.ts = 0;
    report.swarm_size = report.food_amount = report.waste_amount = report.charge_amount = report.stalled_amount = 0;
    
    int jmin = (int)(robot->battery->GetCapacity() * BATTERY_LVL_MIN);
    if(robot->model->GetFiducialReturn() == ROBOT_STATE_FOOD)
        robot->battery->SetStored(robot->battery->GetCapacity());
    else
        robot->battery->SetStored((joules_t)(jmin + (rand() % (int)(robot->battery->GetCapacity() - jmin))));
    
    
    // subscribe to the ranger, which we use for navigating
    robot->ranger = (ModelRanger*)mod->GetChild( "ranger:0" );
    assert( robot->ranger );
    robot->laser = (ModelRanger*)mod->GetChild( "ranger:1" );
    assert( robot->laser );
    
    // ask Stage to call into our ranger update function
    robot->ranger->AddCallback( Model::CB_UPDATE, (model_callback_t)RangerUpdate, robot );
    
    robot->fiducial = (ModelFiducial*)mod->GetUnusedModelOfType( "fiducial" ) ;
    assert( robot->fiducial );
    robot->fiducial->AddCallback( Model::CB_UPDATE, (model_callback_t)FiducialUpdate, robot );
    
    robot->fiducial->Subscribe();
    robot->ranger->Subscribe();
    robot->position->Subscribe();
        
    return 0; //ok
}
