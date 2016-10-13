#include <ros/ros.h>
#include <std_msgs/String.h>
#include <uv_msgs/ImageBoundingBox.h>
#include <uv_msgs/PtuControlParams.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <stdio.h>
#include <random_numbers/random_numbers.h>
#include <fuzzyLogic.h>
#include "uv_pr2_head_controller_defs.h"

#define grad2Rad 0.0174533

ros::Publisher tracker_pub,controlStatus;
ros::Subscriber sub1,sub2,paramSubs;
sensor_msgs::JointState ptuControlMsg,currentPR2HeadState;
trajectory_msgs::JointTrajectory uvPR2HeadControlMsg;
control_msgs::JointTrajectoryControllerState pr2HeadState;
uv_msgs::PtuControlParams status;


/* PID parameters */
float kp=0.9,ki=0.2,kd=0.01; 

float pxl2Rad=0.001690788;

/* General parameters */

float _cameraFoV_W=62.0,_cameraFoV_H=48.6; 
int _imgWidth=640,_imgHeight=480; /* Default image width and height */
int _imgSetPointX=320,_imgSetPointY=240;/* Default image center coordinates */

double msgTime=0.0,currentTime,deltaTime; 
double timeBeforeHome=10.0,stopTime=1.0;
double timeBeforeXplor=15.0,xplorWaitingTime=3.0;
double panHomePos=0.0;
double tiltHomePos=0.0;
int goHome=1;

/* Xplor params */
double xplorPan=0.0,xplorPanSigma=0.26;
double xplorTilt=0.07,xplorTiltSigma=0.16;

int PR2_HEAD_controlMode = 0;
int PR2_HEAD_controlDataBased;
int PR2_HEAD_controlStatus;
int PR2_HEAD_xplor_on = 1;

float _MaxPanSpeed=0.8;        /* Internal Maximal Pan Speed */
float _MaxTiltSpeed=0.8;        /* Internal Maximal Tilt Speed */
float _MinPanSpeed=0.0;        /* Internal Maximal Pan Speed */
float _MinTiltSpeed=0.0;        /* Internal Maximal Tilt Speed */

float _MaxPanPos=2.7;        /* Internal Maximal Pan Position */
float _MaxTiltPos=-0.4;       /* Internal Maximal Tilt Position */
float _MinPanPos=-2.7;        /* Internal Minimal Pan Position */
float _MinTiltPos=1.3519;       /* Internal Minimal Tilt Position */

char pr2HeadDefaultID[]="";

int PR2_HEAD_error[3],PR2_HEAD_perror[3];  /* PR2_HEAD current and previous errors */
int PR2_HEAD_errorSpd[3],PR2_HEAD_perrorSpd[3]; /* PR2_HEAD current and previous errors speed */

/* Fuzzy parameters */

int isFuzzyInitialized=0;
int NoOfPanLabels=DEFAULT_NO_OF_FUZZY_LABELS;
int NoOfTiltLabels=DEFAULT_NO_OF_FUZZY_LABELS;
int NoOfPanRules=DEFAULT_NO_OF_FUZZY_LABELS;
int NoOfTiltRules=DEFAULT_NO_OF_FUZZY_LABELS;
int PR2_HEAD_FirtsIt=1;
int _widthEps=10,_heightEps=10;
double PR2_HEADFuzzyPanThres[3*DEFAULT_NO_OF_FUZZY_LABELS];
double PR2_HEADFuzzyTiltThres[3*DEFAULT_NO_OF_FUZZY_LABELS];
double PR2_HEADFuzzyPanDotThres[3*DEFAULT_NO_OF_FUZZY_LABELS];
double PR2_HEADFuzzyTiltDotThres[3*DEFAULT_NO_OF_FUZZY_LABELS];
double PR2_HEADFuzzyPanVal[DEFAULT_NO_OF_FUZZY_LABELS];
double PR2_HEADFuzzyTiltVal[DEFAULT_NO_OF_FUZZY_LABELS];
fuzzSet *FzyPan,*FzyTilt;

double  PR2_HEAD_PanVal[]={-0.50,-0.1,0,0.1,0.50};
double  PR2_HEAD_TiltVal[]={-0.50,-0.1,0,0.1,0.50};

double fuzzy[60] = {-150,-80,0,-150,-10,0,-80,80,20,10,150,0,80,150,0,
		    -110,-60,0,-110,-10,0,-60,60,20,10,110,0,60,110,0,
		    -8,-5,0,-8,-3,0,-4,4,2,3,8,0,5,8,0,
		    -8,-5,0,-8,-3,0,-4,4,2,3,8,0,5,8,0};
short RMtx[]={4,3,3,1,1,
	      3,3,3,1,1,
	      3,3,2,1,1,
	      3,3,1,1,1,
	      3,3,1,1,0};
int l[]={5,5,5};

/* end fuzzy parameters */

int PR2_HEAD_ImageDimensions(int width,int height)
{
  _imgWidth=width;  _imgHeight=height;
  _imgSetPointX=width/2; _imgSetPointY=height/2;
  return 0;
}

/*-------------------------------------------------------------------
 *  function: PR2_HEAD_calcErrorVect this function computes error of the 
 *            center of bonding Box relative to the center of the 
 *            image. Errors are asigned to a global variable called
 *            PR2_HEAD_error and precedent errors are asigned to 
 *            PR2_HEAD_perror
 *           
 *  param   : int bBoxCenterX  
 *            int bBoxCenterY
 *
 *  return  : int -> 0 if SUCCESS
 *
 *------------------------------------------------------------------*/

int PR2_HEAD_calcErrorVect(int bBoxCenterX,int bBoxCenterY)
{

  PR2_HEAD_perror[0]=PR2_HEAD_error[0];
  PR2_HEAD_perror[1]=PR2_HEAD_error[1];

  PR2_HEAD_error[0]=bBoxCenterX-_imgSetPointX;
  PR2_HEAD_error[1]=bBoxCenterY-_imgSetPointY;

  if (PR2_HEAD_FirtsIt) {
    PR2_HEAD_FirtsIt=0;
    PR2_HEAD_errorSpd[0]=0;
    PR2_HEAD_errorSpd[1]=0;
  } else {
    PR2_HEAD_errorSpd[0]=(PR2_HEAD_error[0]-PR2_HEAD_perror[0]);
    PR2_HEAD_errorSpd[1]=(PR2_HEAD_error[1]-PR2_HEAD_perror[1]);
    PR2_HEAD_perrorSpd[0]=PR2_HEAD_errorSpd[0];///PR2_HEAD_dtime;
    PR2_HEAD_perrorSpd[1]=PR2_HEAD_errorSpd[1];///PR2_HEAD_dtime;
  }

  return 0;
}

/*-------------------------------------------------------------------
 *  function: PR2_HEAD__SetFuzzyRules
 *
 *  param   : int panLabels  
 *            int tiltLabels
 *            double *fuzzySet
 *
 *  return  : int -> 0 if SUCCESS
 *
 *------------------------------------------------------------------*/
int PR2_HEAD_SetFuzzyRules(int panLabels,int tiltLabels, double *fuzzySet)
{
  int i;
  
  NoOfPanLabels=panLabels;
  NoOfTiltLabels=tiltLabels;
  NoOfPanRules=panLabels;
  NoOfTiltRules=tiltLabels;
  
  if ((panLabels!=5) || (tiltLabels!=5)){
    ROS_INFO("No valid fuzzy parameters!");
    return -1;
  }

  for(i=0;i<3*panLabels;i++){
    PR2_HEADFuzzyPanThres[i]=fuzzySet[i];
    PR2_HEADFuzzyTiltThres[i]=fuzzySet[i+3*NoOfTiltLabels];
    PR2_HEADFuzzyPanDotThres[i]=fuzzySet[i+6*NoOfPanLabels];
    PR2_HEADFuzzyTiltDotThres[i]=fuzzySet[i+9*NoOfTiltLabels];
 }
  // for(i=0;i<3*tiltLabels;i++){
  // to incorporate later
  // }
  FzyPan=createFuzzyLogSet(l,PR2_HEADFuzzyPanThres,PR2_HEADFuzzyPanDotThres,
			   PR2_HEAD_PanVal,RMtx);
  FzyTilt=createFuzzyLogSet(l,PR2_HEADFuzzyTiltThres,PR2_HEADFuzzyTiltDotThres,
			    PR2_HEAD_TiltVal,RMtx);
  return 0;
}

/*-------------------------------------------------------------------
 *  function: initialize_node
 *
 *  param   : none
 *
 *  return  : int -> 0 if SUCCESS
 *
 *------------------------------------------------------------------*/

int initialize_node()
{

  uvPR2HeadControlMsg.header.frame_id=pr2HeadDefaultID;
  // currentPR2_HEADState
  uvPR2HeadControlMsg.joint_names.resize(2);
  uvPR2HeadControlMsg.joint_names[0]="head_pan_joint";  
  uvPR2HeadControlMsg.joint_names[1]="head_tilt_joint";  
  uvPR2HeadControlMsg.points.resize(1);
  uvPR2HeadControlMsg.points[0].positions.resize(2);
  uvPR2HeadControlMsg.points[0].velocities.resize(2);
  uvPR2HeadControlMsg.points[0].positions[0]=0.0;
  uvPR2HeadControlMsg.points[0].velocities[0]=_MaxPanSpeed;
  uvPR2HeadControlMsg.points[0].positions[1]=0.0;
  uvPR2HeadControlMsg.points[0].velocities[1]=_MaxTiltSpeed;
  uvPR2HeadControlMsg.points[0].time_from_start=ros::Duration(1.7);
 

  currentPR2HeadState.header.frame_id=pr2HeadDefaultID;
  currentPR2HeadState.name.resize(2);
  currentPR2HeadState.position.resize(2);
  currentPR2HeadState.velocity.resize(2);
  currentPR2HeadState.name[0] ="pan";
  currentPR2HeadState.position[0] = 0.0;
  currentPR2HeadState.velocity[0] = _MaxPanSpeed;
  currentPR2HeadState.name[1] ="tilt";
  currentPR2HeadState.position[1] = 0.0;
  currentPR2HeadState.velocity[1] = _MaxTiltSpeed;

  status.header.frame_id=pr2HeadDefaultID;
  status.hardware_id="pr2Head";
  status.status="UV_PR2_HEAD_WAITING";
  if (PR2_HEAD_xplor_on==1) status.xplor_on=true;
  else status.xplor_on=false;
  status.panHome=panHomePos;
  status.tiltHome=tiltHomePos;
  status.time2Stop=stopTime;
  status.time2Home=timeBeforeHome;
  status.time2Xplor=timeBeforeXplor;
  status.exploringWaiting=xplorWaitingTime;

  PR2_HEAD_controlStatus = UV_PR2_HEAD_WAITING;
 
  if (PR2_HEAD_controlMode==UV_PR2_HEAD_FUZZY_IMAGE_SPEED_CONTROL) {
    PR2_HEAD_SetFuzzyRules(5,5,fuzzy);
  }

  return 0;
}

/*-------------------------------------------------------------------
 *  function: 
 *     
 *  param   : 
 *
 *  return  : int -> 0 if SUCCESS
 *
 *------------------------------------------------------------------*/
int ptuXplor()
{
  double panXplorVal,tiltXplorVal;
  random_numbers::RandomNumberGenerator gen;
 
  panXplorVal=gen.gaussian(xplorPan,xplorPanSigma);
  tiltXplorVal=gen.gaussian(xplorTilt,xplorTiltSigma);

  msgTime = ros::Time::now().toSec();
  uvPR2HeadControlMsg.header.stamp=ros::Time(msgTime);
  uvPR2HeadControlMsg.points[0].positions[0] = panXplorVal;
  uvPR2HeadControlMsg.points[0].positions[1] = tiltXplorVal;
  uvPR2HeadControlMsg.points[0].velocities[0] = _MaxPanSpeed;
  uvPR2HeadControlMsg.points[0].velocities[1] = _MaxTiltSpeed;
  uvPR2HeadControlMsg.points[0].time_from_start=ros::Duration(1.7);
  tracker_pub.publish(uvPR2HeadControlMsg);
  //  printf("Xplor %f %f \n",panXplorVal,tiltXplorVal);

  return 0;
}


/*-------------------------------------------------------------------
 *  function: pidImageBasedPosTracker
 *
 *  param   : none
 *
 *  return  : void 
 *
 *------------------------------------------------------------------*/
int pidImageBasedPosTracker()
{
  double panPID=0.0,tiltPID=0.0;
  double panOutput,tiltOutput;
  
  if (abs(PR2_HEAD_error[0])>_widthEps) 
    panPID = ((kp*PR2_HEAD_error[0]) + (ki*PR2_HEAD_perror[0]) +
	      (kd*PR2_HEAD_errorSpd[0]))*pxl2Rad;
  
  if (abs(PR2_HEAD_error[1])>_heightEps)
    tiltPID = ((kp*PR2_HEAD_error[1]) + (ki*PR2_HEAD_perror[1]) +
	       (kd*PR2_HEAD_errorSpd[1]))*pxl2Rad;
  
  panOutput = currentPR2HeadState.position[0] - panPID;
  tiltOutput = currentPR2HeadState.position[1] - tiltPID;

  msgTime = ros::Time::now().toSec();

  uvPR2HeadControlMsg.header.stamp=ros::Time(msgTime);

  uvPR2HeadControlMsg.points[0].positions[0]= panOutput;
  uvPR2HeadControlMsg.points[0].positions[1] = -tiltOutput;
  uvPR2HeadControlMsg.points[0].velocities[0] = _MaxPanSpeed;
  uvPR2HeadControlMsg.points[0].velocities[1] = -_MaxTiltSpeed;
  tracker_pub.publish(uvPR2HeadControlMsg);

  return 0;
}

/*-------------------------------------------------------------------
 *  function: fuzzyImageBasedSpeedTracker
 *
 *  param   : none
 *
 *  return  : int -> error code 0 = OK 
 *
 *------------------------------------------------------------------*/
int fuzzyImageBasedSpeedTracker()
{
  double panSpeed,tiltSpeed;

  /* PAN values */

    uvPR2HeadControlMsg.points[0].positions[0]=panHomePos;
    uvPR2HeadControlMsg.points[0].velocities[0]=_MaxPanSpeed;
    uvPR2HeadControlMsg.points[0].positions[1]=tiltHomePos;
    uvPR2HeadControlMsg.points[0].velocities[1]=_MaxTiltSpeed;
    uvPR2HeadControlMsg.points[0].time_from_start=ros::Duration(10.0);

  if (abs(PR2_HEAD_error[0])>_widthEps) {
    panSpeed=getFuzzyOutput(PR2_HEAD_error[0],PR2_HEAD_errorSpd[0],FzyPan);
    if (panSpeed>_MinPanSpeed) uvPR2HeadControlMsg.points[0].positions[0]=_MaxPanPos;
    else if (panSpeed<-_MinPanSpeed) uvPR2HeadControlMsg.points[0].positions[0]=_MinPanPos;
  }

  if ((abs(PR2_HEAD_error[0])<_widthEps) || (fabs(panSpeed)<_MinPanSpeed)) {
    panSpeed=_MinPanSpeed;
        uvPR2HeadControlMsg.points[0].positions[0]= 
	  currentPR2HeadState.position[0];
  }

  /* Tilt values */

  if (abs(PR2_HEAD_error[1])>_heightEps) {
    tiltSpeed=-getFuzzyOutput(PR2_HEAD_error[1],PR2_HEAD_errorSpd[1],FzyTilt);
    if (tiltSpeed>_MinTiltSpeed) uvPR2HeadControlMsg.points[0].positions[1]=_MinTiltPos;
    else if (tiltSpeed<-_MinTiltSpeed) uvPR2HeadControlMsg.points[0].positions[1]=_MaxTiltPos;
  } 

  if ((abs(PR2_HEAD_error[1])<_heightEps) || (fabs(tiltSpeed)<_MinTiltSpeed)) {
    tiltSpeed=_MinTiltSpeed;
        uvPR2HeadControlMsg.points[0].positions[1]= currentPR2HeadState.position[1];
  }
  
  msgTime = ros::Time::now().toSec();
  uvPR2HeadControlMsg.header.stamp=ros::Time(msgTime);
  uvPR2HeadControlMsg.points[0].velocities[0] = fabs(panSpeed);
  uvPR2HeadControlMsg.points[0].velocities[1]= fabs(tiltSpeed);

  //  printf("Pan Speed %f, tilt speed %f \n",panSpeed,tiltSpeed);

  return 0;
}
/*-------------------------------------------------------------------
 *                    Callbacks function 
 *------------------------------------------------------------------*/
/*-------------------------------------------------------------------
 *  function: 
 *
 *  param   : 
 *
 *  return  : void
 *
 *------------------------------------------------------------------*/
void modifParams(const uv_msgs::PtuControlParams::ConstPtr& newParams)
{

  /* Xplor On */
  if (newParams->xplor_on!=status.xplor_on){
    if (newParams->xplor_on==true) {
      PR2_HEAD_xplor_on = 1;
      status.xplor_on=true;
    } else { PR2_HEAD_xplor_on = -1;
      status.xplor_on=false;
      PR2_HEAD_controlStatus=UV_PR2_HEAD_STOPPED;
    }
  }

 // if (newParams->time2Stop!=status.time2Stop)
 //      status.time2Stop=newParams->time2Stop;

 // if (newParams->time2Home!=status.time2Home)
 //      status.time2Home=newParams->time2Home;

}

/*-------------------------------------------------------------------
 *  function: imageBasedTracker
 *
 *  param   : const uv_msgs::ImageBoundingBox::ConstPtr& boundingBox
 *
 *  return  : void
 *
 *------------------------------------------------------------------*/
void imageBasedTracker(const uv_msgs::ImageBoundingBox::ConstPtr& boundingBox)
{
  PR2_HEAD_controlStatus =  UV_PR2_HEAD_BEING_CONTROLED;
  PR2_HEAD_calcErrorVect(boundingBox->center.u,boundingBox->center.v);

  if (PR2_HEAD_controlMode==UV_PR2_HEAD_FUZZY_IMAGE_SPEED_CONTROL) {
    fuzzyImageBasedSpeedTracker();
    status.controller="Fuzzy";
    status.mode="Speed";
  }
  if (PR2_HEAD_controlMode==UV_PR2_HEAD_PID_IMAGE_POSITION_CONTROL) {
   pidImageBasedPosTracker();
    status.controller="PID";
    status.mode="Position";
  }

  tracker_pub.publish(uvPR2HeadControlMsg);
  //  tracker_pub.publish(ptuControlMsg);

  status.based_on="Image";
  status.status="UV_PR2_HEAD_BEING_CONTROLED";
  status.header.stamp=ros::Time(ros::Time::now().toSec());
  controlStatus.publish(status);
}

/*-------------------------------------------------------------------
 *  function: pan_tilt_states -> 
 *                        
 *
 *  param   :  none
 *
 *  return  :  int status -> 0 = OK
 *
 *------------------------------------------------------------------*/

void pan_tilt_states(const sensor_msgs::JointState::ConstPtr& pr2JointStates)
{
  currentPR2HeadState.position[0] = pr2JointStates->position[15];
  currentPR2HeadState.position[1] = pr2JointStates->position[16];
}

/*-------------------------------------------------------------------
 *  function: verifyLastTime -> verifies last time control message 
 *                              has been published
 *                        
 *
 *  param   :  none
 *
 *  return  :  int status -> 0 = OK
 *
 *------------------------------------------------------------------*/
int verifyLastTime()
{
  currentTime = ros::Time::now().toSec();
  deltaTime= currentTime-msgTime;

  // Going Home after timeBeforeHome time of inactivity
  if ((PR2_HEAD_controlStatus==UV_PR2_HEAD_STOPPED) && 
      (deltaTime>timeBeforeHome)) {
 
    uvPR2HeadControlMsg.header.stamp=ros::Time(currentTime);
    uvPR2HeadControlMsg.points[0].positions[0]=panHomePos;
    uvPR2HeadControlMsg.points[0].velocities[0]=_MaxPanSpeed;
    uvPR2HeadControlMsg.points[0].positions[1]=tiltHomePos;
    uvPR2HeadControlMsg.points[0].velocities[1]=_MaxTiltSpeed;

    tracker_pub.publish(uvPR2HeadControlMsg);
    PR2_HEAD_controlStatus= UV_PR2_HEAD_AT_HOME;

    status.status="UV_PR2_HEAD_AT_HOME";
    status.header.stamp=ros::Time(ros::Time::now().toSec());
    controlStatus.publish(status);

  }

  if ((PR2_HEAD_controlStatus==UV_PR2_HEAD_BEING_CONTROLED) && (deltaTime>stopTime)) {
    uvPR2HeadControlMsg.header.stamp=ros::Time(currentTime);
    uvPR2HeadControlMsg.points[0].positions[0]=currentPR2HeadState.position[0];
    uvPR2HeadControlMsg.points[0].velocities[0]=_MinPanSpeed;
    uvPR2HeadControlMsg.points[0].positions[1]=currentPR2HeadState.position[1];
    uvPR2HeadControlMsg.points[0].velocities[1]=_MinTiltSpeed;
   
    tracker_pub.publish(uvPR2HeadControlMsg);

    status.status="UV_PR2_HEAD_STOPPED";
    status.header.stamp=ros::Time(ros::Time::now().toSec());
    controlStatus.publish(status);
    PR2_HEAD_controlStatus=UV_PR2_HEAD_STOPPED;
  }

   if(goHome==1){
    uvPR2HeadControlMsg.header.stamp=ros::Time(currentTime);
    tracker_pub.publish(uvPR2HeadControlMsg);
    controlStatus.publish(status);
    goHome=0;
  }

  if (((PR2_HEAD_controlStatus==UV_PR2_HEAD_WAITING) || 
       (PR2_HEAD_controlStatus==UV_PR2_HEAD_AT_HOME)) && 
      (PR2_HEAD_xplor_on>0) && (deltaTime>timeBeforeXplor)){
    PR2_HEAD_controlStatus=UV_PR2_HEAD_EXPLORING;
  }

  if ((PR2_HEAD_xplor_on>0) && (PR2_HEAD_controlStatus==UV_PR2_HEAD_EXPLORING)
      && (deltaTime>xplorWaitingTime)) {
    ptuXplor();    
    status.status="UV_PR2_HEAD_EXPLORING";
    status.header.stamp=ros::Time(ros::Time::now().toSec());
    controlStatus.publish(status);
    PR2_HEAD_controlStatus=UV_PR2_HEAD_EXPLORING;
  }

  return 0;
}

/*-------------------------------------------------------------------
 * 
 *                          Main function         
 *
 *------------------------------------------------------------------*/
int main(int argc, char **argv)
{
  int option;
  ros::init(argc, argv, "uv_pr2_head_controller");

  // if (argc>1) {
  //   option=atoi(argv[1]);
  //   switch(option){
  //   case 1:
  //     PR2_HEAD_controlMode=UV_PR2_HEAD_FUZZY_IMAGE_SPEED_CONTROL;
  //     PR2_HEAD_controlDataBased=UV_PR2_HEAD_IMAGE_CONTROL; 
  //     break;
  //   case 2: 
  //     PR2_HEAD_controlMode=UV_PR2_HEAD_PID_IMAGE_POSITION_CONTROL;
  //     PR2_HEAD_controlDataBased=UV_PR2_HEAD_IMAGE_CONTROL;
  //     break;
  //   default : 
  //     PR2_HEAD_controlMode=UV_PR2_HEAD_PID_IMAGE_POSITION_CONTROL;
  //     PR2_HEAD_controlDataBased=UV_PR2_HEAD_IMAGE_CONTROL;
  //     break;
  //   }
  // } else {
    PR2_HEAD_controlMode=UV_PR2_HEAD_FUZZY_IMAGE_SPEED_CONTROL;
    PR2_HEAD_controlDataBased=UV_PR2_HEAD_IMAGE_CONTROL;
    //  }
  initialize_node();

  ros::NodeHandle n;
  
  tracker_pub = n.advertise<trajectory_msgs::JointTrajectory>("/head_traj_controller/command", 100);
  controlStatus = n.advertise<uv_msgs::PtuControlParams>
    ("/uv_pr2_head_controller/status",100);

   // if(PR2_HEAD_controlDataBased=UV_PR2_HEAD_IMAGE_CONTROL)
     sub1 = n.subscribe ("/uv_pr2_head_control/tracking/bBox",1,imageBasedTracker);
   // else
   //   sub1 = n.subscribe ("/ptu/tracking/bBox",1,point3DTracker);  

  sub2 = n.subscribe ("/joint_states",1,pan_tilt_states);
  //paramSubs = n.subscribe("/ptu_control/setParams",1,modifParams);

  /* PR2_HEAD go Home */

  while(ros::ok()){
    verifyLastTime();
    ros::spinOnce();
  }

  return 0;
}
