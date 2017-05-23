// kensei.h 2006-2008

#include <stdio.h>
#include <sys/types.h>
#include "ode/ode.h"
#include "drawstuff/drawstuff.h"

#define ACTION_NUM 1000
#define MAX_ROBOT_NUM 20

#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule  dsDrawCapsuleD
#endif

#define BODY_NUM  25    // number of links (body parts)
#define JOINT_NUM 24      // number of joints


#define STARTX            0.0    // Initial position of Robot
// #define STARTY            0.0    // Initial position of Robot
#define STARTZ            0.629  // Initial position of Robot
#define TORSO_LENGTH      0.25   // x axis
#define TORSO_WIDTH       0.27   // y axis
#define TORSO_HEIGHT      0.25   // z axis
#define TORSO_MASS        10.0  //10.0

#define NECK_LENGTH       0.05
#define NECK_WIDTH        0.05
#define NECK_HEIGHT       0.10
#define NECK_MASS         0.1 //0.1

#define HEAD_LENGTH       0.3
#define HEAD_WIDTH        0.3
#define HEAD_HEIGHT       0.3
#define HEAD_RADIUS       0.25
#define HEAD_MASS         2.5 //2.5

#define SHOULDER_RADIUS   0.1

#define UPPER_ARM_LENGTH  0.04
#define UPPER_ARM_WIDTH   0.04
#define UPPER_ARM_HEIGHT  0.10
#define UPPER_ARM_MASS    0.5 //0.5

#define ELBOW_RADIUS      0.1

#define FORE_ARM_LENGTH   0.05
#define FORE_ARM_WIDTH    0.05
#define FORE_ARM_HEIGHT   0.10
#define FORE_ARM_MASS     0.5 //0.5

#define HAND_LENGTH       0.1
#define HAND_WIDTH        0.1
#define HAND_HEIGHT       0.1
#define HAND_RADIUS       0.075
#define HAND_MASS         0.2 //0.2

#define HIP_RADIUS        0.1
#define HIP_JT_WIDTH      0.17  // Distance between Hip joints

#define THIGH_LENGTH      0.06
#define THIGH_WIDTH       0.06
#define THIGH_HEIGHT      0.25
#define THIGH_MASS        10.0 //5.0

#define KNEE_RADIUS       0.1

#define CALF_LENGTH       0.07 //もとは0.07
#define CALF_WIDTH        0.07 //もとは0.07
#define CALF_HEIGHT       0.2
#define CALF_MASS         4.0 //2.0

#define ANKLE_RADIUS      0.1

#define FOOT_LENGTH       0.35 //もとは0.25
#define FOOT_WIDTH        0.18 //もとは0.15
#define FOOT_HEIGHT       0.05 //もとは0.05
#define FOOT_MASS         2.0  //0.5

#define JOINT_SIZE        0.0001
#define JOINT_MASS        0.0001
#define FMAX              200.0
#define FUDGE_FACTOR      0.1

enum parts_num  {
  TORSO,                  //  0
  NECK,                   //  1
  HEAD,                   //  2
  RIGHT_UPPER_ARM,        //  3
  RIGHT_UPPER_ARM_DUMMY1, //  4
  RIGHT_UPPER_ARM_DUMMY2, //  5
  RIGHT_FORE_ARM,         //  6
  RIGHT_HAND,             //  7
  LEFT_UPPER_ARM,         //  8
  LEFT_UPPER_ARM_DUMMY1,  //  9
  LEFT_UPPER_ARM_DUMMY2,  // 10
  LEFT_FORE_ARM,          // 11
  LEFT_HAND,              // 12
  RIGHT_THIGH_DUMMY1,     // 13
  RIGHT_THIGH_DUMMY2,     // 14
  RIGHT_THIGH,            // 15
  RIGHT_CALF,             // 16
  RIGHT_FOOT_DUMMY,       // 17
  RIGHT_FOOT,             // 18
  LEFT_THIGH_DUMMY1,      // 19
  LEFT_THIGH_DUMMY2,      // 20
  LEFT_THIGH,             // 21
  LEFT_CALF,              // 22
  LEFT_FOOT_DUMMY,        // 23
  LEFT_FOOT,              // 24
};

enum joints_num  {
  NECK_PITCH,           //  0
  HEAD_PITCH,           //  1
  RIGHT_SHOULDER_YAW,   //  2
  RIGHT_SHOULDER_ROLL,  //  3
  RIGHT_SHOULDER_PITCH, //  4
  RIGHT_ELBOW_PITCH,    //  5
  RIGHT_WRIST_PITCH,    //  6
  LEFT_SHOULDER_YAW,    //  7
  LEFT_SHOULDER_ROLL,   //  8
  LEFT_SHOULDER_PITCH,  //  9
  LEFT_ELBOW_PITCH,     // 10
  LEFT_WRIST_PITCH,     // 11
  RIGHT_HIP_YAW,        // 12
  RIGHT_HIP_ROLL,       // 13
  RIGHT_HIP_PITCH,      // 14
  RIGHT_KNEE_PITCH,     // 15
  RIGHT_FOOT_PITCH,     // 16
  RIGHT_FOOT_ROLL,      // 17
  LEFT_HIP_YAW,         // 18
  LEFT_HIP_ROLL,        // 19
  LEFT_HIP_PITCH,       // 20
  LEFT_KNEE_PITCH,      // 21
  LEFT_FOOT_PITCH,      // 22
  LEFT_FOOT_ROLL,       // 23
};

enum joints_type
{
  BALL,
  HINGE,
  SLIDER,
  HINGE2,
};

typedef struct {
  dBodyID id;
  dGeomID gid;
  dReal   px,py,pz;  // position (center of gravity) x,y,z
  dReal   lx,ly,lz;  // length x axis, y axis, z axis
  dReal   r;         // radius
  dReal   m;         // weight
} MyLink;

typedef struct {
  dJointID id;
  int   type;                   // 0:ball 1:hinge 2:slider 3:hinge2
  int   link[2];                // Two links should be attached
  dReal px, py, pz;             // anchor position x,y,z
  dReal axis_x, axis_y, axis_z; // rotation axis
  dReal lo_stop, hi_stop;       // low stop, high stop
  dReal fmax;                   // max force
  dReal fudge_factor;           // fudge factor
  dReal bounce;                 // boucyness of the stop
} MyJoint;

static MyLink   rlink[MAX_ROBOT_NUM][BODY_NUM];
static MyJoint  rjoint[MAX_ROBOT_NUM][JOINT_NUM];
static dWorldID world;
static dSpaceID space;
static dGeomID  ground;
static dJointGroupID contactgroup;
static dsFunctions fn;

static void start();
static void command(int cmd);
static void nearCallback(void *, dGeomID, dGeomID);
static void simLoop(int pause);
static void drawRobot();
static void makeRobot(int i);
static void readLinkParam();
static void readJointParam();
static void control(int jointID, dReal target);
static void controlMotor();

// initial position of each joint
static dReal
  right_shoulder_roll_vec[MAX_ROBOT_NUM]  = {0.0,0.0,0.0},
  right_shoulder_yaw_vec[MAX_ROBOT_NUM]   = {0.0,0.0,0.0},
  right_shoulder_pitch_vec[MAX_ROBOT_NUM] = {0.0,0.0,0.0},
  right_elbow_pitch_vec[MAX_ROBOT_NUM]    = {0.0,0.0,0.0},
  right_hip_roll_vec[MAX_ROBOT_NUM]       = {0.0,0.0,0.0},
  right_hip_yaw_vec[MAX_ROBOT_NUM]        = {0.0,0.0,0.0},
  right_hip_pitch_vec[MAX_ROBOT_NUM]      = {0.0,0.0,0.0},
  right_knee_pitch_vec[MAX_ROBOT_NUM]     = {0.0,0.0,0.0},
  right_foot_pitch_vec[MAX_ROBOT_NUM]     = {0.0,0.0,0.0},
  right_foot_roll_vec[MAX_ROBOT_NUM]      = {0.0,0.0,0.0},
  left_shoulder_roll_vec[MAX_ROBOT_NUM]   = {0.0,0.0,0.0},
  left_shoulder_yaw_vec[MAX_ROBOT_NUM]    = {0.0,0.0,0.0},
  left_shoulder_pitch_vec[MAX_ROBOT_NUM]  = {0.0,0.0,0.0},
  left_elbow_pitch_vec[MAX_ROBOT_NUM]     = {0.0,0.0,0.0},
  left_hip_roll_vec[MAX_ROBOT_NUM]        = {0.0,0.0,0.0},
  left_hip_yaw_vec[MAX_ROBOT_NUM]         = {0.0,0.0,0.0},
  left_hip_pitch_vec[MAX_ROBOT_NUM]       = {0.0,0.0,0.0},
  left_knee_pitch_vec[MAX_ROBOT_NUM]      = {0.0,0.0,0.0},
  left_foot_pitch_vec[MAX_ROBOT_NUM]      = {0.0,0.0,0.0},
  left_foot_roll_vec[MAX_ROBOT_NUM]       = {0.0,0.0,0.0};


static dReal
  right_shoulder_roll  = 0.0,
  right_shoulder_yaw   = 0.0,
  right_shoulder_pitch = 0.0,
  right_elbow_pitch    = 0.0,
  right_hip_roll       = 0.0,
  right_hip_yaw        = 0.0,
  right_hip_pitch      = 0.0,
  right_knee_pitch     = 0.0,
  right_foot_pitch     = 0.0,
  right_foot_roll      = 0.0,
  left_shoulder_roll   = 0.0,
  left_shoulder_yaw    = 0.0,
  left_shoulder_pitch  = 0.0,
  left_elbow_pitch     = 0.0,
  left_hip_roll        = 0.0,
  left_hip_yaw         = 0.0,
  left_hip_pitch       = 0.0,
  left_knee_pitch      = 0.0,
  left_foot_pitch      = 0.0,
  left_foot_roll       = 0.0;
