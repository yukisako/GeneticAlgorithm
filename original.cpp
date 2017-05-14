// 簡単！実践！ロボットシミュレーション
// Open Dynamics Engineによるロボットプログラミング
// 出村公成著, 森北出版 (2007) http://demura.net/
// このプログラムは上本のサンプルプログラムです．
// ヒューマノイドモデル：けんせいちゃん

// kensei.cpp copyright by Kosei Demura (2007-2008)
// This program is a sample program from the book as follows,
// Robot Simulation - Robot programming with Open Dynamics Engine - (in Japanese)
// by Kosei Demura, ISBN:978-4627846913, Morikita Publishing Co. Ltd, Tokyo 2007.
// This humanoid model, Kensei-Chan, was named after my son.

// Change Log
// 2008-10-26: makeRobot()の修正。質量パラメータの計算をボディの形状と一致させた。
//             足の位置が脛に大して前にあり転倒しやすかったので後方に変更。
// 2008-10-23: nearCallback()の修正。摩擦モデルをクーロンモデルに変更　dContactApprox1
//             けんせいちゃんの初期位置STARTX, STARTY　(kensei.h) できるよう変更
// 2008-09-02: add dInitODE() and dCloseODE() for ODE-0.10.0

#include "kensei.h"

#ifdef MSVC
#pragma warning(disable:4244 4305) // for VC++, no precision loss complaints
#endif

double STARTY = 0.0;


int step = 0;
static int sw = 0;

static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
    const int contact_no = 10;

    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
    if (b1 && b2 && dAreConnected(b1,b2)) return;

    if ((b1 == rlink[LEFT_UPPER_ARM_DUMMY1].id) ||  (b1 == rlink[LEFT_UPPER_ARM_DUMMY2].id)  || (b1 == rlink[LEFT_THIGH_DUMMY1].id) || (b1 == rlink[LEFT_THIGH_DUMMY2].id)
       || (b1 == rlink[LEFT_THIGH_DUMMY1].id)     || (b1 == rlink[LEFT_THIGH_DUMMY2].id)     || (b1 == rlink[LEFT_FOOT_DUMMY].id)) return;
    if ((b2 == rlink[RIGHT_UPPER_ARM_DUMMY1].id) || (b2 == rlink[RIGHT_UPPER_ARM_DUMMY2].id) || (b2 == rlink[RIGHT_THIGH_DUMMY1].id) || (b2 == rlink[RIGHT_THIGH_DUMMY2].id)
       || (b2 == rlink[RIGHT_THIGH_DUMMY1].id)     || (b2 == rlink[RIGHT_THIGH_DUMMY2].id)   || (b2 == rlink[RIGHT_FOOT_DUMMY].id)) return;
    if (((b1 == rlink[LEFT_UPPER_ARM].id) && (b2 == rlink[TORSO].id)) || ((b2 == rlink[LEFT_UPPER_ARM].id) && (b1 == rlink[TORSO].id)))      return;
    if (((b1 == rlink[RIGHT_UPPER_ARM].id)&& (b2 == rlink[TORSO].id)) || ((b2 == rlink[RIGHT_UPPER_ARM].id)  && (b1 == rlink[TORSO].id)))    return;
    if (((b1 == rlink[LEFT_THIGH].id)     && (b2 == rlink[TORSO].id)) || ((b2 == rlink[LEFT_THIGH].id)     && (b1 == rlink[TORSO].id)))      return;
    if (((b1 == rlink[RIGHT_THIGH].id)    && (b2 == rlink[TORSO].id)) || ((b2 == rlink[RIGHT_THIGH].id)    && (b1 == rlink[TORSO].id)))      return;
    if (((b1 == rlink[LEFT_CALF].id)      && (b2 == rlink[LEFT_FOOT].id)) || ((b2 == rlink[LEFT_CALF].id)  && (b1 == rlink[LEFT_CALF].id)))  return;
    if (((b1 == rlink[RIGHT_CALF].id)     && (b2 == rlink[RIGHT_FOOT].id))|| ((b2 == rlink[RIGHT_CALF].id) && (b1 == rlink[RIGHT_FOOT].id))) return;

    dContact contact[contact_no];
    int numc = dCollide(o1, o2, contact_no, &contact[0].geom, sizeof(dContact));

    for (int i=0; i<numc; i++)
    {
        contact[i].surface.mode       = dContactApprox1 | dContactBounce;
        contact[i].surface.mode       = dContactBounce;
        contact[i].surface.mu         = dInfinity;
        contact[i].surface.bounce     = 0.0;      // bouncing the objects
        contact[i].surface.bounce_vel = 0.0;      // bouncing velocity

        dJointID c = dJointCreateContact(world, contactgroup, contact+i);
        dJointAttach(c, b1, b2);
    }
}

static void start()
{
    static float xyz[3] = {  2.35f, -1.05f, 0.5f};
    static float hpr[3] = {145.0f,   3.0f, 0.0f};
    dsSetViewpoint(xyz,hpr);
    dsSetSphereQuality(3);
}

static void restart(){
    sw = 0;
    // step = 0;
    // dJointGroupDestroy(contactgroup);
    // contactgroup = dJointGroupCreate(0);
    // makeRobot();
    // simLoop(0);
    dJointGroupDestroy(contactgroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    fn.version = DS_VERSION;
    fn.start   = &start;
    fn.step    = &simLoop;
    fn.command = &command;
    //fn.path_to_textures = "../../drawstuff/textures";
    fn.path_to_textures = ".";

    dInitODE();
    world  = dWorldCreate();
    space  = dHashSpaceCreate(0);
    ground = dCreatePlane(space, 0, 0, 1, 0);
    contactgroup = dJointGroupCreate(0);

    dWorldSetGravity(world, 0, 0, -9.8);
    dWorldSetERP(world, 0.9);
    dWorldSetCFM(world, 1e-4);

    makeRobot(1);

    // dsSimulationLoop(argc_tmp, *argv_tmp, 800, 600, &fn);
}


static void command(int cmd)
{
    float xyz[3],hpr[3];
    switch (cmd)
    {
    case 'x':
        exit(0);
        break;
    case 's':
        dsGetViewpoint(xyz, hpr);
        while (1)
        {
            printf("xyz=%f %f %f ",xyz[0],xyz[1],xyz[2]);
            printf("hpr=%f %f %f \n",hpr[0],hpr[1],hpr[2]);
        }
        break;
    case 'r': restart();
        break;
    }
}


static void control(int num, dReal target)
{
    dReal kp = 9.0, kd = 0.1, u, diff;

    diff = target * M_PI/180.0 - dJointGetHingeAngle(rjoint[num].id);
    u    = kp * diff - kd * dJointGetHingeAngleRate(rjoint[num].id);

    dJointSetHingeParam(rjoint[num].id, dParamVel, u);
    dJointSetHingeParam(rjoint[num].id, dParamFMax, rjoint[num].fmax);
}

static void controlMotor()
{
    control(RIGHT_SHOULDER_YAW,   right_shoulder_yaw);
    control(RIGHT_SHOULDER_ROLL,  right_shoulder_roll);
    control(RIGHT_SHOULDER_PITCH, right_shoulder_pitch);
    control(RIGHT_ELBOW_PITCH,    right_elbow_pitch);
    control(LEFT_SHOULDER_YAW,    left_shoulder_yaw);
    control(LEFT_SHOULDER_ROLL,   left_shoulder_roll);
    control(LEFT_SHOULDER_PITCH,  left_shoulder_pitch);
    control(LEFT_ELBOW_PITCH,     left_elbow_pitch);
    control(RIGHT_HIP_YAW,        right_hip_yaw);
    control(RIGHT_HIP_ROLL,       right_hip_roll);
    control(RIGHT_HIP_PITCH,      right_hip_pitch);
    control(RIGHT_KNEE_PITCH,     right_knee_pitch);
    control(RIGHT_FOOT_PITCH,     right_foot_pitch);
    control(RIGHT_FOOT_ROLL,      right_foot_roll);
    control(LEFT_HIP_YAW,         left_hip_yaw);
    control(LEFT_HIP_ROLL,        left_hip_roll);
    control(LEFT_HIP_PITCH,       left_hip_pitch);
    control(LEFT_KNEE_PITCH,      left_knee_pitch);
    control(LEFT_FOOT_PITCH,      left_foot_pitch);
    control(LEFT_FOOT_ROLL,       left_foot_roll);

    /*
    printf("R Shoulder Yaw  :%4.1f\n",dJointGetHingeAngle(rjoint[RIGHT_SHOULDER_YAW].id)  / M_PI * 180);
    printf("R Shoulder Roll :%4.1f\n",dJointGetHingeAngle(rjoint[RIGHT_SHOULDER_ROLL].id) / M_PI * 180);
    printf("R Shoulder Pitch:%4.1f\n",dJointGetHingeAngle(rjoint[RIGHT_SHOULDER_PITCH].id)/ M_PI * 180);
    printf("R Elbow Pitch   :%4.1f\n",dJointGetHingeAngle(rjoint[RIGHT_ELBOW_PITCH].id)   / M_PI * 180);
    printf("L Shoulder Yaw  :%4.1f\n",dJointGetHingeAngle(rjoint[LEFT_SHOULDER_YAW].id)   / M_PI * 180);
    printf("L Shoulder Roll :%4.1f\n",dJointGetHingeAngle(rjoint[LEFT_SHOULDER_ROLL].id)  / M_PI * 180);
    printf("L Shoulder Pitch:%4.1f\n",dJointGetHingeAngle(rjoint[LEFT_SHOULDER_PITCH].id) / M_PI * 180);
    printf("L Elbow Pitch   :%4.1f\n",dJointGetHingeAngle(rjoint[LEFT_ELBOW_PITCH].id)    / M_PI * 180);
    printf("R Hip Yaw   :%4.1f\n",dJointGetHingeAngle(rjoint[RIGHT_HIP_YAW].id)    / M_PI * 180);
    printf("R Hip Roll  :%4.1f\n",dJointGetHingeAngle(rjoint[RIGHT_HIP_ROLL].id)   / M_PI * 180);
    printf("R Hip Pitch :%4.1f\n",dJointGetHingeAngle(rjoint[RIGHT_HIP_PITCH].id)  / M_PI * 180);
    printf("R Knee Pitch:%4.1f\n",dJointGetHingeAngle(rjoint[RIGHT_KNEE_PITCH].id) / M_PI * 180);
    printf("R Foot Pitch:%4.1f\n",dJointGetHingeAngle(rjoint[RIGHT_FOOT_PITCH].id) / M_PI * 180);
    printf("R Foot Roll :%4.1f\n",dJointGetHingeAngle(rjoint[RIGHT_FOOT_ROLL].id)  / M_PI * 180);
    printf("L Hip Yaw   :%4.1f\n",dJointGetHingeAngle(rjoint[LEFT_HIP_YAW].id)     / M_PI * 180);
    printf("L Hip Roll  :%4.1f\n",dJointGetHingeAngle(rjoint[LEFT_HIP_ROLL].id)    / M_PI * 180);
    printf("L Hip Pitch :%4.1f\n",dJointGetHingeAngle(rjoint[LEFT_HIP_PITCH].id)   / M_PI * 180);
    printf("L Knee Pitch:%4.1f\n",dJointGetHingeAngle(rjoint[LEFT_KNEE_PITCH].id)  / M_PI * 180);
    printf("L Foot Pitch:%4.1f\n",dJointGetHingeAngle(rjoint[LEFT_FOOT_PITCH].id)  / M_PI * 180);
    printf("L Foot Roll :%4.1f\n",dJointGetHingeAngle(rjoint[LEFT_FOOT_ROLL].id)   / M_PI * 180);
    */
    //printf("L Foot Pitch:%4.1f\n",dJointGetHingeAngle(rjoint[LEFT_FOOT_PITCH].id)  / M_PI * 180);



}

static void drawRobot()
{
    dReal myradius, myradius2,sides[3];

    for (int i=0; i< BODY_NUM; i++)
    {
        sides[0] = rlink[i].lx;
        sides[1] = rlink[i].ly;
        sides[2] = rlink[i].lz;
        switch (i)
        {
        case HEAD:
            dsSetColor(1.3,0.6, 0.6);
            myradius2 = 0.8 * rlink[i].r;
            dsDrawSphere(dBodyGetPosition(rlink[i].id),dBodyGetRotation(rlink[i].id),(float)myradius2);
            dsSetColor(1.3,1.3,1.3);
            myradius = rlink[i].r;
            dsDrawSphere(dBodyGetPosition(rlink[i].id),dBodyGetRotation(rlink[i].id),(float)myradius);
            break;
        case RIGHT_HAND:
        case LEFT_HAND:
            dsSetColor(1.3,1.3,1.3);
            myradius = rlink[i].r;
            dsDrawSphere(dBodyGetPosition(rlink[i].id),dBodyGetRotation(rlink[i].id),(float)myradius);
            break;
        case RIGHT_CALF:
        case LEFT_CALF:
            dsSetColor(0,0,1.3);
            dsDrawCapsule(dBodyGetPosition(rlink[i].id),dBodyGetRotation(rlink[i].id),
                          0.8 * sides[2],sides[0]);
                          break;
        case RIGHT_FOOT:
        case LEFT_FOOT:
            dsSetColor(1.3,1.3,1.3);
            dsDrawBox(dBodyGetPosition(rlink[i].id),dBodyGetRotation(rlink[i].id),sides);
            break;
        case TORSO:
            dsSetColor(1.3,1.3,1.3);
            dsDrawCapsule(dBodyGetPosition(rlink[i].id),dBodyGetRotation(rlink[i].id),
                          0.5 * sides[2], 0.5 * sides[0]);
            break;
        case NECK:
            dsSetColor(0,0,1.3);
            dsDrawCapsule(dBodyGetPosition(rlink[i].id),dBodyGetRotation(rlink[i].id),
                          2.0 *  sides[2],1.5 * sides[0]);
            break;
        default:
            dsSetColor(0,0,1.3);
            dsDrawCapsule(dBodyGetPosition(rlink[i].id),dBodyGetRotation(rlink[i].id),
                          0.9 * sides[2],sides[0]);
        }
    }

    dVector3 result1,result2,result3,result4,result5,result6;
    dJointGetHingeAnchor(rjoint[LEFT_KNEE_PITCH].id ,   result1);
    dJointGetHingeAnchor(rjoint[RIGHT_KNEE_PITCH].id,   result2);
    dJointGetHingeAnchor(rjoint[LEFT_ELBOW_PITCH].id,   result3);
    dJointGetHingeAnchor(rjoint[RIGHT_ELBOW_PITCH].id,  result4);
    dJointGetHingeAnchor(rjoint[LEFT_SHOULDER_ROLL].id, result5);
    dJointGetHingeAnchor(rjoint[RIGHT_SHOULDER_ROLL].id,result6);

    dsDrawSphere(result1,dBodyGetRotation(rlink[0].id),(float) 0.9 * myradius);
    dsDrawSphere(result2,dBodyGetRotation(rlink[0].id),(float) 0.9 * myradius);
    dsDrawSphere(result3,dBodyGetRotation(rlink[0].id),(float) 0.7 * myradius);
    dsDrawSphere(result4,dBodyGetRotation(rlink[0].id),(float) 0.7 * myradius);
    dsDrawSphere(result5,dBodyGetRotation(rlink[0].id),(float) 0.6 * myradius);
    dsDrawSphere(result6,dBodyGetRotation(rlink[0].id),(float) 0.6 * myradius);
}

static void simLoop(int pause)
{
    int recovery_step = 20;
    static dReal  r_knee = 0.0f, r_hip = 0.0f;
    dReal knee_angle2 = 106;

    if (!pause)
    {
        r_knee = 180.0 * dJointGetHingeAngle(rjoint[RIGHT_KNEE_PITCH].id)/ M_PI;

        if (sw == 0 && r_knee >   knee_angle2 -1)
        {
            sw = 1;
            step = 0;
        }
        if (sw == 1 && r_knee <=   25.0)
        {
            sw = 2;
            step = 0;
        }
        if (sw == 2 && r_knee >=  knee_angle2)
        {
            sw = 3;
            step = 0;
        }
        step++;
        switch (sw)
        {
        case 0:
            left_hip_pitch       = -100.0;
            right_hip_pitch      =  left_hip_pitch;
            left_knee_pitch      =  knee_angle2;
            right_knee_pitch     =  left_knee_pitch;
            left_foot_pitch      =  -75.0;
            right_foot_pitch     =  left_foot_pitch;
            left_shoulder_roll   =  120.0;
            right_shoulder_roll  = -left_shoulder_roll;
            left_shoulder_pitch  =   30.0;
            right_shoulder_pitch =  left_shoulder_pitch;
            break;
        case 1:
            left_hip_pitch       =  0.0;
            right_hip_pitch      =  left_hip_pitch;
            left_knee_pitch      =  0.0;
            right_knee_pitch     =  left_knee_pitch;
            left_foot_pitch      =  0.0;
            right_foot_pitch     =  left_foot_pitch;
            left_shoulder_roll   =  0.0;
            right_shoulder_roll  =  left_shoulder_roll;
            left_shoulder_pitch  = - 90.0;
            right_shoulder_pitch =  left_shoulder_pitch;
            break;
        case 2:
            left_hip_pitch       = -100.0;
            right_hip_pitch      =  left_hip_pitch;
            left_knee_pitch      =  120.0;
            right_knee_pitch     =  left_knee_pitch ;
            left_foot_pitch      =  -65,0; // - 65.0;
            right_foot_pitch     =  left_foot_pitch;
            left_shoulder_roll   =  180.0;
            right_shoulder_roll  = - left_shoulder_roll;
            left_shoulder_pitch  = - 90.0;
            right_shoulder_pitch = left_shoulder_pitch;
            break;
        case 3:
            printf("%d\n",step );
            if (step < recovery_step)
            {
                left_hip_pitch       = - 100 + step * 100 / recovery_step;
                right_hip_pitch      =   left_hip_pitch;
                left_knee_pitch      =   120 - step * 120 / recovery_step;
                right_knee_pitch     =   left_knee_pitch;
                left_foot_pitch      = - 65 + step *  60 / recovery_step;
                right_foot_pitch     =   left_foot_pitch;
            }
            else
            {
                left_hip_pitch       =  0.0;
                right_hip_pitch      =  left_hip_pitch;
                left_knee_pitch      =  0.0;
                right_knee_pitch     =  left_knee_pitch;
                left_foot_pitch      =  -9.0;
                right_foot_pitch     =  left_foot_pitch;
            }
            left_shoulder_roll   =   0.0;
            right_shoulder_roll  =   left_shoulder_roll;
            left_shoulder_pitch  =   0.0;
            right_shoulder_pitch =   left_shoulder_pitch;
            break;
        }
        //printf("sw=%2d right knee=%5.1f right hip=%5.1f\r",sw, r_knee, r_hip);
        //printf("sw=%2d right foot=%5.1f left foot=%5.1f\n",sw, right_foot_pitch, left_foot_pitch);
        //printf("sw=%2d right hip =%5.1f left hip  =%5.1f\n",sw, right_hip_pitch,  left_hip_pitch);

        controlMotor();

        dSpaceCollide(space,0,&nearCallback);
        dWorldStep(world,0.01);

    }
    dJointGroupEmpty(contactgroup);
    drawRobot();
}

static void readLinkParam(int y)
{
    STARTY = STARTY + y;
    rlink[TORSO].lx = TORSO_LENGTH;
    rlink[TORSO].ly = TORSO_WIDTH;
    rlink[TORSO].lz = TORSO_HEIGHT;
    rlink[TORSO].m  = TORSO_MASS;
    rlink[TORSO].px = (dReal) STARTX;
    rlink[TORSO].py = (dReal) STARTY;
    rlink[TORSO].pz = (dReal) STARTZ;

    rlink[NECK].lx = NECK_LENGTH;
    rlink[NECK].ly = NECK_WIDTH;
    rlink[NECK].lz = NECK_HEIGHT;
    rlink[NECK].m  = NECK_MASS;
    rlink[NECK].px = rlink[TORSO].px;
    rlink[NECK].py = rlink[TORSO].py;
    rlink[NECK].pz = rlink[TORSO].pz + (dReal) 0.5 * (TORSO_HEIGHT + NECK_WIDTH);

    rlink[HEAD].lx = HEAD_LENGTH;
    rlink[HEAD].ly = HEAD_WIDTH;
    rlink[HEAD].lz = HEAD_HEIGHT;
    rlink[HEAD].r  = HEAD_RADIUS;
    rlink[HEAD].m  = HEAD_MASS;
    rlink[HEAD].px = rlink[NECK].px;
    rlink[HEAD].py = rlink[NECK].py;
    rlink[HEAD].pz = rlink[NECK].pz + (dReal) 0.4 * NECK_HEIGHT + HEAD_RADIUS;

    rlink[LEFT_UPPER_ARM_DUMMY1].lx = JOINT_SIZE;
    rlink[LEFT_UPPER_ARM_DUMMY1].ly = JOINT_SIZE;
    rlink[LEFT_UPPER_ARM_DUMMY1].lz = JOINT_SIZE;
    rlink[LEFT_UPPER_ARM_DUMMY1].m  = JOINT_MASS;
    rlink[LEFT_UPPER_ARM_DUMMY1].px = rlink[TORSO].px;
    rlink[LEFT_UPPER_ARM_DUMMY1].py = rlink[TORSO].py + 0.5 * (TORSO_WIDTH + JOINT_SIZE);
    rlink[LEFT_UPPER_ARM_DUMMY1].pz = rlink[TORSO].pz + 0.5 * TORSO_HEIGHT - 0.5 * JOINT_SIZE;
    rlink[LEFT_UPPER_ARM_DUMMY2].lx = JOINT_SIZE;
    rlink[LEFT_UPPER_ARM_DUMMY2].ly = JOINT_SIZE;
    rlink[LEFT_UPPER_ARM_DUMMY2].lz = JOINT_SIZE;
    rlink[LEFT_UPPER_ARM_DUMMY2].m  = JOINT_MASS;
    rlink[LEFT_UPPER_ARM_DUMMY2].px = rlink[LEFT_UPPER_ARM_DUMMY1].px;
    rlink[LEFT_UPPER_ARM_DUMMY2].py = rlink[LEFT_UPPER_ARM_DUMMY1].py - 0.5 * JOINT_SIZE + 0.5 * HAND_RADIUS;
    rlink[LEFT_UPPER_ARM_DUMMY2].pz = rlink[LEFT_UPPER_ARM_DUMMY1].pz;
    rlink[LEFT_UPPER_ARM].lx  = UPPER_ARM_LENGTH;
    rlink[LEFT_UPPER_ARM].ly  = UPPER_ARM_WIDTH;
    rlink[LEFT_UPPER_ARM].lz  = UPPER_ARM_HEIGHT;
    rlink[LEFT_UPPER_ARM].m   = UPPER_ARM_MASS;
    rlink[LEFT_UPPER_ARM].px  = rlink[LEFT_UPPER_ARM_DUMMY2].px;
    rlink[LEFT_UPPER_ARM].py  = rlink[LEFT_UPPER_ARM_DUMMY2].py;
    rlink[LEFT_UPPER_ARM].pz  = rlink[LEFT_UPPER_ARM_DUMMY2].pz- 0.5 * (UPPER_ARM_HEIGHT + UPPER_ARM_WIDTH) ;

    rlink[LEFT_FORE_ARM].lx  = FORE_ARM_LENGTH;
    rlink[LEFT_FORE_ARM].ly  = FORE_ARM_WIDTH;
    rlink[LEFT_FORE_ARM].lz  = FORE_ARM_HEIGHT;
    rlink[LEFT_FORE_ARM].m   = FORE_ARM_MASS;
    rlink[LEFT_FORE_ARM].px  = rlink[LEFT_UPPER_ARM].px;
    rlink[LEFT_FORE_ARM].py  = rlink[LEFT_UPPER_ARM].py;
    rlink[LEFT_FORE_ARM].pz  = rlink[LEFT_UPPER_ARM].pz - 0.5 * (FORE_ARM_HEIGHT + FORE_ARM_WIDTH + UPPER_ARM_HEIGHT);

    rlink[LEFT_HAND].lx  = HAND_LENGTH;
    rlink[LEFT_HAND].ly  = HAND_WIDTH;
    rlink[LEFT_HAND].lz  = HAND_HEIGHT;
    rlink[LEFT_HAND].r   = HAND_RADIUS;
    rlink[LEFT_HAND].m   = HAND_MASS;
    rlink[LEFT_HAND].px  = rlink[LEFT_FORE_ARM].px;
    rlink[LEFT_HAND].py  = rlink[LEFT_FORE_ARM].py;
    rlink[LEFT_HAND].pz  = rlink[LEFT_FORE_ARM].pz - 0.5 * (HAND_HEIGHT + FORE_ARM_WIDTH + FORE_ARM_HEIGHT);

    rlink[RIGHT_UPPER_ARM_DUMMY1].lx =   rlink[LEFT_UPPER_ARM_DUMMY1].lx;
    rlink[RIGHT_UPPER_ARM_DUMMY1].ly =   rlink[LEFT_UPPER_ARM_DUMMY1].ly;
    rlink[RIGHT_UPPER_ARM_DUMMY1].lz =   rlink[LEFT_UPPER_ARM_DUMMY1].lz;
    rlink[RIGHT_UPPER_ARM_DUMMY1].m  =   rlink[LEFT_UPPER_ARM_DUMMY1].m;
    rlink[RIGHT_UPPER_ARM_DUMMY1].px =   rlink[LEFT_UPPER_ARM_DUMMY1].px;
    rlink[RIGHT_UPPER_ARM_DUMMY1].py = - rlink[LEFT_UPPER_ARM_DUMMY1].py + 2.0 * STARTY; // change
    rlink[RIGHT_UPPER_ARM_DUMMY1].pz =   rlink[LEFT_UPPER_ARM_DUMMY1].pz;
    rlink[RIGHT_UPPER_ARM_DUMMY2].lx =   rlink[LEFT_UPPER_ARM_DUMMY2].lx;
    rlink[RIGHT_UPPER_ARM_DUMMY2].ly =   rlink[LEFT_UPPER_ARM_DUMMY2].ly;
    rlink[RIGHT_UPPER_ARM_DUMMY2].lz =   rlink[LEFT_UPPER_ARM_DUMMY2].lz;
    rlink[RIGHT_UPPER_ARM_DUMMY2].m  =   rlink[LEFT_UPPER_ARM_DUMMY2].m;
    rlink[RIGHT_UPPER_ARM_DUMMY2].px =   rlink[LEFT_UPPER_ARM_DUMMY2].px;
    rlink[RIGHT_UPPER_ARM_DUMMY2].py = - rlink[LEFT_UPPER_ARM_DUMMY2].py + 2.0 * STARTY; // change
    rlink[RIGHT_UPPER_ARM_DUMMY2].pz =   rlink[LEFT_UPPER_ARM_DUMMY2].pz;
    rlink[RIGHT_UPPER_ARM].lx    =   UPPER_ARM_LENGTH;
    rlink[RIGHT_UPPER_ARM].ly    =   UPPER_ARM_WIDTH;
    rlink[RIGHT_UPPER_ARM].lz    =   UPPER_ARM_HEIGHT;
    rlink[RIGHT_UPPER_ARM].m     =   UPPER_ARM_MASS;
    rlink[RIGHT_UPPER_ARM].px    =   rlink[LEFT_UPPER_ARM].px;
    rlink[RIGHT_UPPER_ARM].py    = - rlink[LEFT_UPPER_ARM].py + 2.0 * STARTY; // change
    rlink[RIGHT_UPPER_ARM].pz    =   rlink[LEFT_UPPER_ARM].pz;

    rlink[RIGHT_FORE_ARM].lx =   FORE_ARM_LENGTH;
    rlink[RIGHT_FORE_ARM].ly =   FORE_ARM_WIDTH;
    rlink[RIGHT_FORE_ARM].lz =   FORE_ARM_HEIGHT;
    rlink[RIGHT_FORE_ARM].m  =   FORE_ARM_MASS;
    rlink[RIGHT_FORE_ARM].px =   rlink[LEFT_FORE_ARM].px;
    rlink[RIGHT_FORE_ARM].py = - rlink[LEFT_FORE_ARM].py + 2.0 * STARTY; // change
    rlink[RIGHT_FORE_ARM].pz =   rlink[LEFT_FORE_ARM].pz;

    rlink[RIGHT_HAND].lx  =   HAND_LENGTH;
    rlink[RIGHT_HAND].ly  =   HAND_WIDTH;
    rlink[RIGHT_HAND].lz  =   HAND_HEIGHT;
    rlink[RIGHT_HAND].r   =   HAND_RADIUS;
    rlink[RIGHT_HAND].m   =   HAND_MASS;
    rlink[RIGHT_HAND].px  =   rlink[LEFT_HAND].px;
    rlink[RIGHT_HAND].py  = - rlink[LEFT_HAND].py + 2.0* STARTY; // change
    rlink[RIGHT_HAND].pz  =   rlink[LEFT_HAND].pz;

    rlink[LEFT_THIGH_DUMMY1].lx = JOINT_SIZE;
    rlink[LEFT_THIGH_DUMMY1].ly = JOINT_SIZE;
    rlink[LEFT_THIGH_DUMMY1].lz = JOINT_SIZE;
    rlink[LEFT_THIGH_DUMMY1].m  = JOINT_MASS;
    rlink[LEFT_THIGH_DUMMY1].px = rlink[TORSO].px - 0.5 * JOINT_SIZE;
    rlink[LEFT_THIGH_DUMMY1].py = rlink[TORSO].py - 0.5 * HIP_JT_WIDTH;
    rlink[LEFT_THIGH_DUMMY1].pz = rlink[TORSO].pz - 0.5 * (TORSO_HEIGHT + JOINT_SIZE);
    rlink[LEFT_THIGH_DUMMY2].lx = JOINT_SIZE;
    rlink[LEFT_THIGH_DUMMY2].ly = JOINT_SIZE;
    rlink[LEFT_THIGH_DUMMY2].lz = JOINT_SIZE;
    rlink[LEFT_THIGH_DUMMY2].m  = JOINT_MASS;
    rlink[LEFT_THIGH_DUMMY2].px = rlink[LEFT_THIGH_DUMMY1].px + JOINT_SIZE;
    rlink[LEFT_THIGH_DUMMY2].py = rlink[LEFT_THIGH_DUMMY1].py;
    rlink[LEFT_THIGH_DUMMY2].pz = rlink[LEFT_THIGH_DUMMY1].pz;
    rlink[LEFT_THIGH].lx    = THIGH_LENGTH;
    rlink[LEFT_THIGH].ly    = THIGH_WIDTH;
    rlink[LEFT_THIGH].lz    = THIGH_HEIGHT;
    rlink[LEFT_THIGH].m     = THIGH_MASS;
    rlink[LEFT_THIGH].px    = rlink[LEFT_THIGH_DUMMY2].px - 0.5 * JOINT_SIZE;
    rlink[LEFT_THIGH].py    = rlink[LEFT_THIGH_DUMMY2].py;
    rlink[LEFT_THIGH].pz    = rlink[LEFT_THIGH_DUMMY2].pz - 0.5 * (JOINT_SIZE + THIGH_HEIGHT);

    rlink[LEFT_CALF].lx   = CALF_LENGTH;
    rlink[LEFT_CALF].ly   = CALF_WIDTH;
    rlink[LEFT_CALF].lz   = CALF_HEIGHT;
    rlink[LEFT_CALF].m    = CALF_MASS;
    rlink[LEFT_CALF].px   = rlink[LEFT_THIGH].px;
    rlink[LEFT_CALF].py   = rlink[LEFT_THIGH].py;
    rlink[LEFT_CALF].pz   = rlink[LEFT_THIGH].pz  - 0.5 * (CALF_HEIGHT + THIGH_HEIGHT);

    rlink[LEFT_FOOT_DUMMY].lx = JOINT_SIZE;
    rlink[LEFT_FOOT_DUMMY].ly = JOINT_SIZE;
    rlink[LEFT_FOOT_DUMMY].lz = JOINT_SIZE;
    rlink[LEFT_FOOT_DUMMY].m  = JOINT_MASS;
    rlink[LEFT_FOOT_DUMMY].px = rlink[LEFT_CALF].px;
    rlink[LEFT_FOOT_DUMMY].py = rlink[LEFT_CALF].py;
    rlink[LEFT_FOOT_DUMMY].pz = rlink[LEFT_CALF].pz - 0.5 * (CALF_HEIGHT + JOINT_SIZE);
    rlink[LEFT_FOOT].lx   = FOOT_LENGTH;
    rlink[LEFT_FOOT].ly   = FOOT_WIDTH;
    rlink[LEFT_FOOT].lz   = FOOT_HEIGHT;
    rlink[LEFT_FOOT].m    = FOOT_MASS;
    rlink[LEFT_FOOT].px   = rlink[LEFT_FOOT_DUMMY].px + 0.5 * FOOT_LENGTH - CALF_LENGTH;
    rlink[LEFT_FOOT].py   = rlink[LEFT_FOOT_DUMMY].py;
    rlink[LEFT_FOOT].pz   = rlink[LEFT_FOOT_DUMMY].pz - 0.5 * (FOOT_HEIGHT + JOINT_SIZE);

    rlink[RIGHT_THIGH_DUMMY1].lx =   JOINT_SIZE;
    rlink[RIGHT_THIGH_DUMMY1].ly =   JOINT_SIZE;
    rlink[RIGHT_THIGH_DUMMY1].lz =   JOINT_SIZE;
    rlink[RIGHT_THIGH_DUMMY1].m  =   JOINT_MASS;
    rlink[RIGHT_THIGH_DUMMY1].px =   rlink[LEFT_THIGH_DUMMY1].px;
    rlink[RIGHT_THIGH_DUMMY1].py = - rlink[LEFT_THIGH_DUMMY1].py + 2.0 * STARTY; // change
    rlink[RIGHT_THIGH_DUMMY1].pz =   rlink[LEFT_THIGH_DUMMY1].pz;
    rlink[RIGHT_THIGH_DUMMY2].lx =   JOINT_SIZE;
    rlink[RIGHT_THIGH_DUMMY2].ly =   JOINT_SIZE;
    rlink[RIGHT_THIGH_DUMMY2].lz =   JOINT_SIZE;
    rlink[RIGHT_THIGH_DUMMY2].m  =   JOINT_MASS;
    rlink[RIGHT_THIGH_DUMMY2].px =   rlink[LEFT_THIGH_DUMMY2].px;
    rlink[RIGHT_THIGH_DUMMY2].py = - rlink[LEFT_THIGH_DUMMY2].py + 2.0 * STARTY; // change
    rlink[RIGHT_THIGH_DUMMY2].pz =   rlink[LEFT_THIGH_DUMMY2].pz;
    rlink[RIGHT_THIGH].lx    =   THIGH_LENGTH;
    rlink[RIGHT_THIGH].ly    =   THIGH_WIDTH;
    rlink[RIGHT_THIGH].lz    =   THIGH_HEIGHT;
    rlink[RIGHT_THIGH].m     =   THIGH_MASS;
    rlink[RIGHT_THIGH].px    =   rlink[LEFT_THIGH].px;
    rlink[RIGHT_THIGH].py    = - rlink[LEFT_THIGH].py + 2.0 * STARTY; // change
    rlink[RIGHT_THIGH].pz    =   rlink[LEFT_THIGH].pz;

    rlink[RIGHT_CALF].lx   =   CALF_LENGTH;
    rlink[RIGHT_CALF].ly   =   CALF_WIDTH;
    rlink[RIGHT_CALF].lz   =   rlink[LEFT_CALF].lz;
    rlink[RIGHT_CALF].m    =   CALF_MASS;
    rlink[RIGHT_CALF].px   =   rlink[LEFT_CALF].px;
    rlink[RIGHT_CALF].py   = - rlink[LEFT_CALF].py + 2.0 * STARTY;
    rlink[RIGHT_CALF].pz   =   rlink[LEFT_CALF].pz;

    rlink[RIGHT_FOOT_DUMMY].lx =   rlink[LEFT_FOOT_DUMMY].lx;
    rlink[RIGHT_FOOT_DUMMY].ly =   rlink[LEFT_FOOT_DUMMY].ly;
    rlink[RIGHT_FOOT_DUMMY].lz =   rlink[LEFT_FOOT_DUMMY].lz;
    rlink[RIGHT_FOOT_DUMMY].m  =   rlink[LEFT_FOOT_DUMMY].m;
    rlink[RIGHT_FOOT_DUMMY].px =   rlink[LEFT_FOOT_DUMMY].px;
    rlink[RIGHT_FOOT_DUMMY].py = - rlink[LEFT_FOOT_DUMMY].py + 2.0 * STARTY; // change
    rlink[RIGHT_FOOT_DUMMY].pz =   rlink[LEFT_FOOT_DUMMY].pz;
    rlink[RIGHT_FOOT].lx   =   rlink[LEFT_FOOT].lx;
    rlink[RIGHT_FOOT].ly   =   rlink[LEFT_FOOT].ly;
    rlink[RIGHT_FOOT].lz   =   rlink[LEFT_FOOT].lz;
    rlink[RIGHT_FOOT].m    =   rlink[LEFT_FOOT].m;
    rlink[RIGHT_FOOT].px   =   rlink[LEFT_FOOT].px;
    rlink[RIGHT_FOOT].py   = - rlink[LEFT_FOOT].py + 2.0 * STARTY; // change
    rlink[RIGHT_FOOT].pz   =   rlink[LEFT_FOOT].pz;
}

void readJointParam()
{
    // neck joint (y axis)
    rjoint[NECK_PITCH].type = HINGE;
    rjoint[NECK_PITCH].px   = rlink[NECK].px;
    rjoint[NECK_PITCH].py   = rlink[NECK].py;
    rjoint[NECK_PITCH].pz   = rlink[NECK].pz - 0.5 * NECK_HEIGHT;
    rjoint[NECK_PITCH].axis_x  = 0;
    rjoint[NECK_PITCH].axis_y  = 1;
    rjoint[NECK_PITCH].axis_z  = 0;
    rjoint[NECK_PITCH].lo_stop  = 0;
    rjoint[NECK_PITCH].hi_stop  = 1;
    rjoint[NECK_PITCH].fmax     = FMAX;
    rjoint[NECK_PITCH].link[0]  = TORSO;
    rjoint[NECK_PITCH].link[1]  = NECK;

    // head joint (y axis)
    rjoint[HEAD_PITCH].type = HINGE;
    rjoint[HEAD_PITCH].px   = rlink[HEAD].px;
    rjoint[HEAD_PITCH].py   = rlink[HEAD].py;
    rjoint[HEAD_PITCH].pz   = rlink[HEAD].pz - HEAD_RADIUS;
    rjoint[HEAD_PITCH].axis_x  = 0;
    rjoint[HEAD_PITCH].axis_y  = 1;
    rjoint[HEAD_PITCH].axis_z  = 0;
    rjoint[HEAD_PITCH].lo_stop  = 0.0;
    rjoint[HEAD_PITCH].hi_stop  = 0.0;
    rjoint[HEAD_PITCH].fmax     = FMAX;
    rjoint[HEAD_PITCH].link[0]  = NECK;
    rjoint[HEAD_PITCH].link[1]  = HEAD;

    // left wrist joint (y axis)
    rjoint[LEFT_WRIST_PITCH].type = HINGE;
    rjoint[LEFT_WRIST_PITCH].px   = rlink[LEFT_FORE_ARM].px;
    rjoint[LEFT_WRIST_PITCH].py   = rlink[LEFT_FORE_ARM].py;
    rjoint[LEFT_WRIST_PITCH].pz   = rlink[LEFT_FORE_ARM].pz - 0.5 * FORE_ARM_HEIGHT;
    rjoint[LEFT_WRIST_PITCH].axis_x  = 0;
    rjoint[LEFT_WRIST_PITCH].axis_y  = 1;
    rjoint[LEFT_WRIST_PITCH].axis_z  = 0;
    rjoint[LEFT_WRIST_PITCH].lo_stop  = 0.0;
    rjoint[LEFT_WRIST_PITCH].hi_stop  = 0.0;
    rjoint[LEFT_WRIST_PITCH].fmax     = FMAX;
    rjoint[LEFT_WRIST_PITCH].link[0]  = LEFT_FORE_ARM;
    rjoint[LEFT_WRIST_PITCH].link[1]  = LEFT_HAND;

    // left wrist joint (y axis)
    rjoint[RIGHT_WRIST_PITCH].type =   rjoint[LEFT_WRIST_PITCH].type;
    rjoint[RIGHT_WRIST_PITCH].px   =   rjoint[LEFT_WRIST_PITCH].px;
    rjoint[RIGHT_WRIST_PITCH].py   = - rjoint[LEFT_WRIST_PITCH].py;
    rjoint[RIGHT_WRIST_PITCH].pz   =   rjoint[LEFT_WRIST_PITCH].pz;
    rjoint[RIGHT_WRIST_PITCH].axis_x  =   rjoint[LEFT_WRIST_PITCH].axis_x;
    rjoint[RIGHT_WRIST_PITCH].axis_y  =   rjoint[LEFT_WRIST_PITCH].axis_y;
    rjoint[RIGHT_WRIST_PITCH].axis_z  =   rjoint[LEFT_WRIST_PITCH].axis_z;
    rjoint[RIGHT_WRIST_PITCH].lo_stop  =   0.0;
    rjoint[RIGHT_WRIST_PITCH].hi_stop  =   0.0;
    rjoint[RIGHT_WRIST_PITCH].fmax     =   FMAX;
    rjoint[RIGHT_WRIST_PITCH].link[0]  =   RIGHT_FORE_ARM;
    rjoint[RIGHT_WRIST_PITCH].link[1]  =   RIGHT_HAND;

    // left shoulder (YAW: z, ROLL: x, PITCH: y)
    rjoint[LEFT_SHOULDER_YAW].type =   HINGE;
    rjoint[LEFT_SHOULDER_YAW].px   =   rlink[LEFT_UPPER_ARM_DUMMY1].px;
    rjoint[LEFT_SHOULDER_YAW].py   =   rlink[LEFT_UPPER_ARM_DUMMY1].py - 0.5 * JOINT_SIZE;
    rjoint[LEFT_SHOULDER_YAW].pz   =   rlink[LEFT_UPPER_ARM_DUMMY1].pz;
    rjoint[LEFT_SHOULDER_YAW].axis_x  =   0;
    rjoint[LEFT_SHOULDER_YAW].axis_y  =   0;
    rjoint[LEFT_SHOULDER_YAW].axis_z  =   1;
    rjoint[LEFT_SHOULDER_YAW].lo_stop  = - 179 * (M_PI/180.0);
    rjoint[LEFT_SHOULDER_YAW].hi_stop  =   179 * (M_PI/180.0);
    rjoint[LEFT_SHOULDER_YAW].fmax     =   FMAX;
    rjoint[LEFT_SHOULDER_YAW].link[0]  =   TORSO;
    rjoint[LEFT_SHOULDER_YAW].link[1]  =   LEFT_UPPER_ARM_DUMMY1;
    rjoint[LEFT_SHOULDER_ROLL].type    =   HINGE;
    rjoint[LEFT_SHOULDER_ROLL].px      =   rlink[LEFT_UPPER_ARM_DUMMY2].px;
    rjoint[LEFT_SHOULDER_ROLL].py      =   rlink[LEFT_UPPER_ARM_DUMMY2].py;
    rjoint[LEFT_SHOULDER_ROLL].pz      =   rlink[LEFT_UPPER_ARM_DUMMY2].pz - 0.5 * JOINT_SIZE;
    rjoint[LEFT_SHOULDER_ROLL].axis_x  =   1;
    rjoint[LEFT_SHOULDER_ROLL].axis_y  =   0;
    rjoint[LEFT_SHOULDER_ROLL].axis_z  =   0;
    rjoint[LEFT_SHOULDER_ROLL].lo_stop = - 179 * (M_PI/180.0);
    rjoint[LEFT_SHOULDER_ROLL].hi_stop =   179 * (M_PI/180.0);
    rjoint[LEFT_SHOULDER_ROLL].fmax    =   FMAX;
    rjoint[LEFT_SHOULDER_ROLL].link[0] =   LEFT_UPPER_ARM_DUMMY1;
    rjoint[LEFT_SHOULDER_ROLL].link[1] =   LEFT_UPPER_ARM_DUMMY2;
    rjoint[LEFT_SHOULDER_PITCH].type   =   HINGE;
    rjoint[LEFT_SHOULDER_PITCH].px     =   rlink[LEFT_UPPER_ARM].px;
    rjoint[LEFT_SHOULDER_PITCH].py     =   rlink[LEFT_UPPER_ARM].py;
    rjoint[LEFT_SHOULDER_PITCH].pz     =   rlink[LEFT_UPPER_ARM].pz + 0.5 * UPPER_ARM_HEIGHT;
    rjoint[LEFT_SHOULDER_PITCH].axis_x =   0;
    rjoint[LEFT_SHOULDER_PITCH].axis_y =   1;
    rjoint[LEFT_SHOULDER_PITCH].axis_z =   0;
    rjoint[LEFT_SHOULDER_PITCH].lo_stop = - 179 * (M_PI/180.0);
    rjoint[LEFT_SHOULDER_PITCH].hi_stop =   179 * (M_PI/180.0);
    rjoint[LEFT_SHOULDER_PITCH].fmax    =   FMAX;
    rjoint[LEFT_SHOULDER_PITCH].link[0] =   LEFT_UPPER_ARM_DUMMY2;
    rjoint[LEFT_SHOULDER_PITCH].link[1] =   LEFT_UPPER_ARM;

    // right shoulder (YAW: z, ROLL: x, PITCH: y)
    rjoint[RIGHT_SHOULDER_YAW].type =   rjoint[LEFT_SHOULDER_YAW].type;
    rjoint[RIGHT_SHOULDER_YAW].px   =   rjoint[LEFT_SHOULDER_YAW].px;
    rjoint[RIGHT_SHOULDER_YAW].py   = - rjoint[LEFT_SHOULDER_YAW].py + 2.0 * STARTY; // change
    rjoint[RIGHT_SHOULDER_YAW].pz   =   rjoint[LEFT_SHOULDER_YAW].pz;
    rjoint[RIGHT_SHOULDER_YAW].axis_x  =   rjoint[LEFT_SHOULDER_YAW].axis_x;
    rjoint[RIGHT_SHOULDER_YAW].axis_y  =   rjoint[LEFT_SHOULDER_YAW].axis_y;
    rjoint[RIGHT_SHOULDER_YAW].axis_z  =   rjoint[LEFT_SHOULDER_YAW].axis_z;
    rjoint[RIGHT_SHOULDER_YAW].lo_stop  =   rjoint[LEFT_SHOULDER_YAW].lo_stop;
    rjoint[RIGHT_SHOULDER_YAW].hi_stop  =   rjoint[LEFT_SHOULDER_YAW].hi_stop;
    rjoint[RIGHT_SHOULDER_YAW].fmax     =   rjoint[LEFT_SHOULDER_YAW].fmax;
    rjoint[RIGHT_SHOULDER_YAW].link[0]  =   TORSO;
    rjoint[RIGHT_SHOULDER_YAW].link[1]  =   RIGHT_UPPER_ARM_DUMMY1;
    rjoint[RIGHT_SHOULDER_ROLL].type    =   rjoint[LEFT_SHOULDER_ROLL].type;
    rjoint[RIGHT_SHOULDER_ROLL].px   =   rjoint[LEFT_SHOULDER_ROLL].px;
    rjoint[RIGHT_SHOULDER_ROLL].py   = - rjoint[LEFT_SHOULDER_ROLL].py + 2.0 * STARTY; // change
    rjoint[RIGHT_SHOULDER_ROLL].pz   =   rjoint[LEFT_SHOULDER_ROLL].pz;
    rjoint[RIGHT_SHOULDER_ROLL].axis_x  =   rjoint[LEFT_SHOULDER_ROLL].axis_x;
    rjoint[RIGHT_SHOULDER_ROLL].axis_y  =   rjoint[LEFT_SHOULDER_ROLL].axis_y;
    rjoint[RIGHT_SHOULDER_ROLL].axis_z  =   rjoint[LEFT_SHOULDER_ROLL].axis_z;
    rjoint[RIGHT_SHOULDER_ROLL].lo_stop  =   rjoint[LEFT_SHOULDER_ROLL].lo_stop;
    rjoint[RIGHT_SHOULDER_ROLL].hi_stop  =   rjoint[LEFT_SHOULDER_ROLL].hi_stop;
    rjoint[RIGHT_SHOULDER_ROLL].fmax     =   rjoint[LEFT_SHOULDER_ROLL].fmax;
    rjoint[RIGHT_SHOULDER_ROLL].link[0]  =   RIGHT_UPPER_ARM_DUMMY1;
    rjoint[RIGHT_SHOULDER_ROLL].link[1]  =   RIGHT_UPPER_ARM_DUMMY2;
    rjoint[RIGHT_SHOULDER_PITCH].type    =   rjoint[LEFT_SHOULDER_PITCH].type;
    rjoint[RIGHT_SHOULDER_PITCH].px   =   rjoint[LEFT_SHOULDER_PITCH].px;
    rjoint[RIGHT_SHOULDER_PITCH].py   = - rjoint[LEFT_SHOULDER_PITCH].py + 2.0 * STARTY; // change
    rjoint[RIGHT_SHOULDER_PITCH].pz   =   rjoint[LEFT_SHOULDER_PITCH].pz;
    rjoint[RIGHT_SHOULDER_PITCH].axis_x  =   rjoint[LEFT_SHOULDER_PITCH].axis_x;
    rjoint[RIGHT_SHOULDER_PITCH].axis_y  =   rjoint[LEFT_SHOULDER_PITCH].axis_y;
    rjoint[RIGHT_SHOULDER_PITCH].axis_z  =   rjoint[LEFT_SHOULDER_PITCH].axis_z;
    rjoint[RIGHT_SHOULDER_PITCH].lo_stop  =   rjoint[LEFT_SHOULDER_PITCH].lo_stop;
    rjoint[RIGHT_SHOULDER_PITCH].hi_stop  =   rjoint[LEFT_SHOULDER_PITCH].hi_stop;
    rjoint[RIGHT_SHOULDER_PITCH].fmax     =   rjoint[LEFT_SHOULDER_PITCH].fmax;
    rjoint[RIGHT_SHOULDER_PITCH].link[0]  =   RIGHT_UPPER_ARM_DUMMY2;
    rjoint[RIGHT_SHOULDER_PITCH].link[1]  =   RIGHT_UPPER_ARM;

    // left hip joint (YAW: z, ROLL: x, PITCH: y)
    rjoint[LEFT_HIP_YAW].type =   HINGE;
    rjoint[LEFT_HIP_YAW].px   =   rlink[LEFT_THIGH_DUMMY1].px;
    rjoint[LEFT_HIP_YAW].py   =   rlink[LEFT_THIGH_DUMMY1].py;
    rjoint[LEFT_HIP_YAW].pz   =   rlink[LEFT_THIGH_DUMMY1].pz + 0.5 * JOINT_SIZE;
    rjoint[LEFT_HIP_YAW].axis_x  =   0;
    rjoint[LEFT_HIP_YAW].axis_y  =   0;
    rjoint[LEFT_HIP_YAW].axis_z  =   1;
    rjoint[LEFT_HIP_YAW].lo_stop  = - 90 * (M_PI/180.0);
    rjoint[LEFT_HIP_YAW].hi_stop  =   90 * (M_PI/180.0);
    rjoint[LEFT_HIP_YAW].fmax     =   FMAX;
    rjoint[LEFT_HIP_YAW].link[0]  =   TORSO;
    rjoint[LEFT_HIP_YAW].link[1]  =   LEFT_THIGH_DUMMY1;
    rjoint[LEFT_HIP_ROLL].type    =   HINGE;
    rjoint[LEFT_HIP_ROLL].px   =   rlink[LEFT_THIGH_DUMMY2].px - 0.5 * JOINT_SIZE;
    rjoint[LEFT_HIP_ROLL].py   =   rlink[LEFT_THIGH_DUMMY2].py;
    rjoint[LEFT_HIP_ROLL].pz   =   rlink[LEFT_THIGH_DUMMY2].pz;
    rjoint[LEFT_HIP_ROLL].axis_x  =   1;
    rjoint[LEFT_HIP_ROLL].axis_y  =   0;
    rjoint[LEFT_HIP_ROLL].axis_z  =   0;
    rjoint[LEFT_HIP_ROLL].lo_stop  = - 90 * (M_PI/180.0);
    rjoint[LEFT_HIP_ROLL].hi_stop  =   90 * (M_PI/180.0);
    rjoint[LEFT_HIP_ROLL].fmax     =   FMAX;
    rjoint[LEFT_HIP_ROLL].link[0]  =   LEFT_THIGH_DUMMY1;
    rjoint[LEFT_HIP_ROLL].link[1]  =   LEFT_THIGH_DUMMY2;
    rjoint[LEFT_HIP_PITCH].type    =   HINGE;
    rjoint[LEFT_HIP_PITCH].px   =   rlink[LEFT_THIGH].px + 0.5 * JOINT_SIZE;
    rjoint[LEFT_HIP_PITCH].py   =   rlink[LEFT_THIGH].py;
    rjoint[LEFT_HIP_PITCH].pz   =   rlink[LEFT_THIGH].pz + 0.5 * THIGH_HEIGHT;
    rjoint[LEFT_HIP_PITCH].axis_x  =   0;
    rjoint[LEFT_HIP_PITCH].axis_y  =   1;
    rjoint[LEFT_HIP_PITCH].axis_z  =   0;
    rjoint[LEFT_HIP_PITCH].lo_stop  = - 90 * (M_PI/180.0);
    rjoint[LEFT_HIP_PITCH].hi_stop  =   90 * (M_PI/180.0);
    rjoint[LEFT_HIP_PITCH].fmax     =   FMAX;
    rjoint[LEFT_HIP_PITCH].link[0]  =   LEFT_THIGH_DUMMY2;
    rjoint[LEFT_HIP_PITCH].link[1]  =   LEFT_THIGH;

    // right hip joint (JT1: roll, JT2: yaw, JT3: pitch)
    rjoint[RIGHT_HIP_YAW].type =   rjoint[LEFT_HIP_YAW].type;
    rjoint[RIGHT_HIP_YAW].px   =   rjoint[LEFT_HIP_YAW].px;
    rjoint[RIGHT_HIP_YAW].py   = - rjoint[LEFT_HIP_YAW].py + 2.0 * STARTY; // change
    rjoint[RIGHT_HIP_YAW].pz   =   rjoint[LEFT_HIP_YAW].pz;
    rjoint[RIGHT_HIP_YAW].axis_x  =   rjoint[LEFT_HIP_YAW].axis_x;
    rjoint[RIGHT_HIP_YAW].axis_y  =   rjoint[LEFT_HIP_YAW].axis_y;
    rjoint[RIGHT_HIP_YAW].axis_z  =   rjoint[LEFT_HIP_YAW].axis_z;
    rjoint[RIGHT_HIP_YAW].lo_stop  =   rjoint[LEFT_HIP_YAW].lo_stop;
    rjoint[RIGHT_HIP_YAW].hi_stop  =   rjoint[LEFT_HIP_YAW].hi_stop;
    rjoint[RIGHT_HIP_YAW].fmax     =   rjoint[LEFT_HIP_YAW].fmax;
    rjoint[RIGHT_HIP_YAW].link[0]  =   TORSO;
    rjoint[RIGHT_HIP_YAW].link[1]  =   RIGHT_THIGH_DUMMY1;
    rjoint[RIGHT_HIP_ROLL].type    =   rjoint[LEFT_HIP_ROLL].type;
    rjoint[RIGHT_HIP_ROLL].px   =   rjoint[LEFT_HIP_ROLL].px;
    rjoint[RIGHT_HIP_ROLL].py   = - rjoint[LEFT_HIP_ROLL].py + 2.0 * STARTY; // change
    rjoint[RIGHT_HIP_ROLL].pz   =   rjoint[LEFT_HIP_ROLL].pz;
    rjoint[RIGHT_HIP_ROLL].axis_x  =   rjoint[LEFT_HIP_ROLL].axis_x;
    rjoint[RIGHT_HIP_ROLL].axis_y  =   rjoint[LEFT_HIP_ROLL].axis_y;
    rjoint[RIGHT_HIP_ROLL].axis_z  =   rjoint[LEFT_HIP_ROLL].axis_z;
    rjoint[RIGHT_HIP_ROLL].lo_stop  =   rjoint[LEFT_HIP_ROLL].lo_stop;
    rjoint[RIGHT_HIP_ROLL].hi_stop  =   rjoint[LEFT_HIP_ROLL].hi_stop;
    rjoint[RIGHT_HIP_ROLL].fmax     =   rjoint[LEFT_HIP_ROLL].fmax;
    rjoint[RIGHT_HIP_ROLL].link[0]  =   RIGHT_THIGH_DUMMY1;
    rjoint[RIGHT_HIP_ROLL].link[1]  =   RIGHT_THIGH_DUMMY2;
    rjoint[RIGHT_HIP_PITCH].type    =   rjoint[LEFT_HIP_PITCH].type;
    rjoint[RIGHT_HIP_PITCH].px   =   rjoint[LEFT_HIP_PITCH].px;
    rjoint[RIGHT_HIP_PITCH].py   = - rjoint[LEFT_HIP_PITCH].py + 2.0 * STARTY; // change
    rjoint[RIGHT_HIP_PITCH].pz   =   rjoint[LEFT_HIP_PITCH].pz;
    rjoint[RIGHT_HIP_PITCH].axis_x  =   rjoint[LEFT_HIP_PITCH].axis_x;
    rjoint[RIGHT_HIP_PITCH].axis_y  =   rjoint[LEFT_HIP_PITCH].axis_y;
    rjoint[RIGHT_HIP_PITCH].axis_z  =   rjoint[LEFT_HIP_PITCH].axis_z;
    rjoint[RIGHT_HIP_PITCH].lo_stop  =   rjoint[LEFT_HIP_PITCH].lo_stop;
    rjoint[RIGHT_HIP_PITCH].hi_stop  =   rjoint[LEFT_HIP_PITCH].hi_stop;
    rjoint[RIGHT_HIP_PITCH].fmax     =   rjoint[LEFT_HIP_PITCH].fmax;
    rjoint[RIGHT_HIP_PITCH].link[0]  =   RIGHT_THIGH_DUMMY2;
    rjoint[RIGHT_HIP_PITCH].link[1]  =   RIGHT_THIGH;

    // left elbow joint (pitch)
    rjoint[LEFT_ELBOW_PITCH].type =   HINGE;
    rjoint[LEFT_ELBOW_PITCH].px   =   rlink[LEFT_UPPER_ARM].px;
    rjoint[LEFT_ELBOW_PITCH].py   =   rlink[LEFT_UPPER_ARM].py;
    rjoint[LEFT_ELBOW_PITCH].pz   =   rlink[LEFT_UPPER_ARM].pz - 0.5 * UPPER_ARM_HEIGHT;
    rjoint[LEFT_ELBOW_PITCH].axis_x   =   0;
    rjoint[LEFT_ELBOW_PITCH].axis_y   =   1;
    rjoint[LEFT_ELBOW_PITCH].axis_z   =   0;
    rjoint[LEFT_ELBOW_PITCH].lo_stop  = - 179 * (M_PI/180.0);
    rjoint[LEFT_ELBOW_PITCH].hi_stop  =   0;
    rjoint[LEFT_ELBOW_PITCH].fmax     =   FMAX;
    rjoint[LEFT_ELBOW_PITCH].link[0]  =   LEFT_UPPER_ARM;
    rjoint[LEFT_ELBOW_PITCH].link[1]  =   LEFT_FORE_ARM;
    rjoint[LEFT_ELBOW_PITCH].fudge_factor = 0.1;

    // right elbow joint (pitch)
    rjoint[RIGHT_ELBOW_PITCH].type =   rjoint[LEFT_ELBOW_PITCH].type;
    rjoint[RIGHT_ELBOW_PITCH].px   =   rjoint[LEFT_ELBOW_PITCH].px;
    rjoint[RIGHT_ELBOW_PITCH].py   = - rjoint[LEFT_ELBOW_PITCH].py + 2.0 * STARTY; // change
    rjoint[RIGHT_ELBOW_PITCH].pz   =   rjoint[LEFT_ELBOW_PITCH].pz;
    rjoint[RIGHT_ELBOW_PITCH].axis_x   =   rjoint[LEFT_ELBOW_PITCH].axis_x;
    rjoint[RIGHT_ELBOW_PITCH].axis_y   =   rjoint[LEFT_ELBOW_PITCH].axis_y;
    rjoint[RIGHT_ELBOW_PITCH].axis_z   =   rjoint[LEFT_ELBOW_PITCH].axis_z;
    rjoint[RIGHT_ELBOW_PITCH].lo_stop  =   rjoint[LEFT_ELBOW_PITCH].lo_stop;
    rjoint[RIGHT_ELBOW_PITCH].hi_stop  =   rjoint[LEFT_ELBOW_PITCH].hi_stop;
    rjoint[RIGHT_ELBOW_PITCH].fmax     =   rjoint[LEFT_ELBOW_PITCH].fmax;
    rjoint[RIGHT_ELBOW_PITCH].link[0]  =   RIGHT_UPPER_ARM;
    rjoint[RIGHT_ELBOW_PITCH].link[1]  =   RIGHT_FORE_ARM;
    rjoint[RIGHT_ELBOW_PITCH].fudge_factor = rjoint[LEFT_ELBOW_PITCH].fudge_factor;

    // left knee joint
    rjoint[LEFT_KNEE_PITCH].type =   HINGE;
    rjoint[LEFT_KNEE_PITCH].px   =   rlink[LEFT_THIGH].px;
    rjoint[LEFT_KNEE_PITCH].py   =   rlink[LEFT_THIGH].py;
    rjoint[LEFT_KNEE_PITCH].pz   =   rlink[LEFT_THIGH].pz - 0.5 * THIGH_HEIGHT;
    rjoint[LEFT_KNEE_PITCH].axis_x  =   0;
    rjoint[LEFT_KNEE_PITCH].axis_y  =   1;
    rjoint[LEFT_KNEE_PITCH].axis_z  =   0;
    rjoint[LEFT_KNEE_PITCH].lo_stop =   0;  //  可動域 最小 max -pi
    rjoint[LEFT_KNEE_PITCH].hi_stop =   170.0 * (M_PI/180.0);  // 可動域 最大 max +pi
    rjoint[LEFT_KNEE_PITCH].fmax    =   FMAX;
    rjoint[LEFT_KNEE_PITCH].link[0] =   LEFT_THIGH;
    rjoint[LEFT_KNEE_PITCH].link[1] =   LEFT_CALF;
    rjoint[LEFT_KNEE_PITCH].fudge_factor = 0.9;

    // right knee joint
    rjoint[RIGHT_KNEE_PITCH].type =   rjoint[LEFT_KNEE_PITCH].type;
    rjoint[RIGHT_KNEE_PITCH].px   =   rjoint[LEFT_KNEE_PITCH].px;
    rjoint[RIGHT_KNEE_PITCH].py   = - rjoint[LEFT_KNEE_PITCH].py + 2.0 * STARTY; // change
    rjoint[RIGHT_KNEE_PITCH].pz   =   rjoint[LEFT_KNEE_PITCH].pz;
    rjoint[RIGHT_KNEE_PITCH].axis_x   =   rjoint[LEFT_KNEE_PITCH].axis_x;
    rjoint[RIGHT_KNEE_PITCH].axis_y   =   rjoint[LEFT_KNEE_PITCH].axis_y;
    rjoint[RIGHT_KNEE_PITCH].axis_z   =   rjoint[LEFT_KNEE_PITCH].axis_z;
    rjoint[RIGHT_KNEE_PITCH].lo_stop  =   rjoint[LEFT_KNEE_PITCH].lo_stop;
    rjoint[RIGHT_KNEE_PITCH].hi_stop  =   rjoint[LEFT_KNEE_PITCH].hi_stop;
    rjoint[RIGHT_KNEE_PITCH].fmax     =   rjoint[LEFT_KNEE_PITCH].fmax;
    rjoint[RIGHT_KNEE_PITCH].link[0]  =   RIGHT_THIGH;
    rjoint[RIGHT_KNEE_PITCH].link[1]  =   RIGHT_CALF;
    rjoint[RIGHT_KNEE_PITCH].fudge_factor = rjoint[RIGHT_KNEE_PITCH].fudge_factor;

    // left foot joint (pitch: y axis, roll: x axis)
    rjoint[LEFT_FOOT_PITCH].type    =   HINGE; //HINGE
    rjoint[LEFT_FOOT_PITCH].px      =   rlink[LEFT_CALF].px;
    rjoint[LEFT_FOOT_PITCH].py      =   rlink[LEFT_CALF].py;
    rjoint[LEFT_FOOT_PITCH].pz      =   rlink[LEFT_CALF].pz - 0.5 * CALF_LENGTH;
    rjoint[LEFT_FOOT_PITCH].axis_x  =   0;
    rjoint[LEFT_FOOT_PITCH].axis_y  =   1;
    rjoint[LEFT_FOOT_PITCH].axis_z  =   0;
    rjoint[LEFT_FOOT_PITCH].lo_stop = - 90.0 * (M_PI/180.0);
    rjoint[LEFT_FOOT_PITCH].hi_stop =   90.0 * (M_PI/180.0);
    rjoint[LEFT_FOOT_PITCH].fmax    =   FMAX;
    rjoint[LEFT_FOOT_PITCH].link[0] =   LEFT_CALF;
    rjoint[LEFT_FOOT_PITCH].link[1] =   LEFT_FOOT_DUMMY;
    rjoint[LEFT_FOOT_ROLL].type     =   HINGE;
    rjoint[LEFT_FOOT_ROLL].px       =   rlink[LEFT_CALF].px;
    rjoint[LEFT_FOOT_ROLL].py       =   rlink[LEFT_CALF].py;
    rjoint[LEFT_FOOT_ROLL].pz       =   rlink[LEFT_FOOT_DUMMY].pz - 0.5 * (JOINT_SIZE);
    rjoint[LEFT_FOOT_ROLL].axis_x   =   1;  // ピッチ
    rjoint[LEFT_FOOT_ROLL].axis_y   =   0;
    rjoint[LEFT_FOOT_ROLL].axis_z   =   0;
    rjoint[LEFT_FOOT_ROLL].lo_stop  = - 90.0 * (M_PI/180.0); // 90.0
    rjoint[LEFT_FOOT_ROLL].hi_stop  =   90.0 * (M_PI/180.0);
    rjoint[LEFT_FOOT_ROLL].fmax     =   FMAX;
    rjoint[LEFT_FOOT_ROLL].link[0]  =   LEFT_FOOT_DUMMY;
    rjoint[LEFT_FOOT_ROLL].link[1]  =   LEFT_FOOT;

    // right foot joint
    rjoint[RIGHT_FOOT_PITCH].type    =   rjoint[LEFT_FOOT_PITCH].type;
    rjoint[RIGHT_FOOT_PITCH].px      =   rjoint[LEFT_FOOT_PITCH].px;
    rjoint[RIGHT_FOOT_PITCH].py      = - rjoint[LEFT_FOOT_PITCH].py + 2.0 * STARTY; // change
    rjoint[RIGHT_FOOT_PITCH].pz      =   rjoint[LEFT_FOOT_PITCH].pz;
    rjoint[RIGHT_FOOT_PITCH].axis_x  =   rjoint[LEFT_FOOT_PITCH].axis_x;
    rjoint[RIGHT_FOOT_PITCH].axis_y  =   rjoint[LEFT_FOOT_PITCH].axis_y;
    rjoint[RIGHT_FOOT_PITCH].axis_z  =   rjoint[LEFT_FOOT_PITCH].axis_z;
    rjoint[RIGHT_FOOT_PITCH].lo_stop =   rjoint[LEFT_FOOT_PITCH].lo_stop;
    rjoint[RIGHT_FOOT_PITCH].hi_stop =   rjoint[LEFT_FOOT_PITCH].hi_stop;
    rjoint[RIGHT_FOOT_PITCH].fmax    =   rjoint[LEFT_FOOT_PITCH].fmax;
    rjoint[RIGHT_FOOT_PITCH].link[0] =   RIGHT_CALF;
    rjoint[RIGHT_FOOT_PITCH].link[1] =   RIGHT_FOOT_DUMMY;
    rjoint[RIGHT_FOOT_ROLL].type     =   rjoint[LEFT_FOOT_ROLL].type;
    rjoint[RIGHT_FOOT_ROLL].px       =   rjoint[LEFT_FOOT_ROLL].px;
    rjoint[RIGHT_FOOT_ROLL].py       = - rjoint[LEFT_FOOT_ROLL].py + 2.0 * STARTY; // chnage
    rjoint[RIGHT_FOOT_ROLL].pz       =   rjoint[LEFT_FOOT_ROLL].pz;
    rjoint[RIGHT_FOOT_ROLL].axis_x   =   rjoint[LEFT_FOOT_ROLL].axis_x;
    rjoint[RIGHT_FOOT_ROLL].axis_y   =   rjoint[LEFT_FOOT_ROLL].axis_y;
    rjoint[RIGHT_FOOT_ROLL].axis_z   =   rjoint[LEFT_FOOT_ROLL].axis_z;
    rjoint[RIGHT_FOOT_ROLL].lo_stop  =   rjoint[LEFT_FOOT_ROLL].hi_stop;
    rjoint[RIGHT_FOOT_ROLL].hi_stop  =   rjoint[LEFT_FOOT_ROLL].lo_stop;
    rjoint[RIGHT_FOOT_ROLL].fmax     =   rjoint[LEFT_FOOT_ROLL].fmax;
    rjoint[RIGHT_FOOT_ROLL].link[0]  =   RIGHT_FOOT_DUMMY;
    rjoint[RIGHT_FOOT_ROLL].link[1]  =   RIGHT_FOOT;
}

static void makeRobot(int robotNum)
{
    readLinkParam(robotNum);
    readJointParam();

    for (int i=0; i< BODY_NUM; i++)
    {
        rlink[i].id = dBodyCreate(world);
        dBodySetPosition(rlink[i].id, rlink[i].px, rlink[i].py, rlink[i].pz);

        if ((i == HEAD) ||  (i == LEFT_HAND) || (i == RIGHT_HAND))
        {
            dMass m;
            dMassSetSphereTotal(&m, rlink[i].m, rlink[i].r);
            dMassAdjust(&m,rlink[i].m);
            dBodySetMass(rlink[i].id, &m);
            rlink[i].gid = dCreateSphere(space,rlink[i].r);
            dGeomSetBody(rlink[i].gid, rlink[i].id);
        }
        else
        {
            if ((i == LEFT_FOOT) || (i == RIGHT_FOOT)) {
              dMass m;
              dMassSetBoxTotal(&m, rlink[i].m, rlink[i].lx, rlink[i].ly, rlink[i].lz);
              dMassAdjust(&m,rlink[i].m);
              dBodySetMass(rlink[i].id, &m);
              rlink[i].gid = dCreateBox(space, rlink[i].lx, rlink[i].ly, rlink[i].lz);
              dGeomSetBody(rlink[i].gid, rlink[i].id);
            }
            else {
              dMass m;
              dMassSetCylinderTotal(&m, rlink[i].m, 3, 0.5 * rlink[i].lx, rlink[i].lz);
              dMassAdjust(&m,rlink[i].m);
              dBodySetMass(rlink[i].id, &m);
              rlink[i].gid = dCreateCylinder(space, 0.5 * rlink[i].lx, rlink[i].lz);
              dGeomSetBody(rlink[i].gid, rlink[i].id);
            }
        }
    }

    // Set joint paramter
    for (int i=0; i< JOINT_NUM; i++)
    {
        rjoint[i].id = dJointCreateHinge(world,0);
        dJointAttach(rjoint[i].id , rlink[rjoint[i].link[1]].id, rlink[rjoint[i].link[0]].id);
        dJointSetHingeAnchor(rjoint[i].id, rjoint[i].px, rjoint[i].py, rjoint[i].pz);
        dJointSetHingeAxis(rjoint[i].id, rjoint[i].axis_x, rjoint[i].axis_y,rjoint[i].axis_z);
        dJointSetHingeParam(rjoint[i].id, dParamLoStop, rjoint[i].lo_stop);
        dJointSetHingeParam(rjoint[i].id, dParamHiStop, rjoint[i].hi_stop);
        dJointSetHingeParam(rjoint[i].id, dParamFMax,   rjoint[i].fmax);
        dJointSetHingeParam(rjoint[i].id, dParamFudgeFactor, FUDGE_FACTOR);
    }
}

int main(int argc, char *argv[]){
    fn.version = DS_VERSION;
    fn.start   = &start;
    fn.step    = &simLoop;
    fn.command = &command;
    //fn.path_to_textures = "../../drawstuff/textures";
    fn.path_to_textures = ".";

    dInitODE();
    world  = dWorldCreate();
    space  = dHashSpaceCreate(0);
    ground = dCreatePlane(space, 0, 0, 1, 0);
    contactgroup = dJointGroupCreate(0);

    dWorldSetGravity(world, 0, 0, -9.8);
    dWorldSetERP(world, 0.9);
    dWorldSetCFM(world, 1e-4);

    makeRobot(0);
    dsSimulationLoop(argc, argv, 800, 600, &fn);

    dJointGroupDestroy(contactgroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}
