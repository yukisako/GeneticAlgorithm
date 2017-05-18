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

#include "original.h"
#include <stdlib.h>
#ifdef MSVC
#pragma warning(disable:4244 4305) // for VC++, no precision loss complaints
#endif

double STARTY = 0.0;
int robotNum = 0;
#define EXIST_ROBOT_NUM 100


int action[EXIST_ROBOT_NUM][ACTION_NUM];
FILE *fp;
FILE *record;
int currentGeneration;
char filename[100];

int step[EXIST_ROBOT_NUM];
int sw[EXIST_ROBOT_NUM];

static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
    // const int contact_no = 10;
    // for (int robotNum = 0; robotNum < EXIST_ROBOT_NUM; robotNum++){
    //     dBodyID b1 = dGeomGetBody(o1);
    //     dBodyID b2 = dGeomGetBody(o2);
    //     if (b1 && b2 && dAreConnected(b1,b2)) return;

    //     if ((b1 == rlink[robotNum][LEFT_UPPER_ARM_DUMMY1].id) ||  (b1 == rlink[robotNum][LEFT_UPPER_ARM_DUMMY2].id)  || (b1 == rlink[robotNum][LEFT_THIGH_DUMMY1].id) || (b1 == rlink[robotNum][LEFT_THIGH_DUMMY2].id)
    //        || (b1 == rlink[robotNum][LEFT_THIGH_DUMMY1].id)     || (b1 == rlink[robotNum][LEFT_THIGH_DUMMY2].id)     || (b1 == rlink[robotNum][LEFT_FOOT_DUMMY].id)) return;
    //     if ((b2 == rlink[robotNum][RIGHT_UPPER_ARM_DUMMY1].id) || (b2 == rlink[robotNum][RIGHT_UPPER_ARM_DUMMY2].id) || (b2 == rlink[robotNum][RIGHT_THIGH_DUMMY1].id) || (b2 == rlink[robotNum][RIGHT_THIGH_DUMMY2].id)
    //        || (b2 == rlink[robotNum][RIGHT_THIGH_DUMMY1].id)     || (b2 == rlink[robotNum][RIGHT_THIGH_DUMMY2].id)   || (b2 == rlink[robotNum][RIGHT_FOOT_DUMMY].id)) return;
    //     if (((b1 == rlink[robotNum][LEFT_UPPER_ARM].id) && (b2 == rlink[robotNum][TORSO].id)) || ((b2 == rlink[robotNum][LEFT_UPPER_ARM].id) && (b1 == rlink[robotNum][TORSO].id)))      return;
    //     if (((b1 == rlink[robotNum][RIGHT_UPPER_ARM].id)&& (b2 == rlink[robotNum][TORSO].id)) || ((b2 == rlink[robotNum][RIGHT_UPPER_ARM].id)  && (b1 == rlink[robotNum][TORSO].id)))    return;
    //     if (((b1 == rlink[robotNum][LEFT_THIGH].id)     && (b2 == rlink[robotNum][TORSO].id)) || ((b2 == rlink[robotNum][LEFT_THIGH].id)     && (b1 == rlink[robotNum][TORSO].id)))      return;
    //     if (((b1 == rlink[robotNum][RIGHT_THIGH].id)    && (b2 == rlink[robotNum][TORSO].id)) || ((b2 == rlink[robotNum][RIGHT_THIGH].id)    && (b1 == rlink[robotNum][TORSO].id)))      return;
    //     if (((b1 == rlink[robotNum][LEFT_CALF].id)      && (b2 == rlink[robotNum][LEFT_FOOT].id)) || ((b2 == rlink[robotNum][LEFT_CALF].id)  && (b1 == rlink[robotNum][LEFT_CALF].id)))  return;
    //     if (((b1 == rlink[robotNum][RIGHT_CALF].id)     && (b2 == rlink[robotNum][RIGHT_FOOT].id))|| ((b2 == rlink[robotNum][RIGHT_CALF].id) && (b1 == rlink[robotNum][RIGHT_FOOT].id))) return;
    //     if ((b1 > rlink[robotNum][BODY_NUM-1].id) && (b2 > rlink[robotNum][BODY_NUM-1].id)) return;

    //     dContact contact[contact_no];
    //     int numc = dCollide(o1, o2, contact_no, &contact[0].geom, sizeof(dContact));

    //     for (int i=0; i<numc; i++)
    //     {
    //         contact[i].surface.mode       = dContactApprox1 | dContactBounce;
    //         contact[i].surface.mode       = dContactBounce;
    //         contact[i].surface.mu         = dInfinity;
    //         contact[i].surface.bounce     = 0.0;      // bouncing the objects
    //         contact[i].surface.bounce_vel = 0.0;      // bouncing velocity

    //         dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);

    //         dJointAttach(c, b1, b2);
    //     }
    // }

  static const int N = 10;
  dContact contact[N];
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);
  int isGround = ((ground == o1)||(ground == o2));

  int n = dCollide(o1, o2, N, &contact[0].geom, sizeof(dContact));

  if(isGround){
    for (int i = 0; i < n; ++i){
        contact[i].surface.mode       = dContactApprox1 | dContactBounce;
        contact[i].surface.bounce = 0.0;
        contact[i].surface.mu         = dInfinity;
        contact[i].surface.bounce_vel = 0.0;

        dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
        dJointAttach(c, b1,b2);
        }
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
    for (int robotNum = 0; robotNum < EXIST_ROBOT_NUM; robotNum++){
        sw[robotNum] = 0;
    }
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

    makeRobot(0);
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


static void control(int num, dReal target,int robotNum)
{
    dReal kp = 9.0, kd = 0.1, u, diff;
    // printf("diff:%f\n", diff);
    diff = target * M_PI/180.0 - dJointGetHingeAngle(rjoint[robotNum][num].id);
    u    = kp * diff - kd * dJointGetHingeAngleRate(rjoint[robotNum][num].id);
    dJointSetHingeParam(rjoint[robotNum][num].id, dParamVel, u);
    dJointSetHingeParam(rjoint[robotNum][num].id, dParamFMax, rjoint[robotNum][num].fmax);
}

static void controlMotor(int robotNum){
    // for (int robotNum = 0; robotNum < EXIST_ROBOT_NUM; robotNum++){


        control(RIGHT_SHOULDER_YAW,   right_shoulder_yaw_vec[robotNum],robotNum);
        control(RIGHT_SHOULDER_ROLL,  right_shoulder_roll_vec[robotNum],robotNum);
        control(RIGHT_SHOULDER_PITCH, right_shoulder_pitch_vec[robotNum],robotNum);
        control(RIGHT_ELBOW_PITCH,    right_elbow_pitch_vec[robotNum],robotNum);
        control(LEFT_SHOULDER_YAW,    left_shoulder_yaw_vec[robotNum],robotNum);
        control(LEFT_SHOULDER_ROLL,   left_shoulder_roll_vec[robotNum],robotNum);
        control(LEFT_SHOULDER_PITCH,  left_shoulder_pitch_vec[robotNum],robotNum);
        control(LEFT_ELBOW_PITCH,     left_elbow_pitch_vec[robotNum],robotNum);
        control(RIGHT_HIP_YAW,        right_hip_yaw_vec[robotNum],robotNum);
        control(RIGHT_HIP_ROLL,       right_hip_roll_vec[robotNum],robotNum);
        control(RIGHT_HIP_PITCH,      right_hip_pitch_vec[robotNum],robotNum);
        control(RIGHT_KNEE_PITCH,     right_knee_pitch_vec[robotNum],robotNum);
        control(RIGHT_FOOT_PITCH,     right_foot_pitch_vec[robotNum],robotNum);
        control(RIGHT_FOOT_ROLL,      right_foot_roll_vec[robotNum],robotNum);
        control(LEFT_HIP_YAW,         left_hip_yaw_vec[robotNum],robotNum);
        control(LEFT_HIP_ROLL,        left_hip_roll_vec[robotNum],robotNum);
        control(LEFT_HIP_PITCH,       left_hip_pitch_vec[robotNum],robotNum);
        control(LEFT_KNEE_PITCH,      left_knee_pitch_vec[robotNum],robotNum);
        control(LEFT_FOOT_PITCH,      left_foot_pitch_vec[robotNum],robotNum);
        control(LEFT_FOOT_ROLL,       left_foot_roll_vec[robotNum],robotNum);


        // control(RIGHT_SHOULDER_YAW,   0,robotNum);
        // control(RIGHT_SHOULDER_ROLL,  0,robotNum);
        // control(RIGHT_SHOULDER_PITCH, 0,robotNum);
        // control(RIGHT_ELBOW_PITCH,    0,robotNum);
        // control(LEFT_SHOULDER_YAW,    0,robotNum);
        // control(LEFT_SHOULDER_ROLL,   0,robotNum);
        // control(LEFT_SHOULDER_PITCH,  0,robotNum);
        // control(LEFT_ELBOW_PITCH,     0,robotNum);
        // control(RIGHT_HIP_YAW,        0,robotNum);
        // control(RIGHT_HIP_ROLL,       0,robotNum);
        // control(RIGHT_HIP_PITCH,      0,robotNum);
        // control(RIGHT_KNEE_PITCH,     0,robotNum);
        // control(RIGHT_FOOT_PITCH,     0,robotNum);
        // control(RIGHT_FOOT_ROLL,      0,robotNum);
        // control(LEFT_HIP_YAW,         0,robotNum);
        // control(LEFT_HIP_ROLL,        0,robotNum);
        // control(LEFT_HIP_PITCH,       0,robotNum);
        // control(LEFT_KNEE_PITCH,      0,robotNum);
        // control(LEFT_FOOT_PITCH,      0,robotNum);
        // control(LEFT_FOOT_ROLL,       0,robotNum);


        // printf("foot%d:%f\n",robotNum,left_foot_pitch_vec[robotNum],robotNum);
    // }
    
    // printf("R Shoulder Yaw  :%4.1f\n",dJointGetHingeAngle(rjoint[robotNum][RIGHT_SHOULDER_YAW].id)  / M_PI * 180);
    // printf("R Shoulder Roll :%4.1f\n",dJointGetHingeAngle(rjoint[robotNum][RIGHT_SHOULDER_ROLL].id) / M_PI * 180);
    // printf("R Shoulder Pitch:%4.1f\n",dJointGetHingeAngle(rjoint[robotNum][RIGHT_SHOULDER_PITCH].id)/ M_PI * 180);
    // printf("R Elbow Pitch   :%4.1f\n",dJointGetHingeAngle(rjoint[robotNum][RIGHT_ELBOW_PITCH].id)   / M_PI * 180);
    // printf("L Shoulder Yaw  :%4.1f\n",dJointGetHingeAngle(rjoint[robotNum][LEFT_SHOULDER_YAW].id)   / M_PI * 180);
    // printf("L Shoulder Roll :%4.1f\n",dJointGetHingeAngle(rjoint[robotNum][LEFT_SHOULDER_ROLL].id)  / M_PI * 180);
    // printf("L Shoulder Pitch:%4.1f\n",dJointGetHingeAngle(rjoint[robotNum][LEFT_SHOULDER_PITCH].id) / M_PI * 180);
    // printf("L Elbow Pitch   :%4.1f\n",dJointGetHingeAngle(rjoint[robotNum][LEFT_ELBOW_PITCH].id)    / M_PI * 180);
    // printf("R Hip Yaw   :%4.1f\n",dJointGetHingeAngle(rjoint[robotNum][RIGHT_HIP_YAW].id)    / M_PI * 180);
    // printf("R Hip Roll  :%4.1f\n",dJointGetHingeAngle(rjoint[robotNum][RIGHT_HIP_ROLL].id)   / M_PI * 180);
    // printf("R Hip Pitch :%4.1f\n",dJointGetHingeAngle(rjoint[robotNum][RIGHT_HIP_PITCH].id)  / M_PI * 180);
    // printf("R Knee Pitch:%4.1f\n",dJointGetHingeAngle(rjoint[robotNum][RIGHT_KNEE_PITCH].id) / M_PI * 180);
    // printf("R Foot Pitch:%4.1f\n",dJointGetHingeAngle(rjoint[robotNum][RIGHT_FOOT_PITCH].id) / M_PI * 180);
    // printf("R Foot Roll :%4.1f\n",dJointGetHingeAngle(rjoint[robotNum][RIGHT_FOOT_ROLL].id)  / M_PI * 180);
    // printf("L Hip Yaw   :%4.1f\n",dJointGetHingeAngle(rjoint[robotNum][LEFT_HIP_YAW].id)     / M_PI * 180);
    // printf("L Hip Roll  :%4.1f\n",dJointGetHingeAngle(rjoint[robotNum][LEFT_HIP_ROLL].id)    / M_PI * 180);
    // printf("L Hip Pitch :%4.1f\n",dJointGetHingeAngle(rjoint[robotNum][LEFT_HIP_PITCH].id)   / M_PI * 180);
    // printf("L Knee Pitch:%4.1f\n",dJointGetHingeAngle(rjoint[robotNum][LEFT_KNEE_PITCH].id)  / M_PI * 180);
    // printf("L Foot Pitch:%4.1f\n",dJointGetHingeAngle(rjoint[robotNum][LEFT_FOOT_PITCH].id)  / M_PI * 180);
    // printf("L Foot Roll :%4.1f\n",dJointGetHingeAngle(rjoint[robotNum][LEFT_FOOT_ROLL].id)   / M_PI * 180);
    
    //printf("L Foot Pitch:%4.1f\n",dJointGetHingeAngle(rjoint[robotNum][LEFT_FOOT_PITCH].id)  / M_PI * 180);
}

static void drawRobot(int robotNum)
{
    dReal myradius, myradius2,sides[3];

    for (int i=0; i< BODY_NUM; i++)
    {
        sides[0] = rlink[robotNum][i].lx;
        sides[1] = rlink[robotNum][i].ly;
        sides[2] = rlink[robotNum][i].lz;
        switch (i)
        {
        case HEAD:
            dsSetColor(1.3,0.6, 0.6);
            myradius2 = 0.8 * rlink[robotNum][i].r;
            dsDrawSphere(dBodyGetPosition(rlink[robotNum][i].id),dBodyGetRotation(rlink[robotNum][i].id),(float)myradius2);
            dsSetColor(1.3,1.3,1.3);
            myradius = rlink[robotNum][i].r;
            dsDrawSphere(dBodyGetPosition(rlink[robotNum][i].id),dBodyGetRotation(rlink[robotNum][i].id),(float)myradius);
            break;
        case RIGHT_HAND:
        case LEFT_HAND:
            dsSetColor(1.3,1.3,1.3);
            myradius = rlink[robotNum][i].r;
            dsDrawSphere(dBodyGetPosition(rlink[robotNum][i].id),dBodyGetRotation(rlink[robotNum][i].id),(float)myradius);
            break;
        case RIGHT_CALF:
        case LEFT_CALF:
            dsSetColor(0,0,1.3);
            dsDrawCapsule(dBodyGetPosition(rlink[robotNum][i].id),dBodyGetRotation(rlink[robotNum][i].id),
                          0.8 * sides[2],sides[0]);
                          break;
        case RIGHT_FOOT:
        case LEFT_FOOT:
            dsSetColor(1.3,1.3,1.3);
            dsDrawBox(dBodyGetPosition(rlink[robotNum][i].id),dBodyGetRotation(rlink[robotNum][i].id),sides);
            break;
        case TORSO:
            dsSetColor(1.3,1.3,1.3);
            dsDrawCapsule(dBodyGetPosition(rlink[robotNum][i].id),dBodyGetRotation(rlink[robotNum][i].id),
                          0.5 * sides[2], 0.5 * sides[0]);
            break;
        case NECK:
            dsSetColor(0,0,1.3);
            dsDrawCapsule(dBodyGetPosition(rlink[robotNum][i].id),dBodyGetRotation(rlink[robotNum][i].id),
                          2.0 *  sides[2],1.5 * sides[0]);
            break;
        default:
            dsSetColor(0,0,1.3);
            dsDrawCapsule(dBodyGetPosition(rlink[robotNum][i].id),dBodyGetRotation(rlink[robotNum][i].id),
                          0.9 * sides[2],sides[0]);
        }
    }

    dVector3 result1,result2,result3,result4,result5,result6;
    dJointGetHingeAnchor(rjoint[robotNum][LEFT_KNEE_PITCH].id ,   result1);
    dJointGetHingeAnchor(rjoint[robotNum][RIGHT_KNEE_PITCH].id,   result2);
    dJointGetHingeAnchor(rjoint[robotNum][LEFT_ELBOW_PITCH].id,   result3);
    dJointGetHingeAnchor(rjoint[robotNum][RIGHT_ELBOW_PITCH].id,  result4);
    dJointGetHingeAnchor(rjoint[robotNum][LEFT_SHOULDER_ROLL].id, result5);
    dJointGetHingeAnchor(rjoint[robotNum][RIGHT_SHOULDER_ROLL].id,result6);

    dsDrawSphere(result1,dBodyGetRotation(rlink[robotNum][0].id),(float) 0.9 * myradius);
    dsDrawSphere(result2,dBodyGetRotation(rlink[robotNum][0].id),(float) 0.9 * myradius);
    dsDrawSphere(result3,dBodyGetRotation(rlink[robotNum][0].id),(float) 0.7 * myradius);
    dsDrawSphere(result4,dBodyGetRotation(rlink[robotNum][0].id),(float) 0.7 * myradius);
    dsDrawSphere(result5,dBodyGetRotation(rlink[robotNum][0].id),(float) 0.6 * myradius);
    dsDrawSphere(result6,dBodyGetRotation(rlink[robotNum][0].id),(float) 0.6 * myradius);
}

void evaluation(){
    //ロボット個体の評価関数
    //ロボットは3000ステップ経過時の左足のX座標で評価する
    int bestRobotNum = 0;
    int secondRobotNum = 1;
    double bestRobotX = 0;
    double secondRobotX = 0;
    for (int robotNum = 0; robotNum < EXIST_ROBOT_NUM; ++robotNum){
        double *pos = (double *) dBodyGetPosition(rlink[robotNum][LEFT_FOOT].id);
        if(bestRobotX < pos[0]){
            //pos[0]がx座標
            bestRobotX = pos[0];
            bestRobotNum = robotNum;
        } else if (secondRobotX < pos[0]){
            //1番ではないが2番のとき
            secondRobotX = pos[0];
            secondRobotNum = robotNum;
        }
        printf("ロボット%dのX=%f\n",robotNum,pos[0]);
    }
    
    printf("トップが%d番の%f[m]，次が%d番の%f[m]です．\n",bestRobotNum,bestRobotX,secondRobotNum,secondRobotX);
    if((record=fopen("record.txt","a"))==NULL){
        printf("error\n");
        exit(1);
    }
    fprintf(record,"%004d世代: 1位%f[m]，2位の%f[m]\n",  currentGeneration,bestRobotX,secondRobotX);
    fclose(record);

    printf("\n優勝のロボット%dアクション\n",bestRobotNum);
    for (int i = 0; i < ACTION_NUM; ++i){
        printf("%d,",action[bestRobotNum][i]);
    }

    printf("\n準優勝ロボット%dアクション\n",secondRobotNum);
    for (int i = 0; i < ACTION_NUM; ++i){
        printf("%d,",action[secondRobotNum][i]);
    }


    //2つの親から新しい個体を生成
    int newAction[EXIST_ROBOT_NUM][ACTION_NUM];

    for (int i = 0; i < EXIST_ROBOT_NUM; ++i){
        for (int j = 0; j < ACTION_NUM; ++j){
            int random = rand();
            printf("random:%d\n",random );
            if(random % 200 == 0){
                //突然変異
                newAction[i][j] = rand()%8;
            } else if(random % 2 == 0){
                newAction[i][j] = action[bestRobotNum][j];
            } else {
                newAction[i][j] = action[secondRobotNum][j];
            }
        }
    }

    currentGeneration++;
    //新個体
    for (int i = 0; i < EXIST_ROBOT_NUM; ++i){
        printf("------------------\n");
        printf("%d番目の新しいロボット\n",i );
        for (int j = 0; j < ACTION_NUM; ++j){
            printf("%d,",newAction[i][j] );
        }
        printf("\n");
    }


    sprintf(filename, "./gene_data/%04d.csv", currentGeneration);

      if((fp=fopen(filename,"w"))==NULL){
        printf("error\n");
        exit(1);
      }

      for (int i = 0; i < EXIST_ROBOT_NUM; ++i){
        for (int j = 0; j < ACTION_NUM; ++j){
          fprintf(fp,"%d,",newAction[i][j]);
        }
        fprintf(fp,"\n");
      }

    exit(0);
}

static void simLoop(int pause)
{
    // printf("step:%d:%d\n",step[0],step[1] );
    int recovery_step = 20;
    static dReal  r_knee[MAX_ROBOT_NUM], r_hip[MAX_ROBOT_NUM];
    dReal knee_angle2[MAX_ROBOT_NUM];
    for (int robotNum = 0; robotNum < EXIST_ROBOT_NUM; robotNum++){
        r_knee[robotNum] = 0.0;
        knee_angle2[robotNum] = 106;
        r_hip[robotNum] = 0.0;
    }


    for (int robotNum = 0; robotNum < EXIST_ROBOT_NUM; robotNum++){
        // printf("r_knee[%d]:%f\n",robotNum,r_knee[robotNum]);
        if (step[robotNum] < ACTION_NUM){
            r_knee[robotNum] = 180.0 * dJointGetHingeAngle(rjoint[robotNum][RIGHT_KNEE_PITCH].id)/ M_PI;
            // if (sw[robotNum] == 0 && r_knee[robotNum] >   knee_angle2[robotNum] -1){
            //     sw[robotNum] = 1;
            //     step[robotNum] = 0;
            // }
            // if (sw[robotNum] == 1 && r_knee[robotNum] <=   25.0){
            //     sw[robotNum] = 2;
            //     step[robotNum] = 0;
            // }
            // if (sw[robotNum] == 2 && r_knee[robotNum] >=  knee_angle2[robotNum]){
            //     sw[robotNum] = 3;
            //     step[robotNum] = 0;
            // }
            // sw[robotNum] = rand() % 4 + 1;
            // printf("sw[%d]=%d\n",robotNum,sw[robotNum]);


            printf("%d体目の%dステップが%d",robotNum,step[robotNum],action[robotNum][step[robotNum]]);
            step[robotNum]++;
            switch (action[robotNum][step[robotNum]]){
            case 0:
                left_hip_pitch       = -100.0;
                left_knee_pitch      =  knee_angle2[robotNum];
                left_foot_pitch      =  -75.0;
                left_shoulder_roll   =  120.0;
                left_shoulder_pitch  =   30.0;
                break;
            case 1:
                right_hip_pitch       = -100.0;
                right_knee_pitch      =  knee_angle2[robotNum];
                right_foot_pitch      =  -75.0;
                right_shoulder_roll   =  120.0;
                right_shoulder_pitch  =   30.0;
                break;
            case 2:
                left_hip_pitch       =  0.0;
                left_knee_pitch      =  0.0;
                left_foot_pitch      =  0.0;
                left_shoulder_roll   =  0.0;
                left_shoulder_pitch  = - 90.0;
                break;
            case 3:
                right_hip_pitch       =  0.0;
                right_knee_pitch      =  0.0;
                right_foot_pitch      =  0.0;
                right_shoulder_roll   =  0.0;
                right_shoulder_pitch  = - 90.0;
                break;
            case 4:
                left_hip_pitch       = -100.0;
                left_knee_pitch      =  120.0;
                left_foot_pitch      =  -65,0; // - 65.0;
                left_shoulder_roll   =  180.0;
                left_shoulder_pitch  = - 90.0;
                break;
            case 5:
                right_hip_pitch       = -100.0;
                right_knee_pitch      =  120.0;
                right_foot_pitch      =  -65,0; // - 65.0;
                right_shoulder_roll   =  180.0;
                right_shoulder_pitch  = - 90.0;
                break;
            case 6:
                // if (step[robotNum] < recovery_step){
                    left_hip_pitch       = - 100 + step[robotNum] * 100 / recovery_step;
                    right_hip_pitch      =   left_hip_pitch;
                    left_knee_pitch      =   120 - step[robotNum] * 120 / recovery_step;
                    right_knee_pitch     =   left_knee_pitch;
                    left_foot_pitch      = - 65 + step[robotNum] *  60 / recovery_step;
                    right_foot_pitch     =   left_foot_pitch;
                // }

            case 7:
                // } else{
                left_hip_pitch       =  0.0;
                right_hip_pitch      =  left_hip_pitch;
                left_knee_pitch      =  0.0;
                right_knee_pitch     =  left_knee_pitch;
                left_foot_pitch      =  -9.0;
                right_foot_pitch     =  left_foot_pitch;

                //     //最初へ
                //     sw[robotNum] = 0;
                //     step[robotNum] = 0;
                // }
                left_shoulder_roll   =   0.0;
                right_shoulder_roll  =   left_shoulder_roll;
                left_shoulder_pitch  =   0.0;
                right_shoulder_pitch =   left_shoulder_pitch;
                break;
            }

            // left_hip_pitch_vec[robotNum] =  0;
            // right_hip_pitch_vec[robotNum] =  0;
            // left_knee_pitch_vec[robotNum]      =  0;
            // right_knee_pitch_vec[robotNum]     =  0;
            // left_foot_pitch_vec[robotNum]      =  0;
            // right_foot_pitch_vec[robotNum]     =  0;
            // left_shoulder_roll_vec[robotNum]   =  0;
            // right_shoulder_roll_vec[robotNum]  =  0;
            // left_shoulder_pitch_vec[robotNum]  =  0;
            // right_shoulder_pitch_vec[robotNum] =  0;
            // left_shoulder_roll_vec[robotNum]   =  0;
            // right_shoulder_roll_vec[robotNum]  =  0;
            // left_shoulder_pitch_vec[robotNum]  =  0;
            // right_shoulder_pitch_vec[robotNum] =  0;

            left_hip_pitch_vec[robotNum] =  left_hip_pitch;
            right_hip_pitch_vec[robotNum] =  right_hip_pitch;
            left_knee_pitch_vec[robotNum]      =  left_knee_pitch;
            right_knee_pitch_vec[robotNum]     =  right_knee_pitch;
            left_foot_pitch_vec[robotNum]      =  left_foot_pitch;
            right_foot_pitch_vec[robotNum]     =  right_foot_pitch;
            left_shoulder_roll_vec[robotNum]   =  left_shoulder_roll;
            right_shoulder_roll_vec[robotNum]  =  right_shoulder_roll;
            left_shoulder_pitch_vec[robotNum]  =  left_shoulder_pitch;
            right_shoulder_pitch_vec[robotNum] =  right_shoulder_pitch;
            left_shoulder_roll_vec[robotNum]   =  left_shoulder_roll;
            right_shoulder_roll_vec[robotNum]  =  right_shoulder_roll;
            left_shoulder_pitch_vec[robotNum]  =  left_shoulder_pitch;
            right_shoulder_pitch_vec[robotNum] =  right_shoulder_pitch;


            controlMotor(robotNum);

            // printf("%d\n",robotNum );
        } else {
            evaluation();
        }
        dJointGroupEmpty(contactgroup);
        drawRobot(robotNum);
        printf("\n");
        // printf("sw=%2d right knee=%5.1f right hip=%5.1f\r",sw[robotNum], r_knee[robotNum], r_hip[robotNum]);
        // printf("sw=%2d right foot=%5.1f left foot=%5.1f\n",sw[robotNum], right_foot_pitch_vec[robotNum], left_foot_pitch_vec[robotNum]);
        // printf("sw=%2d right hip =%5.1f left hip  =%5.1f\n",sw[robotNum], right_hip_pitch_vec[robotNum],  left_hip_pitch_vec[robotNum]);
    }
            dSpaceCollide(space,0,&nearCallback);
            dWorldStep(world,0.005);
}





static void readLinkParam(int robotNum)
{
    STARTY = robotNum*3;
    printf("%f\n", STARTY);
    rlink[robotNum][TORSO].lx = TORSO_LENGTH;
    rlink[robotNum][TORSO].ly = TORSO_WIDTH;
    rlink[robotNum][TORSO].lz = TORSO_HEIGHT;
    rlink[robotNum][TORSO].m  = TORSO_MASS;
    rlink[robotNum][TORSO].px = (dReal) STARTX;
    rlink[robotNum][TORSO].py = (dReal) STARTY;
    rlink[robotNum][TORSO].pz = (dReal) STARTZ;

    rlink[robotNum][NECK].lx = NECK_LENGTH;
    rlink[robotNum][NECK].ly = NECK_WIDTH;
    rlink[robotNum][NECK].lz = NECK_HEIGHT;
    rlink[robotNum][NECK].m  = NECK_MASS;
    rlink[robotNum][NECK].px = rlink[robotNum][TORSO].px;
    rlink[robotNum][NECK].py = rlink[robotNum][TORSO].py;
    rlink[robotNum][NECK].pz = rlink[robotNum][TORSO].pz + (dReal) 0.5 * (TORSO_HEIGHT + NECK_WIDTH);

    rlink[robotNum][HEAD].lx = HEAD_LENGTH;
    rlink[robotNum][HEAD].ly = HEAD_WIDTH;
    rlink[robotNum][HEAD].lz = HEAD_HEIGHT;
    rlink[robotNum][HEAD].r  = HEAD_RADIUS;
    rlink[robotNum][HEAD].m  = HEAD_MASS;
    rlink[robotNum][HEAD].px = rlink[robotNum][NECK].px;
    rlink[robotNum][HEAD].py = rlink[robotNum][NECK].py;
    rlink[robotNum][HEAD].pz = rlink[robotNum][NECK].pz + (dReal) 0.4 * NECK_HEIGHT + HEAD_RADIUS;

    rlink[robotNum][LEFT_UPPER_ARM_DUMMY1].lx = JOINT_SIZE;
    rlink[robotNum][LEFT_UPPER_ARM_DUMMY1].ly = JOINT_SIZE;
    rlink[robotNum][LEFT_UPPER_ARM_DUMMY1].lz = JOINT_SIZE;
    rlink[robotNum][LEFT_UPPER_ARM_DUMMY1].m  = JOINT_MASS;
    rlink[robotNum][LEFT_UPPER_ARM_DUMMY1].px = rlink[robotNum][TORSO].px;
    rlink[robotNum][LEFT_UPPER_ARM_DUMMY1].py = rlink[robotNum][TORSO].py + 0.5 * (TORSO_WIDTH + JOINT_SIZE);
    rlink[robotNum][LEFT_UPPER_ARM_DUMMY1].pz = rlink[robotNum][TORSO].pz + 0.5 * TORSO_HEIGHT - 0.5 * JOINT_SIZE;
    rlink[robotNum][LEFT_UPPER_ARM_DUMMY2].lx = JOINT_SIZE;
    rlink[robotNum][LEFT_UPPER_ARM_DUMMY2].ly = JOINT_SIZE;
    rlink[robotNum][LEFT_UPPER_ARM_DUMMY2].lz = JOINT_SIZE;
    rlink[robotNum][LEFT_UPPER_ARM_DUMMY2].m  = JOINT_MASS;
    rlink[robotNum][LEFT_UPPER_ARM_DUMMY2].px = rlink[robotNum][LEFT_UPPER_ARM_DUMMY1].px;
    rlink[robotNum][LEFT_UPPER_ARM_DUMMY2].py = rlink[robotNum][LEFT_UPPER_ARM_DUMMY1].py - 0.5 * JOINT_SIZE + 0.5 * HAND_RADIUS;
    rlink[robotNum][LEFT_UPPER_ARM_DUMMY2].pz = rlink[robotNum][LEFT_UPPER_ARM_DUMMY1].pz;
    rlink[robotNum][LEFT_UPPER_ARM].lx  = UPPER_ARM_LENGTH;
    rlink[robotNum][LEFT_UPPER_ARM].ly  = UPPER_ARM_WIDTH;
    rlink[robotNum][LEFT_UPPER_ARM].lz  = UPPER_ARM_HEIGHT;
    rlink[robotNum][LEFT_UPPER_ARM].m   = UPPER_ARM_MASS;
    rlink[robotNum][LEFT_UPPER_ARM].px  = rlink[robotNum][LEFT_UPPER_ARM_DUMMY2].px;
    rlink[robotNum][LEFT_UPPER_ARM].py  = rlink[robotNum][LEFT_UPPER_ARM_DUMMY2].py;
    rlink[robotNum][LEFT_UPPER_ARM].pz  = rlink[robotNum][LEFT_UPPER_ARM_DUMMY2].pz- 0.5 * (UPPER_ARM_HEIGHT + UPPER_ARM_WIDTH) ;

    rlink[robotNum][LEFT_FORE_ARM].lx  = FORE_ARM_LENGTH;
    rlink[robotNum][LEFT_FORE_ARM].ly  = FORE_ARM_WIDTH;
    rlink[robotNum][LEFT_FORE_ARM].lz  = FORE_ARM_HEIGHT;
    rlink[robotNum][LEFT_FORE_ARM].m   = FORE_ARM_MASS;
    rlink[robotNum][LEFT_FORE_ARM].px  = rlink[robotNum][LEFT_UPPER_ARM].px;
    rlink[robotNum][LEFT_FORE_ARM].py  = rlink[robotNum][LEFT_UPPER_ARM].py;
    rlink[robotNum][LEFT_FORE_ARM].pz  = rlink[robotNum][LEFT_UPPER_ARM].pz - 0.5 * (FORE_ARM_HEIGHT + FORE_ARM_WIDTH + UPPER_ARM_HEIGHT);

    rlink[robotNum][LEFT_HAND].lx  = HAND_LENGTH;
    rlink[robotNum][LEFT_HAND].ly  = HAND_WIDTH;
    rlink[robotNum][LEFT_HAND].lz  = HAND_HEIGHT;
    rlink[robotNum][LEFT_HAND].r   = HAND_RADIUS;
    rlink[robotNum][LEFT_HAND].m   = HAND_MASS;
    rlink[robotNum][LEFT_HAND].px  = rlink[robotNum][LEFT_FORE_ARM].px;
    rlink[robotNum][LEFT_HAND].py  = rlink[robotNum][LEFT_FORE_ARM].py;
    rlink[robotNum][LEFT_HAND].pz  = rlink[robotNum][LEFT_FORE_ARM].pz - 0.5 * (HAND_HEIGHT + FORE_ARM_WIDTH + FORE_ARM_HEIGHT);

    rlink[robotNum][RIGHT_UPPER_ARM_DUMMY1].lx =   rlink[robotNum][LEFT_UPPER_ARM_DUMMY1].lx;
    rlink[robotNum][RIGHT_UPPER_ARM_DUMMY1].ly =   rlink[robotNum][LEFT_UPPER_ARM_DUMMY1].ly;
    rlink[robotNum][RIGHT_UPPER_ARM_DUMMY1].lz =   rlink[robotNum][LEFT_UPPER_ARM_DUMMY1].lz;
    rlink[robotNum][RIGHT_UPPER_ARM_DUMMY1].m  =   rlink[robotNum][LEFT_UPPER_ARM_DUMMY1].m;
    rlink[robotNum][RIGHT_UPPER_ARM_DUMMY1].px =   rlink[robotNum][LEFT_UPPER_ARM_DUMMY1].px;
    rlink[robotNum][RIGHT_UPPER_ARM_DUMMY1].py = - rlink[robotNum][LEFT_UPPER_ARM_DUMMY1].py + 2.0 * STARTY; // change
    rlink[robotNum][RIGHT_UPPER_ARM_DUMMY1].pz =   rlink[robotNum][LEFT_UPPER_ARM_DUMMY1].pz;
    rlink[robotNum][RIGHT_UPPER_ARM_DUMMY2].lx =   rlink[robotNum][LEFT_UPPER_ARM_DUMMY2].lx;
    rlink[robotNum][RIGHT_UPPER_ARM_DUMMY2].ly =   rlink[robotNum][LEFT_UPPER_ARM_DUMMY2].ly;
    rlink[robotNum][RIGHT_UPPER_ARM_DUMMY2].lz =   rlink[robotNum][LEFT_UPPER_ARM_DUMMY2].lz;
    rlink[robotNum][RIGHT_UPPER_ARM_DUMMY2].m  =   rlink[robotNum][LEFT_UPPER_ARM_DUMMY2].m;
    rlink[robotNum][RIGHT_UPPER_ARM_DUMMY2].px =   rlink[robotNum][LEFT_UPPER_ARM_DUMMY2].px;
    rlink[robotNum][RIGHT_UPPER_ARM_DUMMY2].py = - rlink[robotNum][LEFT_UPPER_ARM_DUMMY2].py + 2.0 * STARTY; // change
    rlink[robotNum][RIGHT_UPPER_ARM_DUMMY2].pz =   rlink[robotNum][LEFT_UPPER_ARM_DUMMY2].pz;
    rlink[robotNum][RIGHT_UPPER_ARM].lx    =   UPPER_ARM_LENGTH;
    rlink[robotNum][RIGHT_UPPER_ARM].ly    =   UPPER_ARM_WIDTH;
    rlink[robotNum][RIGHT_UPPER_ARM].lz    =   UPPER_ARM_HEIGHT;
    rlink[robotNum][RIGHT_UPPER_ARM].m     =   UPPER_ARM_MASS;
    rlink[robotNum][RIGHT_UPPER_ARM].px    =   rlink[robotNum][LEFT_UPPER_ARM].px;
    rlink[robotNum][RIGHT_UPPER_ARM].py    = - rlink[robotNum][LEFT_UPPER_ARM].py + 2.0 * STARTY; // change
    rlink[robotNum][RIGHT_UPPER_ARM].pz    =   rlink[robotNum][LEFT_UPPER_ARM].pz;

    rlink[robotNum][RIGHT_FORE_ARM].lx =   FORE_ARM_LENGTH;
    rlink[robotNum][RIGHT_FORE_ARM].ly =   FORE_ARM_WIDTH;
    rlink[robotNum][RIGHT_FORE_ARM].lz =   FORE_ARM_HEIGHT;
    rlink[robotNum][RIGHT_FORE_ARM].m  =   FORE_ARM_MASS;
    rlink[robotNum][RIGHT_FORE_ARM].px =   rlink[robotNum][LEFT_FORE_ARM].px;
    rlink[robotNum][RIGHT_FORE_ARM].py = - rlink[robotNum][LEFT_FORE_ARM].py + 2.0 * STARTY; // change
    rlink[robotNum][RIGHT_FORE_ARM].pz =   rlink[robotNum][LEFT_FORE_ARM].pz;

    rlink[robotNum][RIGHT_HAND].lx  =   HAND_LENGTH;
    rlink[robotNum][RIGHT_HAND].ly  =   HAND_WIDTH;
    rlink[robotNum][RIGHT_HAND].lz  =   HAND_HEIGHT;
    rlink[robotNum][RIGHT_HAND].r   =   HAND_RADIUS;
    rlink[robotNum][RIGHT_HAND].m   =   HAND_MASS;
    rlink[robotNum][RIGHT_HAND].px  =   rlink[robotNum][LEFT_HAND].px;
    rlink[robotNum][RIGHT_HAND].py  = - rlink[robotNum][LEFT_HAND].py + 2.0* STARTY; // change
    rlink[robotNum][RIGHT_HAND].pz  =   rlink[robotNum][LEFT_HAND].pz;

    rlink[robotNum][LEFT_THIGH_DUMMY1].lx = JOINT_SIZE;
    rlink[robotNum][LEFT_THIGH_DUMMY1].ly = JOINT_SIZE;
    rlink[robotNum][LEFT_THIGH_DUMMY1].lz = JOINT_SIZE;
    rlink[robotNum][LEFT_THIGH_DUMMY1].m  = JOINT_MASS;
    rlink[robotNum][LEFT_THIGH_DUMMY1].px = rlink[robotNum][TORSO].px - 0.5 * JOINT_SIZE;
    rlink[robotNum][LEFT_THIGH_DUMMY1].py = rlink[robotNum][TORSO].py - 0.5 * HIP_JT_WIDTH;
    rlink[robotNum][LEFT_THIGH_DUMMY1].pz = rlink[robotNum][TORSO].pz - 0.5 * (TORSO_HEIGHT + JOINT_SIZE);
    rlink[robotNum][LEFT_THIGH_DUMMY2].lx = JOINT_SIZE;
    rlink[robotNum][LEFT_THIGH_DUMMY2].ly = JOINT_SIZE;
    rlink[robotNum][LEFT_THIGH_DUMMY2].lz = JOINT_SIZE;
    rlink[robotNum][LEFT_THIGH_DUMMY2].m  = JOINT_MASS;
    rlink[robotNum][LEFT_THIGH_DUMMY2].px = rlink[robotNum][LEFT_THIGH_DUMMY1].px + JOINT_SIZE;
    rlink[robotNum][LEFT_THIGH_DUMMY2].py = rlink[robotNum][LEFT_THIGH_DUMMY1].py;
    rlink[robotNum][LEFT_THIGH_DUMMY2].pz = rlink[robotNum][LEFT_THIGH_DUMMY1].pz;
    rlink[robotNum][LEFT_THIGH].lx    = THIGH_LENGTH;
    rlink[robotNum][LEFT_THIGH].ly    = THIGH_WIDTH;
    rlink[robotNum][LEFT_THIGH].lz    = THIGH_HEIGHT;
    rlink[robotNum][LEFT_THIGH].m     = THIGH_MASS;
    rlink[robotNum][LEFT_THIGH].px    = rlink[robotNum][LEFT_THIGH_DUMMY2].px - 0.5 * JOINT_SIZE;
    rlink[robotNum][LEFT_THIGH].py    = rlink[robotNum][LEFT_THIGH_DUMMY2].py;
    rlink[robotNum][LEFT_THIGH].pz    = rlink[robotNum][LEFT_THIGH_DUMMY2].pz - 0.5 * (JOINT_SIZE + THIGH_HEIGHT);

    rlink[robotNum][LEFT_CALF].lx   = CALF_LENGTH;
    rlink[robotNum][LEFT_CALF].ly   = CALF_WIDTH;
    rlink[robotNum][LEFT_CALF].lz   = CALF_HEIGHT;
    rlink[robotNum][LEFT_CALF].m    = CALF_MASS;
    rlink[robotNum][LEFT_CALF].px   = rlink[robotNum][LEFT_THIGH].px;
    rlink[robotNum][LEFT_CALF].py   = rlink[robotNum][LEFT_THIGH].py;
    rlink[robotNum][LEFT_CALF].pz   = rlink[robotNum][LEFT_THIGH].pz  - 0.5 * (CALF_HEIGHT + THIGH_HEIGHT);

    rlink[robotNum][LEFT_FOOT_DUMMY].lx = JOINT_SIZE;
    rlink[robotNum][LEFT_FOOT_DUMMY].ly = JOINT_SIZE;
    rlink[robotNum][LEFT_FOOT_DUMMY].lz = JOINT_SIZE;
    rlink[robotNum][LEFT_FOOT_DUMMY].m  = JOINT_MASS;
    rlink[robotNum][LEFT_FOOT_DUMMY].px = rlink[robotNum][LEFT_CALF].px;
    rlink[robotNum][LEFT_FOOT_DUMMY].py = rlink[robotNum][LEFT_CALF].py;
    rlink[robotNum][LEFT_FOOT_DUMMY].pz = rlink[robotNum][LEFT_CALF].pz - 0.5 * (CALF_HEIGHT + JOINT_SIZE);
    rlink[robotNum][LEFT_FOOT].lx   = FOOT_LENGTH;
    rlink[robotNum][LEFT_FOOT].ly   = FOOT_WIDTH;
    rlink[robotNum][LEFT_FOOT].lz   = FOOT_HEIGHT;
    rlink[robotNum][LEFT_FOOT].m    = FOOT_MASS;
    rlink[robotNum][LEFT_FOOT].px   = rlink[robotNum][LEFT_FOOT_DUMMY].px + 0.5 * FOOT_LENGTH - CALF_LENGTH;
    rlink[robotNum][LEFT_FOOT].py   = rlink[robotNum][LEFT_FOOT_DUMMY].py;
    rlink[robotNum][LEFT_FOOT].pz   = rlink[robotNum][LEFT_FOOT_DUMMY].pz - 0.5 * (FOOT_HEIGHT + JOINT_SIZE);

    rlink[robotNum][RIGHT_THIGH_DUMMY1].lx =   JOINT_SIZE;
    rlink[robotNum][RIGHT_THIGH_DUMMY1].ly =   JOINT_SIZE;
    rlink[robotNum][RIGHT_THIGH_DUMMY1].lz =   JOINT_SIZE;
    rlink[robotNum][RIGHT_THIGH_DUMMY1].m  =   JOINT_MASS;
    rlink[robotNum][RIGHT_THIGH_DUMMY1].px =   rlink[robotNum][LEFT_THIGH_DUMMY1].px;
    rlink[robotNum][RIGHT_THIGH_DUMMY1].py = - rlink[robotNum][LEFT_THIGH_DUMMY1].py + 2.0 * STARTY; // change
    rlink[robotNum][RIGHT_THIGH_DUMMY1].pz =   rlink[robotNum][LEFT_THIGH_DUMMY1].pz;
    rlink[robotNum][RIGHT_THIGH_DUMMY2].lx =   JOINT_SIZE;
    rlink[robotNum][RIGHT_THIGH_DUMMY2].ly =   JOINT_SIZE;
    rlink[robotNum][RIGHT_THIGH_DUMMY2].lz =   JOINT_SIZE;
    rlink[robotNum][RIGHT_THIGH_DUMMY2].m  =   JOINT_MASS;
    rlink[robotNum][RIGHT_THIGH_DUMMY2].px =   rlink[robotNum][LEFT_THIGH_DUMMY2].px;
    rlink[robotNum][RIGHT_THIGH_DUMMY2].py = - rlink[robotNum][LEFT_THIGH_DUMMY2].py + 2.0 * STARTY; // change
    rlink[robotNum][RIGHT_THIGH_DUMMY2].pz =   rlink[robotNum][LEFT_THIGH_DUMMY2].pz;
    rlink[robotNum][RIGHT_THIGH].lx    =   THIGH_LENGTH;
    rlink[robotNum][RIGHT_THIGH].ly    =   THIGH_WIDTH;
    rlink[robotNum][RIGHT_THIGH].lz    =   THIGH_HEIGHT;
    rlink[robotNum][RIGHT_THIGH].m     =   THIGH_MASS;
    rlink[robotNum][RIGHT_THIGH].px    =   rlink[robotNum][LEFT_THIGH].px;
    rlink[robotNum][RIGHT_THIGH].py    = - rlink[robotNum][LEFT_THIGH].py + 2.0 * STARTY; // change
    rlink[robotNum][RIGHT_THIGH].pz    =   rlink[robotNum][LEFT_THIGH].pz;

    rlink[robotNum][RIGHT_CALF].lx   =   CALF_LENGTH;
    rlink[robotNum][RIGHT_CALF].ly   =   CALF_WIDTH;
    rlink[robotNum][RIGHT_CALF].lz   =   rlink[robotNum][LEFT_CALF].lz;
    rlink[robotNum][RIGHT_CALF].m    =   CALF_MASS;
    rlink[robotNum][RIGHT_CALF].px   =   rlink[robotNum][LEFT_CALF].px;
    rlink[robotNum][RIGHT_CALF].py   = - rlink[robotNum][LEFT_CALF].py + 2.0 * STARTY;
    rlink[robotNum][RIGHT_CALF].pz   =   rlink[robotNum][LEFT_CALF].pz;

    rlink[robotNum][RIGHT_FOOT_DUMMY].lx =   rlink[robotNum][LEFT_FOOT_DUMMY].lx;
    rlink[robotNum][RIGHT_FOOT_DUMMY].ly =   rlink[robotNum][LEFT_FOOT_DUMMY].ly;
    rlink[robotNum][RIGHT_FOOT_DUMMY].lz =   rlink[robotNum][LEFT_FOOT_DUMMY].lz;
    rlink[robotNum][RIGHT_FOOT_DUMMY].m  =   rlink[robotNum][LEFT_FOOT_DUMMY].m;
    rlink[robotNum][RIGHT_FOOT_DUMMY].px =   rlink[robotNum][LEFT_FOOT_DUMMY].px;
    rlink[robotNum][RIGHT_FOOT_DUMMY].py = - rlink[robotNum][LEFT_FOOT_DUMMY].py + 2.0 * STARTY; // change
    rlink[robotNum][RIGHT_FOOT_DUMMY].pz =   rlink[robotNum][LEFT_FOOT_DUMMY].pz;
    rlink[robotNum][RIGHT_FOOT].lx   =   rlink[robotNum][LEFT_FOOT].lx;
    rlink[robotNum][RIGHT_FOOT].ly   =   rlink[robotNum][LEFT_FOOT].ly;
    rlink[robotNum][RIGHT_FOOT].lz   =   rlink[robotNum][LEFT_FOOT].lz;
    rlink[robotNum][RIGHT_FOOT].m    =   rlink[robotNum][LEFT_FOOT].m;
    rlink[robotNum][RIGHT_FOOT].px   =   rlink[robotNum][LEFT_FOOT].px;
    rlink[robotNum][RIGHT_FOOT].py   = - rlink[robotNum][LEFT_FOOT].py + 2.0 * STARTY; // change
    rlink[robotNum][RIGHT_FOOT].pz   =   rlink[robotNum][LEFT_FOOT].pz;
}

void readJointParam2(int robotNum)
{
    // neck joint (y axis)
    rjoint[robotNum][NECK_PITCH].type = HINGE;
    rjoint[robotNum][NECK_PITCH].px   = rlink[robotNum][NECK].px;
    rjoint[robotNum][NECK_PITCH].py   = rlink[robotNum][NECK].py;
    rjoint[robotNum][NECK_PITCH].pz   = rlink[robotNum][NECK].pz - 0.5 * NECK_HEIGHT;
    rjoint[robotNum][NECK_PITCH].axis_x  = 0;
    rjoint[robotNum][NECK_PITCH].axis_y  = 1;
    rjoint[robotNum][NECK_PITCH].axis_z  = 0;
    rjoint[robotNum][NECK_PITCH].lo_stop  = 0;
    rjoint[robotNum][NECK_PITCH].hi_stop  = 1;
    rjoint[robotNum][NECK_PITCH].fmax     = FMAX;
    rjoint[robotNum][NECK_PITCH].link[0]  = TORSO*(robotNum+1);
    rjoint[robotNum][NECK_PITCH].link[1]  = NECK*(robotNum+1);

    // head joint (y axis)
    rjoint[robotNum][HEAD_PITCH].type = HINGE;
    rjoint[robotNum][HEAD_PITCH].px   = rlink[robotNum][HEAD].px;
    rjoint[robotNum][HEAD_PITCH].py   = rlink[robotNum][HEAD].py;
    rjoint[robotNum][HEAD_PITCH].pz   = rlink[robotNum][HEAD].pz - HEAD_RADIUS;
    rjoint[robotNum][HEAD_PITCH].axis_x  = 0;
    rjoint[robotNum][HEAD_PITCH].axis_y  = 1;
    rjoint[robotNum][HEAD_PITCH].axis_z  = 0;
    rjoint[robotNum][HEAD_PITCH].lo_stop  = 0.0;
    rjoint[robotNum][HEAD_PITCH].hi_stop  = 0.0;
    rjoint[robotNum][HEAD_PITCH].fmax     = FMAX;
    rjoint[robotNum][HEAD_PITCH].link[0]  = NECK*(robotNum+1);
    rjoint[robotNum][HEAD_PITCH].link[1]  = HEAD*(robotNum+1);

    // left wrist joint (y axis)
    rjoint[robotNum][LEFT_WRIST_PITCH].type = HINGE;
    rjoint[robotNum][LEFT_WRIST_PITCH].px   = rlink[robotNum][LEFT_FORE_ARM].px;
    rjoint[robotNum][LEFT_WRIST_PITCH].py   = rlink[robotNum][LEFT_FORE_ARM].py;
    rjoint[robotNum][LEFT_WRIST_PITCH].pz   = rlink[robotNum][LEFT_FORE_ARM].pz - 0.5 * FORE_ARM_HEIGHT;
    rjoint[robotNum][LEFT_WRIST_PITCH].axis_x  = 0;
    rjoint[robotNum][LEFT_WRIST_PITCH].axis_y  = 1;
    rjoint[robotNum][LEFT_WRIST_PITCH].axis_z  = 0;
    rjoint[robotNum][LEFT_WRIST_PITCH].lo_stop  = 0.0;
    rjoint[robotNum][LEFT_WRIST_PITCH].hi_stop  = 0.0;
    rjoint[robotNum][LEFT_WRIST_PITCH].fmax     = FMAX;
    rjoint[robotNum][LEFT_WRIST_PITCH].link[0]  = LEFT_FORE_ARM*(robotNum+1);
    rjoint[robotNum][LEFT_WRIST_PITCH].link[1]  = LEFT_HAND*(robotNum+1);

    // left wrist joint (y axis)
    rjoint[robotNum][RIGHT_WRIST_PITCH].type =   rjoint[robotNum][LEFT_WRIST_PITCH].type;
    rjoint[robotNum][RIGHT_WRIST_PITCH].px   =   rjoint[robotNum][LEFT_WRIST_PITCH].px;
    rjoint[robotNum][RIGHT_WRIST_PITCH].py   = - rjoint[robotNum][LEFT_WRIST_PITCH].py;
    rjoint[robotNum][RIGHT_WRIST_PITCH].pz   =   rjoint[robotNum][LEFT_WRIST_PITCH].pz;
    rjoint[robotNum][RIGHT_WRIST_PITCH].axis_x  =   rjoint[robotNum][LEFT_WRIST_PITCH].axis_x;
    rjoint[robotNum][RIGHT_WRIST_PITCH].axis_y  =   rjoint[robotNum][LEFT_WRIST_PITCH].axis_y;
    rjoint[robotNum][RIGHT_WRIST_PITCH].axis_z  =   rjoint[robotNum][LEFT_WRIST_PITCH].axis_z;
    rjoint[robotNum][RIGHT_WRIST_PITCH].lo_stop  =   0.0;
    rjoint[robotNum][RIGHT_WRIST_PITCH].hi_stop  =   0.0;
    rjoint[robotNum][RIGHT_WRIST_PITCH].fmax     =   FMAX;
    rjoint[robotNum][RIGHT_WRIST_PITCH].link[0]  =   RIGHT_FORE_ARM*(robotNum+1);
    rjoint[robotNum][RIGHT_WRIST_PITCH].link[1]  =   RIGHT_HAND*(robotNum+1);

    // left shoulder (YAW: z, ROLL: x, PITCH: y)
    rjoint[robotNum][LEFT_SHOULDER_YAW].type =   HINGE;
    rjoint[robotNum][LEFT_SHOULDER_YAW].px   =   rlink[robotNum][LEFT_UPPER_ARM_DUMMY1].px;
    rjoint[robotNum][LEFT_SHOULDER_YAW].py   =   rlink[robotNum][LEFT_UPPER_ARM_DUMMY1].py - 0.5 * JOINT_SIZE;
    rjoint[robotNum][LEFT_SHOULDER_YAW].pz   =   rlink[robotNum][LEFT_UPPER_ARM_DUMMY1].pz;
    rjoint[robotNum][LEFT_SHOULDER_YAW].axis_x  =   0;
    rjoint[robotNum][LEFT_SHOULDER_YAW].axis_y  =   0;
    rjoint[robotNum][LEFT_SHOULDER_YAW].axis_z  =   1;
    rjoint[robotNum][LEFT_SHOULDER_YAW].lo_stop  = - 179 * (M_PI/180.0);
    rjoint[robotNum][LEFT_SHOULDER_YAW].hi_stop  =   179 * (M_PI/180.0);
    rjoint[robotNum][LEFT_SHOULDER_YAW].fmax     =   FMAX;
    rjoint[robotNum][LEFT_SHOULDER_YAW].link[0]  =   TORSO*(robotNum+1);
    rjoint[robotNum][LEFT_SHOULDER_YAW].link[1]  =   LEFT_UPPER_ARM_DUMMY1*(robotNum+1);
    rjoint[robotNum][LEFT_SHOULDER_ROLL].type    =   HINGE;
    rjoint[robotNum][LEFT_SHOULDER_ROLL].px      =   rlink[robotNum][LEFT_UPPER_ARM_DUMMY2].px;
    rjoint[robotNum][LEFT_SHOULDER_ROLL].py      =   rlink[robotNum][LEFT_UPPER_ARM_DUMMY2].py;
    rjoint[robotNum][LEFT_SHOULDER_ROLL].pz      =   rlink[robotNum][LEFT_UPPER_ARM_DUMMY2].pz - 0.5 * JOINT_SIZE;
    rjoint[robotNum][LEFT_SHOULDER_ROLL].axis_x  =   1;
    rjoint[robotNum][LEFT_SHOULDER_ROLL].axis_y  =   0;
    rjoint[robotNum][LEFT_SHOULDER_ROLL].axis_z  =   0;
    rjoint[robotNum][LEFT_SHOULDER_ROLL].lo_stop = - 179 * (M_PI/180.0);
    rjoint[robotNum][LEFT_SHOULDER_ROLL].hi_stop =   179 * (M_PI/180.0);
    rjoint[robotNum][LEFT_SHOULDER_ROLL].fmax    =   FMAX;
    rjoint[robotNum][LEFT_SHOULDER_ROLL].link[0] =   LEFT_UPPER_ARM_DUMMY1*(robotNum+1);
    rjoint[robotNum][LEFT_SHOULDER_ROLL].link[1] =   LEFT_UPPER_ARM_DUMMY2*(robotNum+1);
    rjoint[robotNum][LEFT_SHOULDER_PITCH].type   =   HINGE;
    rjoint[robotNum][LEFT_SHOULDER_PITCH].px     =   rlink[robotNum][LEFT_UPPER_ARM].px;
    rjoint[robotNum][LEFT_SHOULDER_PITCH].py     =   rlink[robotNum][LEFT_UPPER_ARM].py;
    rjoint[robotNum][LEFT_SHOULDER_PITCH].pz     =   rlink[robotNum][LEFT_UPPER_ARM].pz + 0.5 * UPPER_ARM_HEIGHT;
    rjoint[robotNum][LEFT_SHOULDER_PITCH].axis_x =   0;
    rjoint[robotNum][LEFT_SHOULDER_PITCH].axis_y =   1;
    rjoint[robotNum][LEFT_SHOULDER_PITCH].axis_z =   0;
    rjoint[robotNum][LEFT_SHOULDER_PITCH].lo_stop = - 179 * (M_PI/180.0);
    rjoint[robotNum][LEFT_SHOULDER_PITCH].hi_stop =   179 * (M_PI/180.0);
    rjoint[robotNum][LEFT_SHOULDER_PITCH].fmax    =   FMAX;
    rjoint[robotNum][LEFT_SHOULDER_PITCH].link[0] =   LEFT_UPPER_ARM_DUMMY2*(robotNum+1);
    rjoint[robotNum][LEFT_SHOULDER_PITCH].link[1] =   LEFT_UPPER_ARM*(robotNum+1);

    // right shoulder (YAW: z, ROLL: x, PITCH: y)
    rjoint[robotNum][RIGHT_SHOULDER_YAW].type =   rjoint[robotNum][LEFT_SHOULDER_YAW].type;
    rjoint[robotNum][RIGHT_SHOULDER_YAW].px   =   rjoint[robotNum][LEFT_SHOULDER_YAW].px;
    rjoint[robotNum][RIGHT_SHOULDER_YAW].py   = - rjoint[robotNum][LEFT_SHOULDER_YAW].py + 2.0 * STARTY; // change
    rjoint[robotNum][RIGHT_SHOULDER_YAW].pz   =   rjoint[robotNum][LEFT_SHOULDER_YAW].pz;
    rjoint[robotNum][RIGHT_SHOULDER_YAW].axis_x  =   rjoint[robotNum][LEFT_SHOULDER_YAW].axis_x;
    rjoint[robotNum][RIGHT_SHOULDER_YAW].axis_y  =   rjoint[robotNum][LEFT_SHOULDER_YAW].axis_y;
    rjoint[robotNum][RIGHT_SHOULDER_YAW].axis_z  =   rjoint[robotNum][LEFT_SHOULDER_YAW].axis_z;
    rjoint[robotNum][RIGHT_SHOULDER_YAW].lo_stop  =   rjoint[robotNum][LEFT_SHOULDER_YAW].lo_stop;
    rjoint[robotNum][RIGHT_SHOULDER_YAW].hi_stop  =   rjoint[robotNum][LEFT_SHOULDER_YAW].hi_stop;
    rjoint[robotNum][RIGHT_SHOULDER_YAW].fmax     =   rjoint[robotNum][LEFT_SHOULDER_YAW].fmax;
    rjoint[robotNum][RIGHT_SHOULDER_YAW].link[0]  =   TORSO*(robotNum+1);
    rjoint[robotNum][RIGHT_SHOULDER_YAW].link[1]  =   RIGHT_UPPER_ARM_DUMMY1*(robotNum+1);
    rjoint[robotNum][RIGHT_SHOULDER_ROLL].type    =   rjoint[robotNum][LEFT_SHOULDER_ROLL].type;
    rjoint[robotNum][RIGHT_SHOULDER_ROLL].px   =   rjoint[robotNum][LEFT_SHOULDER_ROLL].px;
    rjoint[robotNum][RIGHT_SHOULDER_ROLL].py   = - rjoint[robotNum][LEFT_SHOULDER_ROLL].py + 2.0 * STARTY; // change
    rjoint[robotNum][RIGHT_SHOULDER_ROLL].pz   =   rjoint[robotNum][LEFT_SHOULDER_ROLL].pz;
    rjoint[robotNum][RIGHT_SHOULDER_ROLL].axis_x  =   rjoint[robotNum][LEFT_SHOULDER_ROLL].axis_x;
    rjoint[robotNum][RIGHT_SHOULDER_ROLL].axis_y  =   rjoint[robotNum][LEFT_SHOULDER_ROLL].axis_y;
    rjoint[robotNum][RIGHT_SHOULDER_ROLL].axis_z  =   rjoint[robotNum][LEFT_SHOULDER_ROLL].axis_z;
    rjoint[robotNum][RIGHT_SHOULDER_ROLL].lo_stop  =   rjoint[robotNum][LEFT_SHOULDER_ROLL].lo_stop;
    rjoint[robotNum][RIGHT_SHOULDER_ROLL].hi_stop  =   rjoint[robotNum][LEFT_SHOULDER_ROLL].hi_stop;
    rjoint[robotNum][RIGHT_SHOULDER_ROLL].fmax     =   rjoint[robotNum][LEFT_SHOULDER_ROLL].fmax;
    rjoint[robotNum][RIGHT_SHOULDER_ROLL].link[0]  =   RIGHT_UPPER_ARM_DUMMY1*(robotNum+1);
    rjoint[robotNum][RIGHT_SHOULDER_ROLL].link[1]  =   RIGHT_UPPER_ARM_DUMMY2*(robotNum+1);
    rjoint[robotNum][RIGHT_SHOULDER_PITCH].type    =   rjoint[robotNum][LEFT_SHOULDER_PITCH].type;
    rjoint[robotNum][RIGHT_SHOULDER_PITCH].px   =   rjoint[robotNum][LEFT_SHOULDER_PITCH].px;
    rjoint[robotNum][RIGHT_SHOULDER_PITCH].py   = - rjoint[robotNum][LEFT_SHOULDER_PITCH].py + 2.0 * STARTY; // change
    rjoint[robotNum][RIGHT_SHOULDER_PITCH].pz   =   rjoint[robotNum][LEFT_SHOULDER_PITCH].pz;
    rjoint[robotNum][RIGHT_SHOULDER_PITCH].axis_x  =   rjoint[robotNum][LEFT_SHOULDER_PITCH].axis_x;
    rjoint[robotNum][RIGHT_SHOULDER_PITCH].axis_y  =   rjoint[robotNum][LEFT_SHOULDER_PITCH].axis_y;
    rjoint[robotNum][RIGHT_SHOULDER_PITCH].axis_z  =   rjoint[robotNum][LEFT_SHOULDER_PITCH].axis_z;
    rjoint[robotNum][RIGHT_SHOULDER_PITCH].lo_stop  =   rjoint[robotNum][LEFT_SHOULDER_PITCH].lo_stop;
    rjoint[robotNum][RIGHT_SHOULDER_PITCH].hi_stop  =   rjoint[robotNum][LEFT_SHOULDER_PITCH].hi_stop;
    rjoint[robotNum][RIGHT_SHOULDER_PITCH].fmax     =   rjoint[robotNum][LEFT_SHOULDER_PITCH].fmax;
    rjoint[robotNum][RIGHT_SHOULDER_PITCH].link[0]  =   RIGHT_UPPER_ARM_DUMMY2*(robotNum+1);
    rjoint[robotNum][RIGHT_SHOULDER_PITCH].link[1]  =   RIGHT_UPPER_ARM*(robotNum+1);

    // left hip joint (YAW: z, ROLL: x, PITCH: y)
    rjoint[robotNum][LEFT_HIP_YAW].type =   HINGE;
    rjoint[robotNum][LEFT_HIP_YAW].px   =   rlink[robotNum][LEFT_THIGH_DUMMY1].px;
    rjoint[robotNum][LEFT_HIP_YAW].py   =   rlink[robotNum][LEFT_THIGH_DUMMY1].py;
    rjoint[robotNum][LEFT_HIP_YAW].pz   =   rlink[robotNum][LEFT_THIGH_DUMMY1].pz + 0.5 * JOINT_SIZE;
    rjoint[robotNum][LEFT_HIP_YAW].axis_x  =   0;
    rjoint[robotNum][LEFT_HIP_YAW].axis_y  =   0;
    rjoint[robotNum][LEFT_HIP_YAW].axis_z  =   1;
    rjoint[robotNum][LEFT_HIP_YAW].lo_stop  = - 90 * (M_PI/180.0);
    rjoint[robotNum][LEFT_HIP_YAW].hi_stop  =   90 * (M_PI/180.0);
    rjoint[robotNum][LEFT_HIP_YAW].fmax     =   FMAX;
    rjoint[robotNum][LEFT_HIP_YAW].link[0]  =   TORSO*(robotNum+1);
    rjoint[robotNum][LEFT_HIP_YAW].link[1]  =   LEFT_THIGH_DUMMY1*(robotNum+1);
    rjoint[robotNum][LEFT_HIP_ROLL].type    =   HINGE;
    rjoint[robotNum][LEFT_HIP_ROLL].px   =   rlink[robotNum][LEFT_THIGH_DUMMY2].px - 0.5 * JOINT_SIZE;
    rjoint[robotNum][LEFT_HIP_ROLL].py   =   rlink[robotNum][LEFT_THIGH_DUMMY2].py;
    rjoint[robotNum][LEFT_HIP_ROLL].pz   =   rlink[robotNum][LEFT_THIGH_DUMMY2].pz;
    rjoint[robotNum][LEFT_HIP_ROLL].axis_x  =   1;
    rjoint[robotNum][LEFT_HIP_ROLL].axis_y  =   0;
    rjoint[robotNum][LEFT_HIP_ROLL].axis_z  =   0;
    rjoint[robotNum][LEFT_HIP_ROLL].lo_stop  = - 90 * (M_PI/180.0);
    rjoint[robotNum][LEFT_HIP_ROLL].hi_stop  =   90 * (M_PI/180.0);
    rjoint[robotNum][LEFT_HIP_ROLL].fmax     =   FMAX;
    rjoint[robotNum][LEFT_HIP_ROLL].link[0]  =   LEFT_THIGH_DUMMY1*(robotNum+1);
    rjoint[robotNum][LEFT_HIP_ROLL].link[1]  =   LEFT_THIGH_DUMMY2*(robotNum+1);
    rjoint[robotNum][LEFT_HIP_PITCH].type    =   HINGE;
    rjoint[robotNum][LEFT_HIP_PITCH].px   =   rlink[robotNum][LEFT_THIGH].px + 0.5 * JOINT_SIZE;
    rjoint[robotNum][LEFT_HIP_PITCH].py   =   rlink[robotNum][LEFT_THIGH].py;
    rjoint[robotNum][LEFT_HIP_PITCH].pz   =   rlink[robotNum][LEFT_THIGH].pz + 0.5 * THIGH_HEIGHT;
    rjoint[robotNum][LEFT_HIP_PITCH].axis_x  =   0;
    rjoint[robotNum][LEFT_HIP_PITCH].axis_y  =   1;
    rjoint[robotNum][LEFT_HIP_PITCH].axis_z  =   0;
    rjoint[robotNum][LEFT_HIP_PITCH].lo_stop  = - 90 * (M_PI/180.0);
    rjoint[robotNum][LEFT_HIP_PITCH].hi_stop  =   90 * (M_PI/180.0);
    rjoint[robotNum][LEFT_HIP_PITCH].fmax     =   FMAX;
    rjoint[robotNum][LEFT_HIP_PITCH].link[0]  =   LEFT_THIGH_DUMMY2*(robotNum+1);
    rjoint[robotNum][LEFT_HIP_PITCH].link[1]  =   LEFT_THIGH*(robotNum+1);

    // right hip joint (JT1: roll, JT2: yaw, JT3: pitch)
    rjoint[robotNum][RIGHT_HIP_YAW].type =   rjoint[robotNum][LEFT_HIP_YAW].type;
    rjoint[robotNum][RIGHT_HIP_YAW].px   =   rjoint[robotNum][LEFT_HIP_YAW].px;
    rjoint[robotNum][RIGHT_HIP_YAW].py   = - rjoint[robotNum][LEFT_HIP_YAW].py + 2.0 * STARTY; // change
    rjoint[robotNum][RIGHT_HIP_YAW].pz   =   rjoint[robotNum][LEFT_HIP_YAW].pz;
    rjoint[robotNum][RIGHT_HIP_YAW].axis_x  =   rjoint[robotNum][LEFT_HIP_YAW].axis_x;
    rjoint[robotNum][RIGHT_HIP_YAW].axis_y  =   rjoint[robotNum][LEFT_HIP_YAW].axis_y;
    rjoint[robotNum][RIGHT_HIP_YAW].axis_z  =   rjoint[robotNum][LEFT_HIP_YAW].axis_z;
    rjoint[robotNum][RIGHT_HIP_YAW].lo_stop  =   rjoint[robotNum][LEFT_HIP_YAW].lo_stop;
    rjoint[robotNum][RIGHT_HIP_YAW].hi_stop  =   rjoint[robotNum][LEFT_HIP_YAW].hi_stop;
    rjoint[robotNum][RIGHT_HIP_YAW].fmax     =   rjoint[robotNum][LEFT_HIP_YAW].fmax;
    rjoint[robotNum][RIGHT_HIP_YAW].link[0]  =   TORSO*(robotNum+1);
    rjoint[robotNum][RIGHT_HIP_YAW].link[1]  =   RIGHT_THIGH_DUMMY1*(robotNum+1);
    rjoint[robotNum][RIGHT_HIP_ROLL].type    =   rjoint[robotNum][LEFT_HIP_ROLL].type;
    rjoint[robotNum][RIGHT_HIP_ROLL].px   =   rjoint[robotNum][LEFT_HIP_ROLL].px;
    rjoint[robotNum][RIGHT_HIP_ROLL].py   = - rjoint[robotNum][LEFT_HIP_ROLL].py + 2.0 * STARTY; // change
    rjoint[robotNum][RIGHT_HIP_ROLL].pz   =   rjoint[robotNum][LEFT_HIP_ROLL].pz;
    rjoint[robotNum][RIGHT_HIP_ROLL].axis_x  =   rjoint[robotNum][LEFT_HIP_ROLL].axis_x;
    rjoint[robotNum][RIGHT_HIP_ROLL].axis_y  =   rjoint[robotNum][LEFT_HIP_ROLL].axis_y;
    rjoint[robotNum][RIGHT_HIP_ROLL].axis_z  =   rjoint[robotNum][LEFT_HIP_ROLL].axis_z;
    rjoint[robotNum][RIGHT_HIP_ROLL].lo_stop  =   rjoint[robotNum][LEFT_HIP_ROLL].lo_stop;
    rjoint[robotNum][RIGHT_HIP_ROLL].hi_stop  =   rjoint[robotNum][LEFT_HIP_ROLL].hi_stop;
    rjoint[robotNum][RIGHT_HIP_ROLL].fmax     =   rjoint[robotNum][LEFT_HIP_ROLL].fmax;
    rjoint[robotNum][RIGHT_HIP_ROLL].link[0]  =   RIGHT_THIGH_DUMMY1*(robotNum+1);
    rjoint[robotNum][RIGHT_HIP_ROLL].link[1]  =   RIGHT_THIGH_DUMMY2*(robotNum+1);
    rjoint[robotNum][RIGHT_HIP_PITCH].type    =   rjoint[robotNum][LEFT_HIP_PITCH].type;
    rjoint[robotNum][RIGHT_HIP_PITCH].px   =   rjoint[robotNum][LEFT_HIP_PITCH].px;
    rjoint[robotNum][RIGHT_HIP_PITCH].py   = - rjoint[robotNum][LEFT_HIP_PITCH].py + 2.0 * STARTY; // change
    rjoint[robotNum][RIGHT_HIP_PITCH].pz   =   rjoint[robotNum][LEFT_HIP_PITCH].pz;
    rjoint[robotNum][RIGHT_HIP_PITCH].axis_x  =   rjoint[robotNum][LEFT_HIP_PITCH].axis_x;
    rjoint[robotNum][RIGHT_HIP_PITCH].axis_y  =   rjoint[robotNum][LEFT_HIP_PITCH].axis_y;
    rjoint[robotNum][RIGHT_HIP_PITCH].axis_z  =   rjoint[robotNum][LEFT_HIP_PITCH].axis_z;
    rjoint[robotNum][RIGHT_HIP_PITCH].lo_stop  =   rjoint[robotNum][LEFT_HIP_PITCH].lo_stop;
    rjoint[robotNum][RIGHT_HIP_PITCH].hi_stop  =   rjoint[robotNum][LEFT_HIP_PITCH].hi_stop;
    rjoint[robotNum][RIGHT_HIP_PITCH].fmax     =   rjoint[robotNum][LEFT_HIP_PITCH].fmax;
    rjoint[robotNum][RIGHT_HIP_PITCH].link[0]  =   RIGHT_THIGH_DUMMY2*(robotNum+1);
    rjoint[robotNum][RIGHT_HIP_PITCH].link[1]  =   RIGHT_THIGH*(robotNum+1);

    // left elbow joint (pitch)
    rjoint[robotNum][LEFT_ELBOW_PITCH].type =   HINGE;
    rjoint[robotNum][LEFT_ELBOW_PITCH].px   =   rlink[robotNum][LEFT_UPPER_ARM].px;
    rjoint[robotNum][LEFT_ELBOW_PITCH].py   =   rlink[robotNum][LEFT_UPPER_ARM].py;
    rjoint[robotNum][LEFT_ELBOW_PITCH].pz   =   rlink[robotNum][LEFT_UPPER_ARM].pz - 0.5 * UPPER_ARM_HEIGHT;
    rjoint[robotNum][LEFT_ELBOW_PITCH].axis_x   =   0;
    rjoint[robotNum][LEFT_ELBOW_PITCH].axis_y   =   1;
    rjoint[robotNum][LEFT_ELBOW_PITCH].axis_z   =   0;
    rjoint[robotNum][LEFT_ELBOW_PITCH].lo_stop  = - 179 * (M_PI/180.0);
    rjoint[robotNum][LEFT_ELBOW_PITCH].hi_stop  =   0;
    rjoint[robotNum][LEFT_ELBOW_PITCH].fmax     =   FMAX;
    rjoint[robotNum][LEFT_ELBOW_PITCH].link[0]  =   LEFT_UPPER_ARM*(robotNum+1);
    rjoint[robotNum][LEFT_ELBOW_PITCH].link[1]  =   LEFT_FORE_ARM*(robotNum+1);
    rjoint[robotNum][LEFT_ELBOW_PITCH].fudge_factor = 0.1;

    // right elbow joint (pitch)
    rjoint[robotNum][RIGHT_ELBOW_PITCH].type =   rjoint[robotNum][LEFT_ELBOW_PITCH].type;
    rjoint[robotNum][RIGHT_ELBOW_PITCH].px   =   rjoint[robotNum][LEFT_ELBOW_PITCH].px;
    rjoint[robotNum][RIGHT_ELBOW_PITCH].py   = - rjoint[robotNum][LEFT_ELBOW_PITCH].py + 2.0 * STARTY; // change
    rjoint[robotNum][RIGHT_ELBOW_PITCH].pz   =   rjoint[robotNum][LEFT_ELBOW_PITCH].pz;
    rjoint[robotNum][RIGHT_ELBOW_PITCH].axis_x   =   rjoint[robotNum][LEFT_ELBOW_PITCH].axis_x;
    rjoint[robotNum][RIGHT_ELBOW_PITCH].axis_y   =   rjoint[robotNum][LEFT_ELBOW_PITCH].axis_y;
    rjoint[robotNum][RIGHT_ELBOW_PITCH].axis_z   =   rjoint[robotNum][LEFT_ELBOW_PITCH].axis_z;
    rjoint[robotNum][RIGHT_ELBOW_PITCH].lo_stop  =   rjoint[robotNum][LEFT_ELBOW_PITCH].lo_stop;
    rjoint[robotNum][RIGHT_ELBOW_PITCH].hi_stop  =   rjoint[robotNum][LEFT_ELBOW_PITCH].hi_stop;
    rjoint[robotNum][RIGHT_ELBOW_PITCH].fmax     =   rjoint[robotNum][LEFT_ELBOW_PITCH].fmax;
    rjoint[robotNum][RIGHT_ELBOW_PITCH].link[0]  =   RIGHT_UPPER_ARM*(robotNum+1);
    rjoint[robotNum][RIGHT_ELBOW_PITCH].link[1]  =   RIGHT_FORE_ARM*(robotNum+1);
    rjoint[robotNum][RIGHT_ELBOW_PITCH].fudge_factor = rjoint[robotNum][LEFT_ELBOW_PITCH].fudge_factor;

    // left knee joint
    rjoint[robotNum][LEFT_KNEE_PITCH].type =   HINGE;
    rjoint[robotNum][LEFT_KNEE_PITCH].px   =   rlink[robotNum][LEFT_THIGH].px;
    rjoint[robotNum][LEFT_KNEE_PITCH].py   =   rlink[robotNum][LEFT_THIGH].py;
    rjoint[robotNum][LEFT_KNEE_PITCH].pz   =   rlink[robotNum][LEFT_THIGH].pz - 0.5 * THIGH_HEIGHT;
    rjoint[robotNum][LEFT_KNEE_PITCH].axis_x  =   0;
    rjoint[robotNum][LEFT_KNEE_PITCH].axis_y  =   1;
    rjoint[robotNum][LEFT_KNEE_PITCH].axis_z  =   0;
    rjoint[robotNum][LEFT_KNEE_PITCH].lo_stop =   0;  //  可動域 最小 max -pi
    rjoint[robotNum][LEFT_KNEE_PITCH].hi_stop =   170.0 * (M_PI/180.0);  // 可動域 最大 max +pi
    rjoint[robotNum][LEFT_KNEE_PITCH].fmax    =   FMAX;
    rjoint[robotNum][LEFT_KNEE_PITCH].link[0] =   LEFT_THIGH*(robotNum+1);
    rjoint[robotNum][LEFT_KNEE_PITCH].link[1] =   LEFT_CALF*(robotNum+1);
    rjoint[robotNum][LEFT_KNEE_PITCH].fudge_factor = 0.9;

    // right knee joint
    rjoint[robotNum][RIGHT_KNEE_PITCH].type =   rjoint[robotNum][LEFT_KNEE_PITCH].type;
    rjoint[robotNum][RIGHT_KNEE_PITCH].px   =   rjoint[robotNum][LEFT_KNEE_PITCH].px;
    rjoint[robotNum][RIGHT_KNEE_PITCH].py   = - rjoint[robotNum][LEFT_KNEE_PITCH].py + 2.0 * STARTY; // change
    rjoint[robotNum][RIGHT_KNEE_PITCH].pz   =   rjoint[robotNum][LEFT_KNEE_PITCH].pz;
    rjoint[robotNum][RIGHT_KNEE_PITCH].axis_x   =   rjoint[robotNum][LEFT_KNEE_PITCH].axis_x;
    rjoint[robotNum][RIGHT_KNEE_PITCH].axis_y   =   rjoint[robotNum][LEFT_KNEE_PITCH].axis_y;
    rjoint[robotNum][RIGHT_KNEE_PITCH].axis_z   =   rjoint[robotNum][LEFT_KNEE_PITCH].axis_z;
    rjoint[robotNum][RIGHT_KNEE_PITCH].lo_stop  =   rjoint[robotNum][LEFT_KNEE_PITCH].lo_stop;
    rjoint[robotNum][RIGHT_KNEE_PITCH].hi_stop  =   rjoint[robotNum][LEFT_KNEE_PITCH].hi_stop;
    rjoint[robotNum][RIGHT_KNEE_PITCH].fmax     =   rjoint[robotNum][LEFT_KNEE_PITCH].fmax;
    rjoint[robotNum][RIGHT_KNEE_PITCH].link[0]  =   RIGHT_THIGH*(robotNum+1);
    rjoint[robotNum][RIGHT_KNEE_PITCH].link[1]  =   RIGHT_CALF*(robotNum+1);
    rjoint[robotNum][RIGHT_KNEE_PITCH].fudge_factor = rjoint[robotNum][RIGHT_KNEE_PITCH].fudge_factor;

    // left foot joint (pitch: y axis, roll: x axis)
    rjoint[robotNum][LEFT_FOOT_PITCH].type    =   HINGE; //HINGE
    rjoint[robotNum][LEFT_FOOT_PITCH].px      =   rlink[robotNum][LEFT_CALF].px;
    rjoint[robotNum][LEFT_FOOT_PITCH].py      =   rlink[robotNum][LEFT_CALF].py;
    rjoint[robotNum][LEFT_FOOT_PITCH].pz      =   rlink[robotNum][LEFT_CALF].pz - 0.5 * CALF_LENGTH;
    rjoint[robotNum][LEFT_FOOT_PITCH].axis_x  =   0;
    rjoint[robotNum][LEFT_FOOT_PITCH].axis_y  =   1;
    rjoint[robotNum][LEFT_FOOT_PITCH].axis_z  =   0;
    rjoint[robotNum][LEFT_FOOT_PITCH].lo_stop = - 90.0 * (M_PI/180.0);
    rjoint[robotNum][LEFT_FOOT_PITCH].hi_stop =   90.0 * (M_PI/180.0);
    rjoint[robotNum][LEFT_FOOT_PITCH].fmax    =   FMAX;
    rjoint[robotNum][LEFT_FOOT_PITCH].link[0] =   LEFT_CALF*(robotNum+1);
    rjoint[robotNum][LEFT_FOOT_PITCH].link[1] =   LEFT_FOOT_DUMMY*(robotNum+1);
    rjoint[robotNum][LEFT_FOOT_ROLL].type     =   HINGE;
    rjoint[robotNum][LEFT_FOOT_ROLL].px       =   rlink[robotNum][LEFT_CALF].px;
    rjoint[robotNum][LEFT_FOOT_ROLL].py       =   rlink[robotNum][LEFT_CALF].py;
    rjoint[robotNum][LEFT_FOOT_ROLL].pz       =   rlink[robotNum][LEFT_FOOT_DUMMY].pz - 0.5 * (JOINT_SIZE);
    rjoint[robotNum][LEFT_FOOT_ROLL].axis_x   =   1;  // ピッチ
    rjoint[robotNum][LEFT_FOOT_ROLL].axis_y   =   0;
    rjoint[robotNum][LEFT_FOOT_ROLL].axis_z   =   0;
    rjoint[robotNum][LEFT_FOOT_ROLL].lo_stop  = - 90.0 * (M_PI/180.0); // 90.0
    rjoint[robotNum][LEFT_FOOT_ROLL].hi_stop  =   90.0 * (M_PI/180.0);
    rjoint[robotNum][LEFT_FOOT_ROLL].fmax     =   FMAX;
    rjoint[robotNum][LEFT_FOOT_ROLL].link[0]  =   LEFT_FOOT_DUMMY*(robotNum+1);
    rjoint[robotNum][LEFT_FOOT_ROLL].link[1]  =   LEFT_FOOT*(robotNum+1);

    // right foot joint
    rjoint[robotNum][RIGHT_FOOT_PITCH].type    =   rjoint[robotNum][LEFT_FOOT_PITCH].type;
    rjoint[robotNum][RIGHT_FOOT_PITCH].px      =   rjoint[robotNum][LEFT_FOOT_PITCH].px;
    rjoint[robotNum][RIGHT_FOOT_PITCH].py      = - rjoint[robotNum][LEFT_FOOT_PITCH].py + 2.0 * STARTY; // change
    rjoint[robotNum][RIGHT_FOOT_PITCH].pz      =   rjoint[robotNum][LEFT_FOOT_PITCH].pz;
    rjoint[robotNum][RIGHT_FOOT_PITCH].axis_x  =   rjoint[robotNum][LEFT_FOOT_PITCH].axis_x;
    rjoint[robotNum][RIGHT_FOOT_PITCH].axis_y  =   rjoint[robotNum][LEFT_FOOT_PITCH].axis_y;
    rjoint[robotNum][RIGHT_FOOT_PITCH].axis_z  =   rjoint[robotNum][LEFT_FOOT_PITCH].axis_z;
    rjoint[robotNum][RIGHT_FOOT_PITCH].lo_stop =   rjoint[robotNum][LEFT_FOOT_PITCH].lo_stop;
    rjoint[robotNum][RIGHT_FOOT_PITCH].hi_stop =   rjoint[robotNum][LEFT_FOOT_PITCH].hi_stop;
    rjoint[robotNum][RIGHT_FOOT_PITCH].fmax    =   rjoint[robotNum][LEFT_FOOT_PITCH].fmax;
    rjoint[robotNum][RIGHT_FOOT_PITCH].link[0] =   RIGHT_CALF*(robotNum+1);
    rjoint[robotNum][RIGHT_FOOT_PITCH].link[1] =   RIGHT_FOOT_DUMMY*(robotNum+1);
    rjoint[robotNum][RIGHT_FOOT_ROLL].type     =   rjoint[robotNum][LEFT_FOOT_ROLL].type;
    rjoint[robotNum][RIGHT_FOOT_ROLL].px       =   rjoint[robotNum][LEFT_FOOT_ROLL].px;
    rjoint[robotNum][RIGHT_FOOT_ROLL].py       = - rjoint[robotNum][LEFT_FOOT_ROLL].py + 2.0 * STARTY; // chnage
    rjoint[robotNum][RIGHT_FOOT_ROLL].pz       =   rjoint[robotNum][LEFT_FOOT_ROLL].pz;
    rjoint[robotNum][RIGHT_FOOT_ROLL].axis_x   =   rjoint[robotNum][LEFT_FOOT_ROLL].axis_x;
    rjoint[robotNum][RIGHT_FOOT_ROLL].axis_y   =   rjoint[robotNum][LEFT_FOOT_ROLL].axis_y;
    rjoint[robotNum][RIGHT_FOOT_ROLL].axis_z   =   rjoint[robotNum][LEFT_FOOT_ROLL].axis_z;
    rjoint[robotNum][RIGHT_FOOT_ROLL].lo_stop  =   rjoint[robotNum][LEFT_FOOT_ROLL].hi_stop;
    rjoint[robotNum][RIGHT_FOOT_ROLL].hi_stop  =   rjoint[robotNum][LEFT_FOOT_ROLL].lo_stop;
    rjoint[robotNum][RIGHT_FOOT_ROLL].fmax     =   rjoint[robotNum][LEFT_FOOT_ROLL].fmax;
    rjoint[robotNum][RIGHT_FOOT_ROLL].link[0]  =   RIGHT_FOOT_DUMMY*(robotNum+1);
    rjoint[robotNum][RIGHT_FOOT_ROLL].link[1]  =   RIGHT_FOOT*(robotNum+1);
}

void readJointParam(int robotNum)
{
    // neck joint (y axis)
    rjoint[robotNum][NECK_PITCH].type = HINGE;
    rjoint[robotNum][NECK_PITCH].px   = rlink[robotNum][NECK].px;
    rjoint[robotNum][NECK_PITCH].py   = rlink[robotNum][NECK].py;
    rjoint[robotNum][NECK_PITCH].pz   = rlink[robotNum][NECK].pz - 0.5 * NECK_HEIGHT;
    rjoint[robotNum][NECK_PITCH].axis_x  = 0;
    rjoint[robotNum][NECK_PITCH].axis_y  = 1;
    rjoint[robotNum][NECK_PITCH].axis_z  = 0;
    rjoint[robotNum][NECK_PITCH].lo_stop  = 0;
    rjoint[robotNum][NECK_PITCH].hi_stop  = 1;
    rjoint[robotNum][NECK_PITCH].fmax     = FMAX;
    rjoint[robotNum][NECK_PITCH].link[0]  = TORSO;
    rjoint[robotNum][NECK_PITCH].link[1]  = NECK;

    // head joint (y axis)
    rjoint[robotNum][HEAD_PITCH].type = HINGE;
    rjoint[robotNum][HEAD_PITCH].px   = rlink[robotNum][HEAD].px;
    rjoint[robotNum][HEAD_PITCH].py   = rlink[robotNum][HEAD].py;
    rjoint[robotNum][HEAD_PITCH].pz   = rlink[robotNum][HEAD].pz - HEAD_RADIUS;
    rjoint[robotNum][HEAD_PITCH].axis_x  = 0;
    rjoint[robotNum][HEAD_PITCH].axis_y  = 1;
    rjoint[robotNum][HEAD_PITCH].axis_z  = 0;
    rjoint[robotNum][HEAD_PITCH].lo_stop  = 0.0;
    rjoint[robotNum][HEAD_PITCH].hi_stop  = 0.0;
    rjoint[robotNum][HEAD_PITCH].fmax     = FMAX;
    rjoint[robotNum][HEAD_PITCH].link[0]  = NECK;
    rjoint[robotNum][HEAD_PITCH].link[1]  = HEAD;

    // left wrist joint (y axis)
    rjoint[robotNum][LEFT_WRIST_PITCH].type = HINGE;
    rjoint[robotNum][LEFT_WRIST_PITCH].px   = rlink[robotNum][LEFT_FORE_ARM].px;
    rjoint[robotNum][LEFT_WRIST_PITCH].py   = rlink[robotNum][LEFT_FORE_ARM].py;
    rjoint[robotNum][LEFT_WRIST_PITCH].pz   = rlink[robotNum][LEFT_FORE_ARM].pz - 0.5 * FORE_ARM_HEIGHT;
    rjoint[robotNum][LEFT_WRIST_PITCH].axis_x  = 0;
    rjoint[robotNum][LEFT_WRIST_PITCH].axis_y  = 1;
    rjoint[robotNum][LEFT_WRIST_PITCH].axis_z  = 0;
    rjoint[robotNum][LEFT_WRIST_PITCH].lo_stop  = 0.0;
    rjoint[robotNum][LEFT_WRIST_PITCH].hi_stop  = 0.0;
    rjoint[robotNum][LEFT_WRIST_PITCH].fmax     = FMAX;
    rjoint[robotNum][LEFT_WRIST_PITCH].link[0]  = LEFT_FORE_ARM;
    rjoint[robotNum][LEFT_WRIST_PITCH].link[1]  = LEFT_HAND;

    // left wrist joint (y axis)
    rjoint[robotNum][RIGHT_WRIST_PITCH].type =   rjoint[robotNum][LEFT_WRIST_PITCH].type;
    rjoint[robotNum][RIGHT_WRIST_PITCH].px   =   rjoint[robotNum][LEFT_WRIST_PITCH].px;
    rjoint[robotNum][RIGHT_WRIST_PITCH].py   = - rjoint[robotNum][LEFT_WRIST_PITCH].py;
    rjoint[robotNum][RIGHT_WRIST_PITCH].pz   =   rjoint[robotNum][LEFT_WRIST_PITCH].pz;
    rjoint[robotNum][RIGHT_WRIST_PITCH].axis_x  =   rjoint[robotNum][LEFT_WRIST_PITCH].axis_x;
    rjoint[robotNum][RIGHT_WRIST_PITCH].axis_y  =   rjoint[robotNum][LEFT_WRIST_PITCH].axis_y;
    rjoint[robotNum][RIGHT_WRIST_PITCH].axis_z  =   rjoint[robotNum][LEFT_WRIST_PITCH].axis_z;
    rjoint[robotNum][RIGHT_WRIST_PITCH].lo_stop  =   0.0;
    rjoint[robotNum][RIGHT_WRIST_PITCH].hi_stop  =   0.0;
    rjoint[robotNum][RIGHT_WRIST_PITCH].fmax     =   FMAX;
    rjoint[robotNum][RIGHT_WRIST_PITCH].link[0]  =   RIGHT_FORE_ARM;
    rjoint[robotNum][RIGHT_WRIST_PITCH].link[1]  =   RIGHT_HAND;

    // left shoulder (YAW: z, ROLL: x, PITCH: y)
    rjoint[robotNum][LEFT_SHOULDER_YAW].type =   HINGE;
    rjoint[robotNum][LEFT_SHOULDER_YAW].px   =   rlink[robotNum][LEFT_UPPER_ARM_DUMMY1].px;
    rjoint[robotNum][LEFT_SHOULDER_YAW].py   =   rlink[robotNum][LEFT_UPPER_ARM_DUMMY1].py - 0.5 * JOINT_SIZE;
    rjoint[robotNum][LEFT_SHOULDER_YAW].pz   =   rlink[robotNum][LEFT_UPPER_ARM_DUMMY1].pz;
    rjoint[robotNum][LEFT_SHOULDER_YAW].axis_x  =   0;
    rjoint[robotNum][LEFT_SHOULDER_YAW].axis_y  =   0;
    rjoint[robotNum][LEFT_SHOULDER_YAW].axis_z  =   1;
    rjoint[robotNum][LEFT_SHOULDER_YAW].lo_stop  = - 179 * (M_PI/180.0);
    rjoint[robotNum][LEFT_SHOULDER_YAW].hi_stop  =   179 * (M_PI/180.0);
    rjoint[robotNum][LEFT_SHOULDER_YAW].fmax     =   FMAX;
    rjoint[robotNum][LEFT_SHOULDER_YAW].link[0]  =   TORSO;
    rjoint[robotNum][LEFT_SHOULDER_YAW].link[1]  =   LEFT_UPPER_ARM_DUMMY1;
    rjoint[robotNum][LEFT_SHOULDER_ROLL].type    =   HINGE;
    rjoint[robotNum][LEFT_SHOULDER_ROLL].px      =   rlink[robotNum][LEFT_UPPER_ARM_DUMMY2].px;
    rjoint[robotNum][LEFT_SHOULDER_ROLL].py      =   rlink[robotNum][LEFT_UPPER_ARM_DUMMY2].py;
    rjoint[robotNum][LEFT_SHOULDER_ROLL].pz      =   rlink[robotNum][LEFT_UPPER_ARM_DUMMY2].pz - 0.5 * JOINT_SIZE;
    rjoint[robotNum][LEFT_SHOULDER_ROLL].axis_x  =   1;
    rjoint[robotNum][LEFT_SHOULDER_ROLL].axis_y  =   0;
    rjoint[robotNum][LEFT_SHOULDER_ROLL].axis_z  =   0;
    rjoint[robotNum][LEFT_SHOULDER_ROLL].lo_stop = - 179 * (M_PI/180.0);
    rjoint[robotNum][LEFT_SHOULDER_ROLL].hi_stop =   179 * (M_PI/180.0);
    rjoint[robotNum][LEFT_SHOULDER_ROLL].fmax    =   FMAX;
    rjoint[robotNum][LEFT_SHOULDER_ROLL].link[0] =   LEFT_UPPER_ARM_DUMMY1;
    rjoint[robotNum][LEFT_SHOULDER_ROLL].link[1] =   LEFT_UPPER_ARM_DUMMY2;
    rjoint[robotNum][LEFT_SHOULDER_PITCH].type   =   HINGE;
    rjoint[robotNum][LEFT_SHOULDER_PITCH].px     =   rlink[robotNum][LEFT_UPPER_ARM].px;
    rjoint[robotNum][LEFT_SHOULDER_PITCH].py     =   rlink[robotNum][LEFT_UPPER_ARM].py;
    rjoint[robotNum][LEFT_SHOULDER_PITCH].pz     =   rlink[robotNum][LEFT_UPPER_ARM].pz + 0.5 * UPPER_ARM_HEIGHT;
    rjoint[robotNum][LEFT_SHOULDER_PITCH].axis_x =   0;
    rjoint[robotNum][LEFT_SHOULDER_PITCH].axis_y =   1;
    rjoint[robotNum][LEFT_SHOULDER_PITCH].axis_z =   0;
    rjoint[robotNum][LEFT_SHOULDER_PITCH].lo_stop = - 179 * (M_PI/180.0);
    rjoint[robotNum][LEFT_SHOULDER_PITCH].hi_stop =   179 * (M_PI/180.0);
    rjoint[robotNum][LEFT_SHOULDER_PITCH].fmax    =   FMAX;
    rjoint[robotNum][LEFT_SHOULDER_PITCH].link[0] =   LEFT_UPPER_ARM_DUMMY2;
    rjoint[robotNum][LEFT_SHOULDER_PITCH].link[1] =   LEFT_UPPER_ARM;

    // right shoulder (YAW: z, ROLL: x, PITCH: y)
    rjoint[robotNum][RIGHT_SHOULDER_YAW].type =   rjoint[robotNum][LEFT_SHOULDER_YAW].type;
    rjoint[robotNum][RIGHT_SHOULDER_YAW].px   =   rjoint[robotNum][LEFT_SHOULDER_YAW].px;
    rjoint[robotNum][RIGHT_SHOULDER_YAW].py   = - rjoint[robotNum][LEFT_SHOULDER_YAW].py + 2.0 * STARTY; // change
    rjoint[robotNum][RIGHT_SHOULDER_YAW].pz   =   rjoint[robotNum][LEFT_SHOULDER_YAW].pz;
    rjoint[robotNum][RIGHT_SHOULDER_YAW].axis_x  =   rjoint[robotNum][LEFT_SHOULDER_YAW].axis_x;
    rjoint[robotNum][RIGHT_SHOULDER_YAW].axis_y  =   rjoint[robotNum][LEFT_SHOULDER_YAW].axis_y;
    rjoint[robotNum][RIGHT_SHOULDER_YAW].axis_z  =   rjoint[robotNum][LEFT_SHOULDER_YAW].axis_z;
    rjoint[robotNum][RIGHT_SHOULDER_YAW].lo_stop  =   rjoint[robotNum][LEFT_SHOULDER_YAW].lo_stop;
    rjoint[robotNum][RIGHT_SHOULDER_YAW].hi_stop  =   rjoint[robotNum][LEFT_SHOULDER_YAW].hi_stop;
    rjoint[robotNum][RIGHT_SHOULDER_YAW].fmax     =   rjoint[robotNum][LEFT_SHOULDER_YAW].fmax;
    rjoint[robotNum][RIGHT_SHOULDER_YAW].link[0]  =   TORSO;
    rjoint[robotNum][RIGHT_SHOULDER_YAW].link[1]  =   RIGHT_UPPER_ARM_DUMMY1;
    rjoint[robotNum][RIGHT_SHOULDER_ROLL].type    =   rjoint[robotNum][LEFT_SHOULDER_ROLL].type;
    rjoint[robotNum][RIGHT_SHOULDER_ROLL].px   =   rjoint[robotNum][LEFT_SHOULDER_ROLL].px;
    rjoint[robotNum][RIGHT_SHOULDER_ROLL].py   = - rjoint[robotNum][LEFT_SHOULDER_ROLL].py + 2.0 * STARTY; // change
    rjoint[robotNum][RIGHT_SHOULDER_ROLL].pz   =   rjoint[robotNum][LEFT_SHOULDER_ROLL].pz;
    rjoint[robotNum][RIGHT_SHOULDER_ROLL].axis_x  =   rjoint[robotNum][LEFT_SHOULDER_ROLL].axis_x;
    rjoint[robotNum][RIGHT_SHOULDER_ROLL].axis_y  =   rjoint[robotNum][LEFT_SHOULDER_ROLL].axis_y;
    rjoint[robotNum][RIGHT_SHOULDER_ROLL].axis_z  =   rjoint[robotNum][LEFT_SHOULDER_ROLL].axis_z;
    rjoint[robotNum][RIGHT_SHOULDER_ROLL].lo_stop  =   rjoint[robotNum][LEFT_SHOULDER_ROLL].lo_stop;
    rjoint[robotNum][RIGHT_SHOULDER_ROLL].hi_stop  =   rjoint[robotNum][LEFT_SHOULDER_ROLL].hi_stop;
    rjoint[robotNum][RIGHT_SHOULDER_ROLL].fmax     =   rjoint[robotNum][LEFT_SHOULDER_ROLL].fmax;
    rjoint[robotNum][RIGHT_SHOULDER_ROLL].link[0]  =   RIGHT_UPPER_ARM_DUMMY1;
    rjoint[robotNum][RIGHT_SHOULDER_ROLL].link[1]  =   RIGHT_UPPER_ARM_DUMMY2;
    rjoint[robotNum][RIGHT_SHOULDER_PITCH].type    =   rjoint[robotNum][LEFT_SHOULDER_PITCH].type;
    rjoint[robotNum][RIGHT_SHOULDER_PITCH].px   =   rjoint[robotNum][LEFT_SHOULDER_PITCH].px;
    rjoint[robotNum][RIGHT_SHOULDER_PITCH].py   = - rjoint[robotNum][LEFT_SHOULDER_PITCH].py + 2.0 * STARTY; // change
    rjoint[robotNum][RIGHT_SHOULDER_PITCH].pz   =   rjoint[robotNum][LEFT_SHOULDER_PITCH].pz;
    rjoint[robotNum][RIGHT_SHOULDER_PITCH].axis_x  =   rjoint[robotNum][LEFT_SHOULDER_PITCH].axis_x;
    rjoint[robotNum][RIGHT_SHOULDER_PITCH].axis_y  =   rjoint[robotNum][LEFT_SHOULDER_PITCH].axis_y;
    rjoint[robotNum][RIGHT_SHOULDER_PITCH].axis_z  =   rjoint[robotNum][LEFT_SHOULDER_PITCH].axis_z;
    rjoint[robotNum][RIGHT_SHOULDER_PITCH].lo_stop  =   rjoint[robotNum][LEFT_SHOULDER_PITCH].lo_stop;
    rjoint[robotNum][RIGHT_SHOULDER_PITCH].hi_stop  =   rjoint[robotNum][LEFT_SHOULDER_PITCH].hi_stop;
    rjoint[robotNum][RIGHT_SHOULDER_PITCH].fmax     =   rjoint[robotNum][LEFT_SHOULDER_PITCH].fmax;
    rjoint[robotNum][RIGHT_SHOULDER_PITCH].link[0]  =   RIGHT_UPPER_ARM_DUMMY2;
    rjoint[robotNum][RIGHT_SHOULDER_PITCH].link[1]  =   RIGHT_UPPER_ARM;

    // left hip joint (YAW: z, ROLL: x, PITCH: y)
    rjoint[robotNum][LEFT_HIP_YAW].type =   HINGE;
    rjoint[robotNum][LEFT_HIP_YAW].px   =   rlink[robotNum][LEFT_THIGH_DUMMY1].px;
    rjoint[robotNum][LEFT_HIP_YAW].py   =   rlink[robotNum][LEFT_THIGH_DUMMY1].py;
    rjoint[robotNum][LEFT_HIP_YAW].pz   =   rlink[robotNum][LEFT_THIGH_DUMMY1].pz + 0.5 * JOINT_SIZE;
    rjoint[robotNum][LEFT_HIP_YAW].axis_x  =   0;
    rjoint[robotNum][LEFT_HIP_YAW].axis_y  =   0;
    rjoint[robotNum][LEFT_HIP_YAW].axis_z  =   1;
    rjoint[robotNum][LEFT_HIP_YAW].lo_stop  = - 90 * (M_PI/180.0);
    rjoint[robotNum][LEFT_HIP_YAW].hi_stop  =   90 * (M_PI/180.0);
    rjoint[robotNum][LEFT_HIP_YAW].fmax     =   FMAX;
    rjoint[robotNum][LEFT_HIP_YAW].link[0]  =   TORSO;
    rjoint[robotNum][LEFT_HIP_YAW].link[1]  =   LEFT_THIGH_DUMMY1;
    rjoint[robotNum][LEFT_HIP_ROLL].type    =   HINGE;
    rjoint[robotNum][LEFT_HIP_ROLL].px   =   rlink[robotNum][LEFT_THIGH_DUMMY2].px - 0.5 * JOINT_SIZE;
    rjoint[robotNum][LEFT_HIP_ROLL].py   =   rlink[robotNum][LEFT_THIGH_DUMMY2].py;
    rjoint[robotNum][LEFT_HIP_ROLL].pz   =   rlink[robotNum][LEFT_THIGH_DUMMY2].pz;
    rjoint[robotNum][LEFT_HIP_ROLL].axis_x  =   1;
    rjoint[robotNum][LEFT_HIP_ROLL].axis_y  =   0;
    rjoint[robotNum][LEFT_HIP_ROLL].axis_z  =   0;
    rjoint[robotNum][LEFT_HIP_ROLL].lo_stop  = - 90 * (M_PI/180.0);
    rjoint[robotNum][LEFT_HIP_ROLL].hi_stop  =   90 * (M_PI/180.0);
    rjoint[robotNum][LEFT_HIP_ROLL].fmax     =   FMAX;
    rjoint[robotNum][LEFT_HIP_ROLL].link[0]  =   LEFT_THIGH_DUMMY1;
    rjoint[robotNum][LEFT_HIP_ROLL].link[1]  =   LEFT_THIGH_DUMMY2;
    rjoint[robotNum][LEFT_HIP_PITCH].type    =   HINGE;
    rjoint[robotNum][LEFT_HIP_PITCH].px   =   rlink[robotNum][LEFT_THIGH].px + 0.5 * JOINT_SIZE;
    rjoint[robotNum][LEFT_HIP_PITCH].py   =   rlink[robotNum][LEFT_THIGH].py;
    rjoint[robotNum][LEFT_HIP_PITCH].pz   =   rlink[robotNum][LEFT_THIGH].pz + 0.5 * THIGH_HEIGHT;
    rjoint[robotNum][LEFT_HIP_PITCH].axis_x  =   0;
    rjoint[robotNum][LEFT_HIP_PITCH].axis_y  =   1;
    rjoint[robotNum][LEFT_HIP_PITCH].axis_z  =   0;
    rjoint[robotNum][LEFT_HIP_PITCH].lo_stop  = - 90 * (M_PI/180.0);
    rjoint[robotNum][LEFT_HIP_PITCH].hi_stop  =   90 * (M_PI/180.0);
    rjoint[robotNum][LEFT_HIP_PITCH].fmax     =   FMAX;
    rjoint[robotNum][LEFT_HIP_PITCH].link[0]  =   LEFT_THIGH_DUMMY2;
    rjoint[robotNum][LEFT_HIP_PITCH].link[1]  =   LEFT_THIGH;

    // right hip joint (JT1: roll, JT2: yaw, JT3: pitch)
    rjoint[robotNum][RIGHT_HIP_YAW].type =   rjoint[robotNum][LEFT_HIP_YAW].type;
    rjoint[robotNum][RIGHT_HIP_YAW].px   =   rjoint[robotNum][LEFT_HIP_YAW].px;
    rjoint[robotNum][RIGHT_HIP_YAW].py   = - rjoint[robotNum][LEFT_HIP_YAW].py + 2.0 * STARTY; // change
    rjoint[robotNum][RIGHT_HIP_YAW].pz   =   rjoint[robotNum][LEFT_HIP_YAW].pz;
    rjoint[robotNum][RIGHT_HIP_YAW].axis_x  =   rjoint[robotNum][LEFT_HIP_YAW].axis_x;
    rjoint[robotNum][RIGHT_HIP_YAW].axis_y  =   rjoint[robotNum][LEFT_HIP_YAW].axis_y;
    rjoint[robotNum][RIGHT_HIP_YAW].axis_z  =   rjoint[robotNum][LEFT_HIP_YAW].axis_z;
    rjoint[robotNum][RIGHT_HIP_YAW].lo_stop  =   rjoint[robotNum][LEFT_HIP_YAW].lo_stop;
    rjoint[robotNum][RIGHT_HIP_YAW].hi_stop  =   rjoint[robotNum][LEFT_HIP_YAW].hi_stop;
    rjoint[robotNum][RIGHT_HIP_YAW].fmax     =   rjoint[robotNum][LEFT_HIP_YAW].fmax;
    rjoint[robotNum][RIGHT_HIP_YAW].link[0]  =   TORSO;
    rjoint[robotNum][RIGHT_HIP_YAW].link[1]  =   RIGHT_THIGH_DUMMY1;
    rjoint[robotNum][RIGHT_HIP_ROLL].type    =   rjoint[robotNum][LEFT_HIP_ROLL].type;
    rjoint[robotNum][RIGHT_HIP_ROLL].px   =   rjoint[robotNum][LEFT_HIP_ROLL].px;
    rjoint[robotNum][RIGHT_HIP_ROLL].py   = - rjoint[robotNum][LEFT_HIP_ROLL].py + 2.0 * STARTY; // change
    rjoint[robotNum][RIGHT_HIP_ROLL].pz   =   rjoint[robotNum][LEFT_HIP_ROLL].pz;
    rjoint[robotNum][RIGHT_HIP_ROLL].axis_x  =   rjoint[robotNum][LEFT_HIP_ROLL].axis_x;
    rjoint[robotNum][RIGHT_HIP_ROLL].axis_y  =   rjoint[robotNum][LEFT_HIP_ROLL].axis_y;
    rjoint[robotNum][RIGHT_HIP_ROLL].axis_z  =   rjoint[robotNum][LEFT_HIP_ROLL].axis_z;
    rjoint[robotNum][RIGHT_HIP_ROLL].lo_stop  =   rjoint[robotNum][LEFT_HIP_ROLL].lo_stop;
    rjoint[robotNum][RIGHT_HIP_ROLL].hi_stop  =   rjoint[robotNum][LEFT_HIP_ROLL].hi_stop;
    rjoint[robotNum][RIGHT_HIP_ROLL].fmax     =   rjoint[robotNum][LEFT_HIP_ROLL].fmax;
    rjoint[robotNum][RIGHT_HIP_ROLL].link[0]  =   RIGHT_THIGH_DUMMY1;
    rjoint[robotNum][RIGHT_HIP_ROLL].link[1]  =   RIGHT_THIGH_DUMMY2;
    rjoint[robotNum][RIGHT_HIP_PITCH].type    =   rjoint[robotNum][LEFT_HIP_PITCH].type;
    rjoint[robotNum][RIGHT_HIP_PITCH].px   =   rjoint[robotNum][LEFT_HIP_PITCH].px;
    rjoint[robotNum][RIGHT_HIP_PITCH].py   = - rjoint[robotNum][LEFT_HIP_PITCH].py + 2.0 * STARTY; // change
    rjoint[robotNum][RIGHT_HIP_PITCH].pz   =   rjoint[robotNum][LEFT_HIP_PITCH].pz;
    rjoint[robotNum][RIGHT_HIP_PITCH].axis_x  =   rjoint[robotNum][LEFT_HIP_PITCH].axis_x;
    rjoint[robotNum][RIGHT_HIP_PITCH].axis_y  =   rjoint[robotNum][LEFT_HIP_PITCH].axis_y;
    rjoint[robotNum][RIGHT_HIP_PITCH].axis_z  =   rjoint[robotNum][LEFT_HIP_PITCH].axis_z;
    rjoint[robotNum][RIGHT_HIP_PITCH].lo_stop  =   rjoint[robotNum][LEFT_HIP_PITCH].lo_stop;
    rjoint[robotNum][RIGHT_HIP_PITCH].hi_stop  =   rjoint[robotNum][LEFT_HIP_PITCH].hi_stop;
    rjoint[robotNum][RIGHT_HIP_PITCH].fmax     =   rjoint[robotNum][LEFT_HIP_PITCH].fmax;
    rjoint[robotNum][RIGHT_HIP_PITCH].link[0]  =   RIGHT_THIGH_DUMMY2;
    rjoint[robotNum][RIGHT_HIP_PITCH].link[1]  =   RIGHT_THIGH;

    // left elbow joint (pitch)
    rjoint[robotNum][LEFT_ELBOW_PITCH].type =   HINGE;
    rjoint[robotNum][LEFT_ELBOW_PITCH].px   =   rlink[robotNum][LEFT_UPPER_ARM].px;
    rjoint[robotNum][LEFT_ELBOW_PITCH].py   =   rlink[robotNum][LEFT_UPPER_ARM].py;
    rjoint[robotNum][LEFT_ELBOW_PITCH].pz   =   rlink[robotNum][LEFT_UPPER_ARM].pz - 0.5 * UPPER_ARM_HEIGHT;
    rjoint[robotNum][LEFT_ELBOW_PITCH].axis_x   =   0;
    rjoint[robotNum][LEFT_ELBOW_PITCH].axis_y   =   1;
    rjoint[robotNum][LEFT_ELBOW_PITCH].axis_z   =   0;
    rjoint[robotNum][LEFT_ELBOW_PITCH].lo_stop  = - 179 * (M_PI/180.0);
    rjoint[robotNum][LEFT_ELBOW_PITCH].hi_stop  =   0;
    rjoint[robotNum][LEFT_ELBOW_PITCH].fmax     =   FMAX;
    rjoint[robotNum][LEFT_ELBOW_PITCH].link[0]  =   LEFT_UPPER_ARM;
    rjoint[robotNum][LEFT_ELBOW_PITCH].link[1]  =   LEFT_FORE_ARM;
    rjoint[robotNum][LEFT_ELBOW_PITCH].fudge_factor = 0.1;

    // right elbow joint (pitch)
    rjoint[robotNum][RIGHT_ELBOW_PITCH].type =   rjoint[robotNum][LEFT_ELBOW_PITCH].type;
    rjoint[robotNum][RIGHT_ELBOW_PITCH].px   =   rjoint[robotNum][LEFT_ELBOW_PITCH].px;
    rjoint[robotNum][RIGHT_ELBOW_PITCH].py   = - rjoint[robotNum][LEFT_ELBOW_PITCH].py + 2.0 * STARTY; // change
    rjoint[robotNum][RIGHT_ELBOW_PITCH].pz   =   rjoint[robotNum][LEFT_ELBOW_PITCH].pz;
    rjoint[robotNum][RIGHT_ELBOW_PITCH].axis_x   =   rjoint[robotNum][LEFT_ELBOW_PITCH].axis_x;
    rjoint[robotNum][RIGHT_ELBOW_PITCH].axis_y   =   rjoint[robotNum][LEFT_ELBOW_PITCH].axis_y;
    rjoint[robotNum][RIGHT_ELBOW_PITCH].axis_z   =   rjoint[robotNum][LEFT_ELBOW_PITCH].axis_z;
    rjoint[robotNum][RIGHT_ELBOW_PITCH].lo_stop  =   rjoint[robotNum][LEFT_ELBOW_PITCH].lo_stop;
    rjoint[robotNum][RIGHT_ELBOW_PITCH].hi_stop  =   rjoint[robotNum][LEFT_ELBOW_PITCH].hi_stop;
    rjoint[robotNum][RIGHT_ELBOW_PITCH].fmax     =   rjoint[robotNum][LEFT_ELBOW_PITCH].fmax;
    rjoint[robotNum][RIGHT_ELBOW_PITCH].link[0]  =   RIGHT_UPPER_ARM;
    rjoint[robotNum][RIGHT_ELBOW_PITCH].link[1]  =   RIGHT_FORE_ARM;
    rjoint[robotNum][RIGHT_ELBOW_PITCH].fudge_factor = rjoint[robotNum][LEFT_ELBOW_PITCH].fudge_factor;

    // left knee joint
    rjoint[robotNum][LEFT_KNEE_PITCH].type =   HINGE;
    rjoint[robotNum][LEFT_KNEE_PITCH].px   =   rlink[robotNum][LEFT_THIGH].px;
    rjoint[robotNum][LEFT_KNEE_PITCH].py   =   rlink[robotNum][LEFT_THIGH].py;
    rjoint[robotNum][LEFT_KNEE_PITCH].pz   =   rlink[robotNum][LEFT_THIGH].pz - 0.5 * THIGH_HEIGHT;
    rjoint[robotNum][LEFT_KNEE_PITCH].axis_x  =   0;
    rjoint[robotNum][LEFT_KNEE_PITCH].axis_y  =   1;
    rjoint[robotNum][LEFT_KNEE_PITCH].axis_z  =   0;
    rjoint[robotNum][LEFT_KNEE_PITCH].lo_stop =   0;  //  可動域 最小 max -pi
    rjoint[robotNum][LEFT_KNEE_PITCH].hi_stop =   170.0 * (M_PI/180.0);  // 可動域 最大 max +pi
    rjoint[robotNum][LEFT_KNEE_PITCH].fmax    =   FMAX;
    rjoint[robotNum][LEFT_KNEE_PITCH].link[0] =   LEFT_THIGH;
    rjoint[robotNum][LEFT_KNEE_PITCH].link[1] =   LEFT_CALF;
    rjoint[robotNum][LEFT_KNEE_PITCH].fudge_factor = 0.9;

    // right knee joint
    rjoint[robotNum][RIGHT_KNEE_PITCH].type =   rjoint[robotNum][LEFT_KNEE_PITCH].type;
    rjoint[robotNum][RIGHT_KNEE_PITCH].px   =   rjoint[robotNum][LEFT_KNEE_PITCH].px;
    rjoint[robotNum][RIGHT_KNEE_PITCH].py   = - rjoint[robotNum][LEFT_KNEE_PITCH].py + 2.0 * STARTY; // change
    rjoint[robotNum][RIGHT_KNEE_PITCH].pz   =   rjoint[robotNum][LEFT_KNEE_PITCH].pz;
    rjoint[robotNum][RIGHT_KNEE_PITCH].axis_x   =   rjoint[robotNum][LEFT_KNEE_PITCH].axis_x;
    rjoint[robotNum][RIGHT_KNEE_PITCH].axis_y   =   rjoint[robotNum][LEFT_KNEE_PITCH].axis_y;
    rjoint[robotNum][RIGHT_KNEE_PITCH].axis_z   =   rjoint[robotNum][LEFT_KNEE_PITCH].axis_z;
    rjoint[robotNum][RIGHT_KNEE_PITCH].lo_stop  =   rjoint[robotNum][LEFT_KNEE_PITCH].lo_stop;
    rjoint[robotNum][RIGHT_KNEE_PITCH].hi_stop  =   rjoint[robotNum][LEFT_KNEE_PITCH].hi_stop;
    rjoint[robotNum][RIGHT_KNEE_PITCH].fmax     =   rjoint[robotNum][LEFT_KNEE_PITCH].fmax;
    rjoint[robotNum][RIGHT_KNEE_PITCH].link[0]  =   RIGHT_THIGH;
    rjoint[robotNum][RIGHT_KNEE_PITCH].link[1]  =   RIGHT_CALF;
    rjoint[robotNum][RIGHT_KNEE_PITCH].fudge_factor = rjoint[robotNum][RIGHT_KNEE_PITCH].fudge_factor;

    // left foot joint (pitch: y axis, roll: x axis)
    rjoint[robotNum][LEFT_FOOT_PITCH].type    =   HINGE; //HINGE
    rjoint[robotNum][LEFT_FOOT_PITCH].px      =   rlink[robotNum][LEFT_CALF].px;
    rjoint[robotNum][LEFT_FOOT_PITCH].py      =   rlink[robotNum][LEFT_CALF].py;
    rjoint[robotNum][LEFT_FOOT_PITCH].pz      =   rlink[robotNum][LEFT_CALF].pz - 0.5 * CALF_LENGTH;
    rjoint[robotNum][LEFT_FOOT_PITCH].axis_x  =   0;
    rjoint[robotNum][LEFT_FOOT_PITCH].axis_y  =   1;
    rjoint[robotNum][LEFT_FOOT_PITCH].axis_z  =   0;
    rjoint[robotNum][LEFT_FOOT_PITCH].lo_stop = - 90.0 * (M_PI/180.0);
    rjoint[robotNum][LEFT_FOOT_PITCH].hi_stop =   90.0 * (M_PI/180.0);
    rjoint[robotNum][LEFT_FOOT_PITCH].fmax    =   FMAX;
    rjoint[robotNum][LEFT_FOOT_PITCH].link[0] =   LEFT_CALF;
    rjoint[robotNum][LEFT_FOOT_PITCH].link[1] =   LEFT_FOOT_DUMMY;
    rjoint[robotNum][LEFT_FOOT_ROLL].type     =   HINGE;
    rjoint[robotNum][LEFT_FOOT_ROLL].px       =   rlink[robotNum][LEFT_CALF].px;
    rjoint[robotNum][LEFT_FOOT_ROLL].py       =   rlink[robotNum][LEFT_CALF].py;
    rjoint[robotNum][LEFT_FOOT_ROLL].pz       =   rlink[robotNum][LEFT_FOOT_DUMMY].pz - 0.5 * (JOINT_SIZE);
    rjoint[robotNum][LEFT_FOOT_ROLL].axis_x   =   1;  // ピッチ
    rjoint[robotNum][LEFT_FOOT_ROLL].axis_y   =   0;
    rjoint[robotNum][LEFT_FOOT_ROLL].axis_z   =   0;
    rjoint[robotNum][LEFT_FOOT_ROLL].lo_stop  = - 90.0 * (M_PI/180.0); // 90.0
    rjoint[robotNum][LEFT_FOOT_ROLL].hi_stop  =   90.0 * (M_PI/180.0);
    rjoint[robotNum][LEFT_FOOT_ROLL].fmax     =   FMAX;
    rjoint[robotNum][LEFT_FOOT_ROLL].link[0]  =   LEFT_FOOT_DUMMY;
    rjoint[robotNum][LEFT_FOOT_ROLL].link[1]  =   LEFT_FOOT;

    // right foot joint
    rjoint[robotNum][RIGHT_FOOT_PITCH].type    =   rjoint[robotNum][LEFT_FOOT_PITCH].type;
    rjoint[robotNum][RIGHT_FOOT_PITCH].px      =   rjoint[robotNum][LEFT_FOOT_PITCH].px;
    rjoint[robotNum][RIGHT_FOOT_PITCH].py      = - rjoint[robotNum][LEFT_FOOT_PITCH].py + 2.0 * STARTY; // change
    rjoint[robotNum][RIGHT_FOOT_PITCH].pz      =   rjoint[robotNum][LEFT_FOOT_PITCH].pz;
    rjoint[robotNum][RIGHT_FOOT_PITCH].axis_x  =   rjoint[robotNum][LEFT_FOOT_PITCH].axis_x;
    rjoint[robotNum][RIGHT_FOOT_PITCH].axis_y  =   rjoint[robotNum][LEFT_FOOT_PITCH].axis_y;
    rjoint[robotNum][RIGHT_FOOT_PITCH].axis_z  =   rjoint[robotNum][LEFT_FOOT_PITCH].axis_z;
    rjoint[robotNum][RIGHT_FOOT_PITCH].lo_stop =   rjoint[robotNum][LEFT_FOOT_PITCH].lo_stop;
    rjoint[robotNum][RIGHT_FOOT_PITCH].hi_stop =   rjoint[robotNum][LEFT_FOOT_PITCH].hi_stop;
    rjoint[robotNum][RIGHT_FOOT_PITCH].fmax    =   rjoint[robotNum][LEFT_FOOT_PITCH].fmax;
    rjoint[robotNum][RIGHT_FOOT_PITCH].link[0] =   RIGHT_CALF;
    rjoint[robotNum][RIGHT_FOOT_PITCH].link[1] =   RIGHT_FOOT_DUMMY;
    rjoint[robotNum][RIGHT_FOOT_ROLL].type     =   rjoint[robotNum][LEFT_FOOT_ROLL].type;
    rjoint[robotNum][RIGHT_FOOT_ROLL].px       =   rjoint[robotNum][LEFT_FOOT_ROLL].px;
    rjoint[robotNum][RIGHT_FOOT_ROLL].py       = - rjoint[robotNum][LEFT_FOOT_ROLL].py + 2.0 * STARTY; // chnage
    rjoint[robotNum][RIGHT_FOOT_ROLL].pz       =   rjoint[robotNum][LEFT_FOOT_ROLL].pz;
    rjoint[robotNum][RIGHT_FOOT_ROLL].axis_x   =   rjoint[robotNum][LEFT_FOOT_ROLL].axis_x;
    rjoint[robotNum][RIGHT_FOOT_ROLL].axis_y   =   rjoint[robotNum][LEFT_FOOT_ROLL].axis_y;
    rjoint[robotNum][RIGHT_FOOT_ROLL].axis_z   =   rjoint[robotNum][LEFT_FOOT_ROLL].axis_z;
    rjoint[robotNum][RIGHT_FOOT_ROLL].lo_stop  =   rjoint[robotNum][LEFT_FOOT_ROLL].hi_stop;
    rjoint[robotNum][RIGHT_FOOT_ROLL].hi_stop  =   rjoint[robotNum][LEFT_FOOT_ROLL].lo_stop;
    rjoint[robotNum][RIGHT_FOOT_ROLL].fmax     =   rjoint[robotNum][LEFT_FOOT_ROLL].fmax;
    rjoint[robotNum][RIGHT_FOOT_ROLL].link[0]  =   RIGHT_FOOT_DUMMY;
    rjoint[robotNum][RIGHT_FOOT_ROLL].link[1]  =   RIGHT_FOOT;
}

static void makeRobot(int robotNum)
{
    readLinkParam(robotNum);
    readJointParam(robotNum);

    for (int i=0; i< BODY_NUM; i++)
    {
        rlink[robotNum][i].id = dBodyCreate(world);
        printf("%d:id:%d\n", robotNum,rlink[robotNum][i].id );
        dBodySetPosition(rlink[robotNum][i].id, rlink[robotNum][i].px, rlink[robotNum][i].py, rlink[robotNum][i].pz);

        if ((i == HEAD) ||  (i == LEFT_HAND) || (i == RIGHT_HAND))
        {
            dMass m;
            dMassSetSphereTotal(&m, rlink[robotNum][i].m, rlink[robotNum][i].r);
            dMassAdjust(&m,rlink[robotNum][i].m);
            dBodySetMass(rlink[robotNum][i].id, &m);
            rlink[robotNum][i].gid = dCreateSphere(space,rlink[robotNum][i].r);
            dGeomSetBody(rlink[robotNum][i].gid, rlink[robotNum][i].id);
        }
        else
        {
            if ((i == LEFT_FOOT) || (i == RIGHT_FOOT)) {
              dMass m;
              dMassSetBoxTotal(&m, rlink[robotNum][i].m, rlink[robotNum][i].lx, rlink[robotNum][i].ly, rlink[robotNum][i].lz);
              dMassAdjust(&m,rlink[robotNum][i].m);
              dBodySetMass(rlink[robotNum][i].id, &m);
              rlink[robotNum][i].gid = dCreateBox(space, rlink[robotNum][i].lx, rlink[robotNum][i].ly, rlink[robotNum][i].lz);
              dGeomSetBody(rlink[robotNum][i].gid, rlink[robotNum][i].id);
            }
            else {
              dMass m;
              dMassSetCylinderTotal(&m, rlink[robotNum][i].m, 3, 0.5 * rlink[robotNum][i].lx, rlink[robotNum][i].lz);
              dMassAdjust(&m,rlink[robotNum][i].m);
              dBodySetMass(rlink[robotNum][i].id, &m);
              rlink[robotNum][i].gid = dCreateCylinder(space, 0.5 * rlink[robotNum][i].lx, rlink[robotNum][i].lz);
              dGeomSetBody(rlink[robotNum][i].gid, rlink[robotNum][i].id);
            }
        }
    }

    // Set joint paramter
    for (int i=0; i< JOINT_NUM; i++)
    {
        rjoint[robotNum][i].id = dJointCreateHinge(world,0);
        dJointAttach(rjoint[robotNum][i].id , rlink[robotNum][rjoint[robotNum][i].link[1]].id, rlink[robotNum][rjoint[robotNum][i].link[0]].id);
        dJointSetHingeAnchor(rjoint[robotNum][i].id, rjoint[robotNum][i].px, rjoint[robotNum][i].py, rjoint[robotNum][i].pz);
        dJointSetHingeAxis(rjoint[robotNum][i].id, rjoint[robotNum][i].axis_x, rjoint[robotNum][i].axis_y,rjoint[robotNum][i].axis_z);
        dJointSetHingeParam(rjoint[robotNum][i].id, dParamLoStop, rjoint[robotNum][i].lo_stop);
        dJointSetHingeParam(rjoint[robotNum][i].id, dParamHiStop, rjoint[robotNum][i].hi_stop);
        dJointSetHingeParam(rjoint[robotNum][i].id, dParamFMax,   rjoint[robotNum][i].fmax);
        dJointSetHingeParam(rjoint[robotNum][i].id, dParamFudgeFactor, FUDGE_FACTOR);
    }
    printf("made robot %d\n",robotNum);
}

int inputFile(){

  sprintf(filename, "./gene_data/%04d.csv", currentGeneration);
  fp = fopen( filename, "r" );
  if( fp == NULL ){
    printf( "ファイルが開けません¥n");
    return -1;
  }

  for (int i = 0; i < EXIST_ROBOT_NUM; ++i){
      for (int j = 0; j < ACTION_NUM; ++j){
          fscanf( fp, "%d,", &action[i][j]);
      }
  }
  fclose(fp);
  for (int i = 0; i < EXIST_ROBOT_NUM; ++i){
    printf("---------------------\n");
    printf("%d体目のロボット\n",i+1);
      for (int j = 0; j < ACTION_NUM; ++j){
          printf("%d,", action[i][j]);
      }
    printf("\n");
  }

  // while( (fscanf( fp, "%d,%d", &action[0], &action[1]) ) != EOF ){
  //   printf( "%d %d\n", action[0],action[1]);
  // }
  return 0;
}

int main(int argc, char *argv[]){
    fn.version = DS_VERSION;
    fn.start   = &start;
    fn.step    = &simLoop;
    fn.command = &command;
    //fn.path_to_textures = "../../drawstuff/textures";
    fn.path_to_textures = ".";

    currentGeneration = atoi(argv[1]);

    dInitODE();
    srand((unsigned) time(NULL));
    inputFile();

    world  = dWorldCreate();
    space  = dHashSpaceCreate(0);
    ground = dCreatePlane(space, 0, 0, 1, 0);
    contactgroup = dJointGroupCreate(0);

    dWorldSetGravity(world, 0, 0, -9.8);
    dWorldSetERP(world, 0.9);
    dWorldSetCFM(world, 1e-4);

    for (int robotNum = 0; robotNum < EXIST_ROBOT_NUM; robotNum++){
        step[robotNum] = 0;
        sw[robotNum] = 0;
        makeRobot(robotNum);
    }

    dsSimulationLoop(argc, argv, 800, 600, &fn);

    dJointGroupDestroy(contactgroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();
    return 0;
}
