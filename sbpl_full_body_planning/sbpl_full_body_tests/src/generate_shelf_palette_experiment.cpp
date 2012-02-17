#include<stdio.h>
#include<stdlib.h>
#include <time.h>
#include<math.h>

int main(int argc, char** argv){
  srand (time(NULL));

  FILE* fLoc = fopen("rand_exp_loc.yaml","w");
  FILE* fExp = fopen("rand_exp.yaml","w");

  fprintf(fExp,"goal_tolerance:\n  xyz: 0.02 0.02 0.02\n  yaw: 0.1\n\n");
  fprintf(fExp,"object_pose_in_gripper:\n  right:\n    xyz: -0.20 -0.1 0.0\n    rpy: 0.0 0.0 0.0\n  left:\n    xyz: -0.20 0.1 0.0\n    rpy: 0.0 0.0 0.0\n\n");
  fprintf(fExp,"use_current_pose_as_start_state: true\n");
  fprintf(fExp,"start_state:\n  right: 0.0 0.018 0.00 -0.43 0.00 -0.00 0.0\n  left: 0.0 0.018 0.00 -0.43 0.00 -0.00 0.0\n  base: 1.9 0.6 1.57\n  spine: 0.0\n\n");
  fprintf(fExp,"experiments:\n\n");
  fprintf(fLoc,"locations:\n\n");

  for(int i=0; i<100; i++){
    if(i%2 == 0){
      double x = (rand()%int((1.9-1.2)/0.02))*0.02 + 1.2; //1.2 to 1.9
      double y = 2.15 - (rand()%6)*0.02; //around 2.15
      double z = (rand()%int((1.15-0.35)/0.02))*0.02 + 0.35; //0.35 to 1.15
      double theta = 1.57 + ((rand()%3)-1)*11.25*M_PI/180;
      fprintf(fExp,"  - name: box%.3d_pickup\n    goal: palette%d\n    pre_action: attach\n    post_action: detach\n    sound_bite: \"picking up box %d\"\n",i/2,i/2,i/2);
      fprintf(fLoc,"  - name: palette%d\n    pose: %f %f %f %f\n",i/2,x,y,z,theta);
    }
    else{
      double x = 0.4 + (rand()%6)*0.02; //around 0.4
      double y = (rand()%int((1.6-0.4)/0.02))*0.02 + 0.4; //0.4 to 1.6
      double z = double(rand()%3); //around 0.25 or 0.7 or 1.15
      if(z==0)
        z = 0.4;
      else if(z==1)
        z = 0.8;
      else if(z==2)
        z = 1.2;
      else
        exit(0);
      z += ((rand()%7)-3)*0.02;
      double theta = 3.14 + ((rand()%3)-1)*11.25*M_PI/180;
      fprintf(fExp,"  - name: box%.3d_putdown\n    goal: shelf%d\n    pre_action: attach\n    post_action: detach\n    sound_bite: \"putting down box %d\"\n",i/2,i/2,i/2);
      fprintf(fLoc,"  - name: shelf%d\n    pose: %f %f %f %f\n",i/2,x,y,z,theta);
    }
  }

  fclose(fLoc);
  fclose(fExp);

  return 0;
}

