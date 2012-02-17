#include<stdio.h>
#include<stdlib.h>
#include <time.h>
#include<math.h>

int main(int argc, char** argv){
  srand (time(NULL));

  FILE* fLoc = fopen("bootstrap_loc.yaml","w");
  FILE* fExp = fopen("bootstrap_exp.yaml","w");

  fprintf(fExp,"goal_tolerance:\n  xyz: 0.02 0.02 0.02\n  yaw: 0.1\n\n");
  fprintf(fExp,"object_pose_in_gripper:\n  right:\n    xyz: -0.20 -0.1 0.0\n    rpy: 0.0 0.0 0.0\n  left:\n    xyz: -0.20 0.1 0.0\n    rpy: 0.0 0.0 0.0\n\n");
  fprintf(fExp,"use_current_pose_as_start_state: true\n");
  fprintf(fExp,"start_state:\n  right: 0.0 0.018 0.00 -0.43 0.00 -0.00 0.0\n  left: 0.0 0.018 0.00 -0.43 0.00 -0.00 0.0\n  base: 1.9 0.6 1.57\n  spine: 0.0\n\n");
  fprintf(fExp,"experiments:\n\n");
  fprintf(fLoc,"locations:\n\n");

  /* bootstrap ordering
   * every 3rd idx is palette (starting at 0)
   *  5 high by 4 wide (20 goals)
   * odd idx is bookshelf
   *  mid shelf palette to wall (10)
   *  bottom shelf palette to wall (10)
   *  top shelf palette to wall (10)
   */

  int palette_height = 0;
  int palette_across = 0;
  int shelf_height = 0;
  int shelf_across = 0;
  for(int i=0; i<45; i++){
    if(i%3 == 0){
      double x = 1.2 + palette_across*(1.9-1.2)/2.0;
      double y = 2.15;
      double z = 0.35 + palette_height*(1.15-0.35)/4.0;
      double theta = 1.57;
      fprintf(fExp,"  - name: box%.3d\n    goal: palette%d\n    pre_action: attach\n    post_action: detach\n    sound_bite: \"box %d\"\n",i,i,i);
      fprintf(fLoc,"  - name: palette%d\n    pose: %f %f %f %f\n",i,x,y,z,theta);
      palette_across++;
      if(palette_across==3){
        palette_across = 0;
        palette_height++;
      }
    }
    else{
      double x = 0.4;
      double y = 0.4 + shelf_across*(1.6-0.4)/9.0;
      double z;
      if(shelf_height==0)
        z = 0.4;
      else if(shelf_height==1)
        z = 0.8;
      else if(shelf_height==2)
        z = 1.2;
      else
        exit(0);
      double theta = 3.14;
      fprintf(fExp,"  - name: box%.3d\n    goal: shelf%d\n    pre_action: attach\n    post_action: detach\n    sound_bite: \"box %d\"\n",i,i,i);
      fprintf(fLoc,"  - name: shelf%d\n    pose: %f %f %f %f\n",i,x,y,z,theta);
      shelf_across++;
      if(shelf_across==10){
        shelf_across = 0;
        shelf_height++;
      }
    }
  }

  fclose(fLoc);
  fclose(fExp);

  return 0;
}

