//1世代目の種を生成するプログラム

#include <stdio.h>
#include <stdlib.h>
#include "original.h"

int main(int argc, char const *argv[])
{
  FILE *fp;
  char filename[100];
  sprintf(filename, "./gene_data/%04d.csv", 1);
  if((fp=fopen(filename,"w"))==NULL){
    printf("error\n");
    exit(1);
  }
  srand((unsigned) time(NULL));

  for (int i = 0; i < MAX_ROBOT_NUM; ++i){
    for (int j = 0; j < ACTION_NUM; ++j){
      fprintf(fp,"%d,", rand()%8);
    }
    fprintf(fp,"\n");
  }


  return 0;
}