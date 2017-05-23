#!/bin/sh
currentGeneration=915
for i in `seq 0 3000`
do
  ./a.out ${currentGeneration} -notex
  currentGeneration=`expr ${currentGeneration} + 1`
done



