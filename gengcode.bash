#!/usr/bin/bash

arguments="
--scale=fit
--tolerance=0.1
--align-x=center
--align-y=center
--area=500,500,0,0
--lift-delta-z=1
--work-z=0"

#--config-file=
#--pens=pensfile


./gcodeplot/gcodeplot.py $arguments $1 >  output$RANDOM.gcode
