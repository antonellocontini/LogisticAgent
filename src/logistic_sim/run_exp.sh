# !/bin/bash
#
# GIT version. Please do not edit this file.
# Make a copy and edit the copy.
#
# MAP: Available maps: grid, example, cumberland, broughton, DIAG_labs, DIAG_floor1
# NROBOTS: number of robots
# INITPOS: initial positions of the robots: (default,a,b,c,...)
# ALG: Algorithm: RAND,CR,HCR,HPCC,CGG,MSP,GBS,SEBS,CBLS,DTAG,DTAP
# LOC: Localization mode: AMCL, fake_localization
# NAV: Navigation module: ros, spqrel_navigation
# GWAIT: Goal wait: how much time the robot stops when it reaches a goal
# COMMDELAY: communication delay of all messages (in seconds)
# TERM: Terminal to use gnome-terminal,xterm
# TIMEOUT: simulation timeout (seconds)
# CUSTOM_STAGE: flag if custom version of Stage is used: true, false
# SPEEDUP: simulator speedup (if Custom Stage is used)

MAP=icelab_black
NROBOTS=2
INITPOS=default
ALG=GlobalAgent
LOC=AMCL
NAV=ros
GWAIT=0
COMMDELAY=0.2
TERM=gnome-terminal
TIMEOUT=1800
CUSTOM_STAGE=false
SPEEDUP=3.0
CAPACITY=3
TP_NAME=GlobalTaskPlanner
GEN=uniform
PERM=true

./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# NROBOTS=4

# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# NROBOTS=6

# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM

# NROBOTS=2
# GEN=non-uniform
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# NROBOTS=4

# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# NROBOTS=6

# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM

# GEN=non-uniform

# NROBOTS=2
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# NROBOTS=4

# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# NROBOTS=6

# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM

# NROBOTS=2
# GEN=uniform
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# NROBOTS=4

# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# NROBOTS=6

# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM
# ./start_experiment.py $MAP $NROBOTS $INITPOS $ALG $LOC $NAV $GWAIT $COMMDELAY $TERM $TIMEOUT $CUSTOM_STAGE $SPEEDUP $CAPACITY $TP_NAME $GEN $PERM

