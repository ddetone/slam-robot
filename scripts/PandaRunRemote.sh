#!/bin/bash
#set -e

if [ -z $1 ]; then
    echo "no host specified (first argument value)"
    exit
fi


control_c()
# run if user hits control-c
{
  echo ""
  echo "killing panda..."
  $BOTLAB_HOME/scripts/killPanda.sh panda
  echo "panda is dead"
  exit $?
}

# trap keyboard interrupt (control-c)
trap control_c SIGINT

clear

echo "Starting procman on panda..."

LCM_LOGFILE="\$BOTLAB_HOME/data/$(date -u +%m"."%d"."%g"."%H"."%M)"
PROCMAN_CMD="/home/panda-user/botlab/scripts/runproc_panda.sh"
PROCMAN_CMD="bash -e \$BOTLAB_HOME/scripts/runproc_panda.sh"

set -x
ssh $1 "$PROCMAN_CMD"
 
