#!/bin/bash
set -e

# $1 should be the hostname

if [ -z $1 ]; then
	echo "no host specified (first argument value)"
	exit
fi

cd $BOTLAB_HOME/java

if [ "$2" = "ant" ]; then
	ant clean; ant
fi

ssh $1 'mkdir -p $3botlab; mkdir -p $3botlab/data'

#scp -pr $BOTLAB_HOME/config $1:$2botlab/
#scp -pr $BOTLAB_HOME/java/botlab.jar $1:$2botlab/
#scp -pr $BOTLAB_HOME/lcmtypes $1:$2botlab/
#scp -pr $BOTLAB_HOME/scripts/runproc_panda.sh $1:$2botlab/
scp -pr $BOTLAB_HOME/scripts/runproc_panda.sh $BOTLAB_HOME/config $BOTLAB_HOME/java/botlab.jar $BOTLAB_HOME/lcmtypes $1:$3botlab/

