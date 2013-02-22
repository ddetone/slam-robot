#!/bin/bash
set -e

# $1 should be the hostname

if [ -z $1 ]; then
	echo "no host specified (first argument value)"
	exit
fi

cd $BOTLAB_HOME/java
ant clean; ant

ssh $1 mkdir -p $2botlab

scp -pr $BOTLAB_HOME/config $1:$2botlab/
scp -pr $BOTLAB_HOME/java/botlab.jar $1:$2botlab/
scp -pr $BOTLAB_HOME/lcmtypes $1:$2botlab/
scp -pr $BOTLAB_HOME/scripts $1:$2botlab/

