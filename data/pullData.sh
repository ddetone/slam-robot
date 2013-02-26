#!/bin/bash
set -e

# $1 should be the hostname

if [ -z $1 ]; then
	echo "no host specified (first argument value)"
	exit
fi

cd $BOTLAB_HOME/data
scp $1:~/botlab/data/* ./
ssh $1 rm $2botlab/data/*
git add ./*
