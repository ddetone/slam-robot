#!/bin/bash
set -e

# $1 should be the hostname

if [ -z $1 ]; then
	echo "no host specified (first argument value)"
	exit
fi

scp -pr java/botlab.jar $1:

