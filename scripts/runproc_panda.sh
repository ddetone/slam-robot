#!/bin/sh

if [ -z $1 ]; then
	echo "no logfile specified (first argument value)"
else
	export LCM_LOGFILE=$1
	echo "LCM logging to: $LCM_LOGFILE"
fi

#run the brain

### Java memory constants
export JAVA=/usr/bin/java
export JAVA_OPTS_3072="-ea -server -Xmx3072m"
export JAVA_OPTS_2048="-ea -server -Xmx2048m"
export JAVA_OPTS_1024="-ea -server -Xmx1024m"
export JAVA_OPTS_512="-ea -server -Xmx512m"
export JAVA_OPTS_256="-ea -server -Xmx256m"
export JAVA_OPTS_128="-ea -server -Xmx128m"
export JAVA_OPTS_64="-ea -server -Xmx64m"

# Assuming that there is a file here...
PROC_CONFIG="$BOTLAB_HOME/config/procman/panda.proc"

# Step 1: start ProcManDaemons
export DAEMON_CMD="$JAVA $JAVA_OPTS_64 april.procman.ProcManDaemon -n panda"
$DAEMON_CMD &

# Step 2: start ProcMan with correct config file:
export CONTROLLER_CMD="$JAVA $JAVA_OPTS_64 april.procman.ProcMan -c $PROC_CONFIG"

$CONTROLLER_CMD
