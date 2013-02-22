#!/bin/bash

#1 should be the atom ip

if [ -z $1 ]; then
    echo "no host specified (first argument value)"
    exit
fi

ssh $1 "kill -2 \`ps -ef | grep -e '.*ProcMan\b.*' | grep -v 'grep' | sed -e 's/\S\+\s\+\(\S\+\).*/\1/'\`"

