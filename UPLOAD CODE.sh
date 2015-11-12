		#!/bin/sh
HOST='10.16.78.2'
USER=''
PASSWD=''

ftp -n $HOST <<END_SCRIPT
quote USER $USER
quote PASS $PASSWD
cd /ni-rt/system
put PPC603gnu/$(basename `pwd`)/Debug/$(basename `pwd`).out FRC_UserProgram.out
quit
END_SCRIPT
exit 0
