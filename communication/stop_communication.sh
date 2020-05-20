kill -9 $(ps -ef|grep multirotor_communication.py|gawk '$0 !~/grep/ {print $2}' |tr -s '\n' ' ')
kill -9 $(ps -ef|grep plane_communication.py|gawk '$0 !~/grep/ {print $2}' |tr -s '\n' ' ')
kill -9 $(ps -ef|grep rover_communication.py|gawk '$0 !~/grep/ {print $2}' |tr -s '\n' ' ')
kill -9 $(ps -ef|grep vtol_communication.py|gawk '$0 !~/grep/ {print $2}' |tr -s '\n' ' ')