kill -9 $(ps -ef|grep control_actor.py|gawk '$0 !~/grep/ {print $2}' |tr -s '\n' ' ')
