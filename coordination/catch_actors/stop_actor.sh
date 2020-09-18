kill -9 $(ps -ef|grep catching_uavs.py|gawk '$0 !~/grep/ {print $2}' |tr -s '\n' ' ')
