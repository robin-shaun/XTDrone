kill -9 $(ps -ef|grep set_position.py|gawk '$0 !~/grep/ {print $2}' |tr -s '\n' ' ')
