kill -9 $(ps -ef|grep follower.py|gawk '$0 !~/grep/ {print $2}' |tr -s '\n' ' ')
kill -9 $(ps -ef|grep follower_consensus.py|gawk '$0 !~/grep/ {print $2}' |tr -s '\n' ' ')
kill -9 $(ps -ef|grep leader.py|gawk '$0 !~/grep/ {print $2}' |tr -s '\n' ' ')
kill -9 $(ps -ef|grep avoid.py|gawk '$0 !~/grep/ {print $2}' |tr -s '\n' ' ')