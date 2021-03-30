kill -9 $(ps -ef|grep xtd_ui.py|gawk '$0 !~/grep/ {print $2}' |tr -s '\n' ' ')
kill -9 $(ps -ef|grep send.py|gawk '$0 !~/grep/ {print $2}' |tr -s '\n' ' ')
kill -9 $(ps -ef|grep receive.py|gawk '$0 !~/grep/ {print $2}' |tr -s '\n' ' ')
kill -9 $(ps -ef|grep main.py|gawk '$0 !~/grep/ {print $2}' |tr -s '\n' ' ')
kill -9 $(ps -ef|grep plotcanvas.py|gawk '$0 !~/grep/ {print $2}' |tr -s '\n' ' ')

