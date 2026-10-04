#coding: utf-8
import rospy
import sys
from std_msgs.msg import Int16,String, Time, Float32
from ros_actor_cmd_pose_plugin_msgs.msg import ActorInfo
from gazebo_msgs.srv import DeleteModel,GetModelState
import time
import os
try:
    import cv2
except ImportError:
    cv2 = None


uav_type = (sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith('-')
            else 'typhoon_h480')
actor_num = 6
uav_num = 6
err_threshold = 1
coordx_bias = 3
coordy_bias = 9
actor_id_dict = {'green':[0], 'blue':[1], 'brown':[2], 'white':[3], 'red':[4,5]}

MAX_UAV_ALTITUDE = 6.0
DETECTION_INTERVAL = 1.0
DETECTION_DURATION = 15.0
MISSION_TIMEOUT = 600.0
DEFAULT_UAV_LOSS_PENALTY = 100.0
UAV_MISS_LIMIT = 3

left_actors = []
find_actors = []
actors_pos = [None] * actor_num
count_flag = [False] * actor_num
topic_arrive_time = [0.0] * actor_num
find_time = [0.0] * actor_num
find_actor_pub = []
mission_finished = False
uav_loss_count = 0
uav_loss_penalty = DEFAULT_UAV_LOSS_PENALTY
sensor_cost = 0.0
target_finish = 0
find_finish = 0
score = 0.0
time_usage = 0.0
start_time = 0.0
flag_1 = 0
flag_2 = 1


def _now():
    return rospy.get_time()


def _publish_score():
    if 'score_pub' not in globals():
        return
    value = int(round(max(-32768, min(32767, score))))
    score_pub.publish(value)


def _progress_score():
    return (find_finish * 50 + target_finish * 100
            - sensor_cost * 3e-3 - uav_loss_count * uav_loss_penalty)


def _finish(reason, final_score=None):
    global score, mission_finished
    if mission_finished:
        return
    mission_finished = True
    if final_score is not None:
        score = final_score
    print(reason)
    print('score:', score)
    _publish_score()
    signal_shutdown = getattr(rospy, 'signal_shutdown', None)
    if signal_shutdown:
        signal_shutdown(reason)


def _reset_detection(actor_id):
    count_flag[actor_id] = False
    find_time[actor_id] = 0.0
    topic_arrive_time[actor_id] = 0.0


def _delete_actor(actor_id):
    global target_finish, score
    try:
        response = del_model('actor_' + str(actor_id))
        if hasattr(response, 'success') and not response.success:
            print('Unable to remove actor_%d: %s' % (actor_id, getattr(response, 'status_message', 'unknown error')))
            return False
    except Exception as exc:
        print('Unable to remove actor_%d: %s' % (actor_id, exc))
        return False
    if actor_id in left_actors:
        left_actors.remove(actor_id)
    target_finish = actor_num - len(left_actors)
    score = _progress_score()
    return True


def _process_actor_detection(msg, actor_ids):
    global find_finish, score, time_usage
    if mission_finished or not getattr(msg, 'cls', None):
        return
    now = _now()
    for actor_id in actor_ids:
        if actor_id not in left_actors:
            continue
        position = actors_pos[actor_id]
        previous = topic_arrive_time[actor_id]
        topic_arrive_time[actor_id] = now
        if position is None:
            _reset_detection(actor_id)
            continue
        distance_sq = ((msg.x - position.x) ** 2 + (msg.y - position.y) ** 2)
        continuous = previous == 0.0 or now - previous <= DETECTION_INTERVAL
        if distance_sq >= err_threshold ** 2 or not continuous:
            _reset_detection(actor_id)
            continue
        if not count_flag[actor_id]:
            count_flag[actor_id] = True
            find_time[actor_id] = now
            if actor_id in find_actors:
                find_actors.remove(actor_id)
            find_finish = actor_num - len(find_actors)
            if actor_id < len(find_actor_pub):
                find_actor_pub[actor_id].publish(now)
            score = _progress_score()
            print('find actor_' + str(actor_id))
            _publish_score()
            continue
        if now - find_time[actor_id] < DETECTION_DURATION:
            continue
        if not _delete_actor(actor_id):
            _reset_detection(actor_id)
            continue
        _reset_detection(actor_id)
        print('actor_%d is OK' % actor_id)
        print('Time usage:', time_usage)
        if target_finish == actor_num:
            elapsed = max(0.0, now - start_time)
            score = (2160.0 - elapsed - sensor_cost * 3e-3
                     - uav_loss_count * uav_loss_penalty)
            _finish('Mission finished', score)
        else:
            print('score:', score)
            _publish_score()


def _process_red_detection(msg, flag_name, reset_value):
    """Process a red-target message using the legacy two-target matching.

    Both red topics are allowed to match either actor_4 or actor_5.  The
    callback only clears a pending red detection after both red target
    candidates fail the same message, which is the behavior used by the
    Downloads version of this node.
    """
    global find_finish, score, time_usage, flag_1, flag_2
    if mission_finished or getattr(msg, 'cls', '') != 'red':
        return

    red_cnt = 0
    actor_ids = actor_id_dict['red']
    now = _now()
    for actor_id in actor_ids:
        if actor_id not in left_actors:
            continue

        position = actors_pos[actor_id]
        previous = topic_arrive_time[actor_id]
        topic_arrive_time[actor_id] = now
        valid = (position is not None and
                 ((msg.x - position.x) ** 2 +
                  (msg.y - position.y) ** 2) < err_threshold ** 2 and
                 now - previous < DETECTION_INTERVAL)

        if valid:
            if not count_flag[actor_id]:
                count_flag[actor_id] = True
                find_time[actor_id] = now
                if actor_id in find_actors:
                    find_actors.remove(actor_id)
                find_finish = actor_num - len(find_actors)
                if actor_id < len(find_actor_pub):
                    find_actor_pub[actor_id].publish(now)
                score = _progress_score()
                print('find actor_' + str(actor_id))
                _publish_score()
                if flag_name == 'flag_1':
                    flag_1 = actor_id
                else:
                    flag_2 = actor_id
                continue

            if now - find_time[actor_id] < DETECTION_DURATION:
                continue
            if not _delete_actor(actor_id):
                continue

            _reset_detection(actor_id)
            print('actor_%d is OK' % actor_id)
            print('Time usage:', time_usage)
            if target_finish == actor_num:
                elapsed = max(0.0, now - start_time)
                score = (2160.0 - elapsed - sensor_cost * 3e-3
                         - uav_loss_count * uav_loss_penalty)
                _finish('Mission finished', score)
            else:
                print('score:', score)
                _publish_score()
            continue

        red_cnt += 1

    # Match the Downloads implementation: reset the candidate remembered by
    # this red topic only when both red actors fail this message.
    if red_cnt == len(actor_ids):
        current_flag = flag_1 if flag_name == 'flag_1' else flag_2
        if current_flag != reset_value and current_flag in actor_ids:
            count_flag[current_flag] = False
            if flag_name == 'flag_1':
                flag_1 = reset_value
            else:
                flag_2 = reset_value


def actor_info_callback(msg):
    ids = actor_id_dict.get(getattr(msg, 'cls', ''), [])
    _process_actor_detection(msg, ids)

def actor_info1_callback(msg):
    _process_red_detection(msg, 'flag_1', 0)

def actor_info2_callback(msg):
    _process_red_detection(msg, 'flag_2', 1)

if __name__ == "__main__":
    left_actors = list(range(actor_num))
    find_actors = list(range(actor_num))
    actors_pos = [None] * actor_num
    count_flag = [False] * actor_num
    topic_arrive_time = [0.0] * actor_num
    find_time = [0.0] * actor_num
    find_actor_pub = []
    mission_finished = False
    uav_loss_count = 0
    uav_loss_penalty = DEFAULT_UAV_LOSS_PENALTY
    flag_1 = 0
    flag_2 = 1
    uav_seen = [False] * uav_num
    uav_lost = [False] * uav_num
    uav_miss_count = [0] * uav_num
    rospy.init_node('score_cal')
    time.sleep(1)
    start_time = rospy.get_time()
    del_model = rospy.ServiceProxy("/gazebo/delete_model",DeleteModel)
    get_model_state = rospy.ServiceProxy("/gazebo/get_model_state",GetModelState) 
    score_pub = rospy.Publisher("/score",Int16,queue_size=1) 
    time_usage_pub = rospy.Publisher("/time_usage",Int16,queue_size=1) 
    left_actors_pub = rospy.Publisher("/left_actors",String,queue_size=1)
    actor_blue_sub = rospy.Subscriber("/actor_blue_info",ActorInfo,actor_info_callback,queue_size=1)
    actor_green_sub = rospy.Subscriber("/actor_green_info",ActorInfo,actor_info_callback,queue_size=1)
    actor_white_sub = rospy.Subscriber("/actor_white_info",ActorInfo,actor_info_callback,queue_size=1)
    actor_brown_sub = rospy.Subscriber("/actor_brown_info",ActorInfo,actor_info_callback,queue_size=1)
    actor_red1_sub = rospy.Subscriber("/actor_red1_info",ActorInfo,actor_info1_callback,queue_size=1)
    actor_red2_sub = rospy.Subscriber("/actor_red2_info",ActorInfo,actor_info2_callback,queue_size=1)
    for i in range(actor_num):
        find_actor_pub.append(rospy.Publisher("/find_actor_%d"%i, Float32, queue_size=5))


    mono_cam = int(rospy.get_param('~mono_cam', 1))
    stereo_cam = int(rospy.get_param('~stereo_cam', 0))
    laser1d = int(rospy.get_param('~laser1d', 0))
    laser2d = int(rospy.get_param('~laser2d', 1))
    laser3d = int(rospy.get_param('~laser3d', 0))
    gimbal = int(rospy.get_param('~gimbal', 1))
    bino_cam = int(rospy.get_param('~bino_cam', 0))
    sensor_cost = (mono_cam * 5e2 + stereo_cam * 1e3 + laser1d * 2e2
                   + laser2d * 1e3 + laser3d * 2e3 + gimbal * 2e2
                   + bino_cam * 1e3)


    uav_loss_penalty = float(rospy.get_param('~uav_loss_penalty', DEFAULT_UAV_LOSS_PENALTY))
    target_finish = 0
    find_finish = 0
    score = _progress_score()
    time_usage = 0.0
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        for i in range(uav_num):
            try:
                response = get_model_state(uav_type + '_' + str(i), 'ground_plane')
                success = getattr(response, 'success', True)
                if success:
                    uav_pos_tmp = response.pose.position
                    uav_seen[i] = True
                    uav_miss_count[i] = 0
                    if uav_pos_tmp.z > MAX_UAV_ALTITUDE:
                        score = 0
                        _finish('Warning: UAV %d is higher than 6 meters' % i, 0)
                        break
                elif uav_seen[i] and not uav_lost[i]:
                    uav_miss_count[i] += 1
                    if uav_miss_count[i] >= UAV_MISS_LIMIT:
                        uav_lost[i] = True
                        uav_loss_count += 1
                        score = _progress_score()
                        print('UAV %d lost; penalty %.1f' % (i, uav_loss_penalty))
            except Exception as exc:
                if uav_seen[i] and not uav_lost[i]:
                    uav_miss_count[i] += 1
                    if uav_miss_count[i] >= UAV_MISS_LIMIT:
                        uav_lost[i] = True
                        uav_loss_count += 1
                        score = _progress_score()
                        print('UAV %d lost (%s); penalty %.1f' % (i, exc, uav_loss_penalty))
        if mission_finished:
            break
        for i in list(left_actors):
            try:
                response = get_model_state('actor_' + str(i), 'ground_plane')
                if not getattr(response, 'success', True):
                    continue
                actors_pos_tmp = response.pose.position
                if actors_pos_tmp.x ** 2 + actors_pos_tmp.y ** 2 != 0:
                    actors_pos[i] = actors_pos_tmp
            except Exception:
                continue
        time_usage = rospy.get_time() - start_time
        if time_usage > MISSION_TIMEOUT:
            _finish('Time out, mission failed', score)
            break
        score = _progress_score()
        _publish_score()
        time_usage_pub.publish(int(time_usage))
        left_actors_pub.publish(str(left_actors))
        if cv2 is not None and (os.name == 'nt' or os.environ.get('DISPLAY')):
            try:
                background = cv2.imread(os.path.join(os.path.dirname(__file__), "white_background.png"))
                if background is not None:
                    cv2.putText(background,"Score: "+str(score) ,(60,25),cv2.FONT_HERSHEY_SIMPLEX,0.75,(0,0,0),2)
                    cv2.putText(background,"Time usage: "+str(time_usage) ,(360,25),cv2.FONT_HERSHEY_SIMPLEX,0.75,(0,0,0),2)
                    cv2.putText(background,"Left targets: "+str(left_actors) ,(760,25),cv2.FONT_HERSHEY_SIMPLEX,0.75,(0,0,0),2)
                    cv2.imshow("Multi-UAV search simulation competition judgment system",background)
                    cv2.waitKey(1)
            except Exception:
                pass
        rate.sleep()
