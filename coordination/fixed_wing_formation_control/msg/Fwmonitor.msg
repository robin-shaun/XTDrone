#飞机的控制状态，任务状态的flags，表示任务完成状况，飞机的飞行状态，失联状态，
#飞机控制保护状态

uint8 planeID#飞机编号

bool fw_complete_idel#飞机已经空闲

bool fw_is_connected #飞机链接地面站标志位

bool fw_is_wellctrlled #飞机控制状态标志位

bool fw_complete_takeoff #飞机已经起飞标志位

bool fw_complete_landed #飞机已经降落标志位

bool formation_distance_complete #飞机编队距离已经满足

bool formation_time_complete #飞机编队时间已经满足