#这里也要记得写上planeid，后面得向地面站发送哈！！！！
uint16 planeID

#####################################编队控制器期望状态###############################

#编队期望ned速度
float64 sp_ned_vel_x
float64 sp_ned_vel_y
float64 sp_ned_vel_z

#编队期望经纬度
float64 sp_latitude
float64 sp_longitude
float64 sp_altitude
float64 sp_relative_alt

#编队期望空速地速
float64 sp_air_speed
float64 sp_ground_speed

#####################################编队控制器误差状态###############################

#NED下的位置误差
float64 err_P_N
float64 err_P_E
float64 err_P_D
float64 err_P_NE

#航迹轴系位置误差<与自己期望>
float64 err_PXk
float64 err_PYk
float64 err_PZk

#航迹系速度误差<与领机>
float64 led_fol_vxk
float64 led_fol_vyk
float64 led_fol_vzk

#航迹轴系速度误差<与自己期望>
float64 err_VXk
float64 err_VYk
float64 err_VZk

#角度误差
float64 led_fol_eta
float64 eta_deg

#体轴系位置误差<与自己期望>
float64 err_PXb
float64 err_PYb
float64 err_PZb

#体轴系速度误差<与自己期望>
float64 err_VXb
float64 err_VYb
float64 err_VZb

#体轴系速度误差<与领机>
float64 led_fol_vxb
float64 led_fol_vyb
float64 led_fol_vzb

