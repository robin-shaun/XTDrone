#include "abs_formation_controller.hpp"


void ABS_FORMATION_CONTROLLER::set_ID(int id)
{
    planeID = id;
    ABS_FORMATION_CONTROLLER_INFO("planeID = " << planeID << ";" );
}






/**
 * @Input: void
 * @Output: void
 * @Description: 以便重置控制器中有“记忆”的量
 */
void ABS_FORMATION_CONTROLLER::reset_formation_controller()
{
  rest_tecs = true;
}

/**
 * @Input: void
 * @Output: void
 * @Description: 设定tecs控制器参数
 */
void ABS_FORMATION_CONTROLLER::set_tecs_params(struct _s_tecs_params &input_params)
{
  tecs_params = input_params;
}

/**
 * @brief 僚机控制律，僚机及其对应长机信息传递
 * 
 */

void ABS_FORMATION_CONTROLLER::filter_fw_states()
{
  fw_states_f = fw_states; 
  everyfw_states_f0 = everyfwstates0;
  everyfw_states_f1 = everyfwstates1;
  everyfw_states_f2 = everyfwstates2;
  everyfw_states_f3 = everyfwstates3;
  everyfw_states_f4 = everyfwstates4;
  everyfw_states_f5 = everyfwstates5;
  its_neighbor_scurve[0] = itsneighbor_scurve0;
  its_neighbor_scurve[1] = itsneighbor_scurve1;
  nowtime=currenttime;
  /* 最终传出量everyfw_states_f和fw_states_f */
}
/**
 * @Input: void
 * @Output: void
 * @Description: 长机控制律，调用滤波器对输入的飞机原始状态进行滤波
 */



/**
 * @Input: void
 * @Output: void
 * @Description: 计算从飞机当前位置到期望的位置的向量
 */
Point ABS_FORMATION_CONTROLLER::get_plane_to_sp_vector(Point origin, Point target)
{
  Point out(deg_2_rad((target.x - origin.x)), deg_2_rad((target.y - origin.y) * cosf(deg_2_rad(origin.x))));

  return out * double(CONSTANTS_RADIUS_OF_EARTH);
}

/**
 * @brief 将角度归到[-pi,pi)的区间内
 * 
 */
float ABS_FORMATION_CONTROLLER::RoundAngle(float ang)
{
  float ang1;
  
  while (ang >= PI)
  {
    ang = ang - 2 * PI;
  }
  while (ang < -PI)
  {
     ang = ang + 2 * PI;
  }
  ang1 = ang;

  return ang1;
}
/**
 * @brief z避障算法
 * 
 */
void ABS_FORMATION_CONTROLLER::obstacle_avoidance(float pos_x,float pos_y)
{
  const float k=2;
  is_avoid=false;//初始化避障标志
  Vec f_obs_total;//排斥力汇总
  f_obs_total.set_vec_ele(0.0f,0.0f);
  /*角度关系*/
  Vec havoid_1,havoid_2;
  havoid_1.set_vec_ele(cosf(fw_states_f.yaw_angle),sinf(fw_states_f.yaw_angle));
  havoid_2.set_vec_ele(-sinf(fw_states_f.yaw_angle),cosf(fw_states_f.yaw_angle));
  /*位置、速度关系*/
  Vec rel_pos,rel_pos_normalized;
  float distance,closing_speed,safe_dist;
  Vec uav_vel,rel_vel;
  /*累积计算排斥力*/
  for(int i=1;i<=obstacle_params.obstacle_num;i++)
  {
    
    rel_pos.set_vec_ele(pos_x-obstacle_params.obstacle_pos[0][i],pos_y-obstacle_params.obstacle_pos[1][i]);//相对位移
    distance=rel_pos.len();//与障碍物距离
    rel_pos_normalized=rel_pos.operator/(distance + 1e-5) ;
    //相对速度计算
    uav_vel=fw_gspeed_2d;
    rel_vel=uav_vel.operator-(0.0f);//相对速度
    closing_speed=rel_vel.operator*(rel_pos_normalized);
    if(closing_speed > 0)continue;//远离情况不考虑

    safe_dist=obstacle_params.obstacle_radius+obstacle_params.uav_radius+obstacle_params.staticEdgeDistance+uav_vel.len()/fw_states_f.yaw_rate;
    if(distance<=safe_dist){
      float denominator;
      is_avoid=true;
      denominator=max(distance-obstacle_params.obstacle_radius,0.1);
      Vec f_obs;
      f_obs=rel_pos.operator*((safe_dist-distance)/denominator/distance);
      f_obs_total=f_obs_total.operator+(f_obs);
    }
  }
  if(is_avoid){
    vel_cmd=sat_function(uav_vel.len()+havoid_1.operator*(f_obs_total),fw_params.vmin, fw_params.vmax);
    angular_rate_cmd=sat_function(fw_states_f.yaw_rate+havoid_2.operator*(f_obs_total),-0.5, 0.5);
  }
}
//*********************************************************************
//***基于导航律的Unicycle模型的方法，这里期望速度直接用水平二维速度，事先飞到统一高度，再用TECS得到pitch与油门，roll根据偏航角速度，由BTT条件得到
//***10.23;问题：当从机在主机前面的时候，从机减速，等待主机追过他去(前提是航向一致)，这影响了效果？
//***QGC南向飞的时候效果不好，应该不是TECS的问题，因为起飞航线直接设置为南向直线，而不是转弯进入南向直线效果也不好(角度计算有误?)，并且这时候感觉制律产生的结果也不行
//***已解决 规整误差（-PI,PI）
/* 僚机跟随长机控制律 ，长机和僚机用同一个fwstates描述，只不过增加编号信息来区分长僚机*/
void ABS_FORMATION_CONTROLLER::control_law()
{
  //***第一阶段：规整速度、角速度和GPS信息
  //***第二阶段：定义control（）内部所需输入变量fw_states_f 、its_neighbor_f 
  //***第三阶段：定义最终输出控制信号vel_2D_sp = 30m/s, angular_rate_sp, temp
  //***第四阶段：TECS转换,通过tecs类获得最终的控制量_cmd.pitch俯仰角, _cmd.thrust油门, _cmd.roll滚转角
  //***输入fw_states_f
  //***中间控制律(19)(20)输出: 航向角速度angular_rate_sp、线速度(2D)vel_2D_sp
  //***最终输出: _cmd.pitch俯仰角, _cmd.thrust油门, _cmd.roll滚转角
  long now = get_sys_time();

  /* 控制时间间隔 */
  _dt = constrain((now - abs_pos_vel_ctrl_timestamp) * 1.0e-3f, _dtMin, _dtMax);
  /* 对无人机信息滤波 */
  filter_fw_states();
  /*判断无人机起飞状态，若没则警告
  if(!identify_led_fol_states())
  {
    ABS_FORMATION_CONTROLLER_INFO("警告：领机或从机未在飞行中，无法执行编队控制");
    return;
  }*/

  /* 不考虑风速了 */
  float fw_airspd_x =  fw_states_f.global_vel_x;
  float fw_airspd_y =  fw_states_f.global_vel_y;


  /*###########################第一阶段：规整速度、角速度和GPS信息#######################*/

  //***领机地速向量
  fw_gspeed_2d.set_vec_ele(fw_states_f.global_vel_x, fw_states_f.global_vel_y);

  //***长机空速向量，读取的空速
  fw_arispd.set_vec_ele(fw_airspd_x, fw_airspd_y); //***2D

 //***判断读取空速与计算空速的差距，差距太大不合法
  if((fw_arispd.len() - fw_states_f.air_speed) >= 3.0)
  {
    fw_airspd_states_valid = false; //不合法
  }
  else
  {
    fw_airspd_states_valid = true; //合法
  }

  float fw_yaw_angle; //长机偏航角

  //***地速大小判别 根据地速判别当前航向角(风的影响?),地速分量有正负
  //***计算领机航向(全局坐标系下)  根据论文(-pi, pi]

  if(fw_gspeed_2d.len() <= 3.0)//空速小于3m/s时
  {
    if(fw_states_f.yaw_valid)  //true能得到领机信息标识
    {
      fw_yaw_angle = fw_states_f.yaw_angle;
    }
    else                      //false不能得到领机信息标识
    {
      fw_yaw_angle = 0;
    }
  }
  else   //***由x和y向的地速来计算当前航向角，航向角速度使用imu获得的
  {          //***角度应该也可以直接使用imu传感器获得的
    if(fw_gspeed_2d.x >0 && fw_gspeed_2d.y >0)                                      
    {
      fw_yaw_angle = atanf(fw_gspeed_2d.y / fw_gspeed_2d.x); 
    }
    else if(fw_gspeed_2d.x >0 && fw_gspeed_2d.y < 0)
    {
      fw_yaw_angle = atanf(fw_gspeed_2d.y / fw_gspeed_2d.x);
    }
    else if (fw_gspeed_2d.x <0 && fw_gspeed_2d.y >0)
    {
      fw_yaw_angle = atanf(fw_gspeed_2d.y / fw_gspeed_2d.x) + PI;
    }
    else if (fw_gspeed_2d.x <0 && fw_gspeed_2d.y <0)
    {
      fw_yaw_angle = atanf(fw_gspeed_2d.y / fw_gspeed_2d.x) - PI;
    }
    else if (fw_gspeed_2d.x == 0 && fw_gspeed_2d.y <0)
    {
      fw_yaw_angle = -PI/2;
    }
    else if (fw_gspeed_2d.x ==0 && fw_gspeed_2d.y >0)
    {
      fw_yaw_angle = PI/2;
    }
    else if (fw_gspeed_2d.x >0 && fw_gspeed_2d.y == 0)
    {
      fw_yaw_angle = 0;
    }
    else if (fw_gspeed_2d.x <0 && fw_gspeed_2d.y ==0)  //*** -PI和PI应该一样，按照论文为PI
    {
      fw_yaw_angle = PI;
    }
    else
    {
      fw_yaw_angle = 0;
      ABS_FORMATION_CONTROLLER_INFO("领机航向角计算有误");
    }
    ABS_FORMATION_CONTROLLER_INFO("领机地速正常，选用领机地速航向角");   //***这样也对应论文里的全局系了吧
  }

  //***其他输入领从机的角速度，线速度，位置(全局系下，这里直接使用GPS了)
  //***计算全局误差信息
  //***这里角度误差的规整要调整一下，看试验结果是当两个飞机角度一正一负的时候，从机角度的调整方向偏向于大的方向

  //***长机位置坐标转换
  //***长机当前位置二维数组[纬度,经度]
  double l_pos[2]; 
  
  l_pos[0] = fw_states_f.latitude;
  l_pos[1] = fw_states_f.longitude;
  ABS_FORMATION_CONTROLLER_INFO("长机编号uav"<<fw_states_f.planeID<<";"<<"纬度 = "<< l_pos[0] <<";"<<"经度"<<l_pos[1]<<";");
  /*经纬度原点[纬度，经度],根据QGC进行原点设置*/
  double origin[2]; 
  /*当前长机位置坐标为[x,y]*/
  double leader_local[2];
  double result[2];

  origin[0] = 47.3977419;
  origin[1] = 8.5455946;
  /*GPS经纬度坐标转换到NED距离*/
  cov_lat_long_2_m(origin,l_pos,result); 
  
  leader_local[0] = result[0];//y
  leader_local[1] = result[1];//x
  ABS_FORMATION_CONTROLLER_INFO("长机编号uav"<<fw_states_f.planeID<<";"<<"x = "<< leader_local[0] <<";"<<"y"<<leader_local[1]<<";");

  /* 目标路径弧长参数 */
  double scurve, nb_scurve[2], zeta{0};
  planeID = fw_states_f.planeID;
  
  scurve = fw_states_f.scurve;
  for(int i=0;i<2;i++){
    nb_scurve[i] = its_neighbor_scurve[i];
  };   
  





  //***利用目标路径弧长参数scurve设置虚拟目标点坐标tarpos[3]
  /*长机虚拟目标点位置二维数组[x,y,ang]*/
  double tarpos[3];         /*虚拟目标点坐标tarpos[3]*/
  double inity{-300};       /*第一架长机路径初始纵坐标-300*/
  double leader_inter{100}; /*相邻长机间隔x方向间隔*/
  double dis_inter{30};     /*相邻僚机间隔*/
  
  if (nowtime > 350) leader_inter=50;

 ABS_FORMATION_CONTROLLER_INFO("当前时间:"<< nowtime<<";"<<"leader_inter="<<leader_inter);
  /*三角形用于leader-follower仿真，对应multi_uav_sim_ba_6vtol.sh*/

  //三角形，用于僚机编队跟踪长机整体仿真
  tarpos[0] = scurve / sqrtf(2);
  if(planeID==0)tarpos[1] = scurve / sqrtf(2) + inity + leader_inter;//y0=x-200
  if(planeID==1)tarpos[1] = scurve / sqrtf(2) + inity ;//y1=x-300
  if(planeID==2)tarpos[1] = scurve / sqrtf(2) + inity + 2 * leader_inter;//y2=x-100
  tarpos[2] = PI / 4;
  //长机协同变量zeta计算
  for(int i=0;i<2;i++){                                             
    if(planeID==0)zeta = zeta - nb_scurve[i] + scurve - leader_inter / sqrt(2);
    if(planeID==1)zeta = zeta - nb_scurve[i] + scurve;
    if(planeID==2)zeta = zeta - nb_scurve[i] + scurve + 2 * leader_inter / sqrt(2);
   }


  /*一字形仅用于leader协同仿真，对应multi_uav_sim_ba_3vtol_leader.sh*/
  /*注释从此处开始 
  //一字型，仅用于长机协同仿真
  tarpos[0] = scurve / sqrtf(2);
  tarpos[1] = scurve / sqrtf(2) + inity + planeID * leader_inter;
  tarpos[2] = PI / 4;

  //长机协同变量zeta计算
  for(int i=0;i<2;i++){                                             
    zeta = zeta - nb_scurve[i] + scurve;
  } 
  */

  //***误差动力学模型,转到Frenet-serret坐标系
  // 北、东方向误差，航向角误差，公式（4）
  float es, ed, psi; 

  es = (leader_local[0] - tarpos[0]) * cosf(tarpos[2]) + (leader_local[1] - tarpos[1]) * sinf(tarpos[2]);//计算侧向误差
  ed = -(leader_local[0] - tarpos[0]) * sinf(tarpos[2]) + (leader_local[1] - tarpos[1]) * cosf(tarpos[2]);//计算纵向误差

  //***无人机航向与期望路径切线方向夹角，即航向角误差psi,规整角度误差到(-pi,pi)上 ,相当于MATLAB的RoundAngle函数
  psi = fw_yaw_angle - tarpos[2];
  //***这步是很有必要的：从机的控制有时候会有一定的滞后，所以当主机出现正负PI/2时，从机若没有及时跟上则会影响精度，这是因为导航律设计的原因(优先逆时针调整), 这样可以一定程度上减缓这种现象

  //***导航律的设计会优先向逆时针方向转
  psi = RoundAngle(psi);

  //***设置dotL ,求scurve计算下一个虚拟目标点位置 
  float dotL;/* 弧长的导数 */
  dotL = control_law_params.gammad - control_law_params.beta * tanhf(control_law_params.ktheta * zeta);
  
  //第二阶段 定义长机控制律内部所需输入变量
  //*** 产生控制信号：2D线速度,航向角速度,控制律temp用于饱和函数
  float vel_2D_sp, angular_rate_sp, temp;

  //***求解速度，注意速度正负 在论文里边速度始终是正值
  vel_2D_sp = (dotL - control_law_params.ks * es) / cosf(psi);
  //***这里速度限幅在12~25之间
  vel_2D_sp = sat_function(vel_2D_sp, fw_params.vmin, fw_params.vmax);
  //*** 根据限幅后的线速度更新dotL
  dotL = vel_2D_sp * cosf(psi) + control_law_params.ks * es ;
  ABS_FORMATION_CONTROLLER_INFO("理论传入的长机信息的编号planeID = "<< planeID <<";");
  ABS_FORMATION_CONTROLLER_INFO("实际传入的长机信息的编号planeID = "<< fw_states_f.planeID <<";"); 
  ABS_FORMATION_CONTROLLER_INFO("实际传入的长机邻居的弧长参数 = "<< its_neighbor_scurve[0] <<";"<< its_neighbor_scurve[1] <<";"); 
  ABS_FORMATION_CONTROLLER_INFO("################################ 控制律参数与中间输出 ################################");
  ABS_FORMATION_CONTROLLER_INFO("纵向误差es:" << es << ";" << "侧向误差ed:" << ed << ";"<<"航向角误差psi"<< psi <<";");
  ABS_FORMATION_CONTROLLER_INFO("目标路径曲线弧长参数scurve: " << scurve << ";" );
  ABS_FORMATION_CONTROLLER_INFO("本长机邻居弧长参数nb_scurve: " << nb_scurve[0] << ";"<< nb_scurve[1] << ";" );  
  ABS_FORMATION_CONTROLLER_INFO("长机平面内ENU位置:x = " << leader_local[0] << ";" <<"y = "<< leader_local[1] << ";");
  ABS_FORMATION_CONTROLLER_INFO("虚拟目标点位置x = " <<tarpos[0]<<";"<<"y = "<<tarpos[1] << ";" <<"angle = "<< tarpos[2]<<";"); 
  
  //***看一下长机垂直速度的变化，对期望速度有多大的影响
  
  ABS_FORMATION_CONTROLLER_INFO("长机z方向的速度: " << fw_states_f.global_vel_z);  //***gps的速度消息得不到z轴的速度信息



  //***所需输入量
  //***-->长机偏航角速度
  float  fw_yaw_rate;
  //***-->长机空速标量vL，正值
  float  fw_2D_vel;

 fw_yaw_rate = fw_states_f.yaw_rate;

 fw_2D_vel = fw_gspeed_2d.len();  //***使用地速来表示当前速度 都要为正值

  ABS_FORMATION_CONTROLLER_INFO("长机速度2D_vel= " << fw_2D_vel << ";" << "偏航角度yaw_angle= "<< fw_yaw_angle <<";");
 //***发现问题：yaw_rate为0 已解决；

  



 
 //***求解角速度
 //***omegamax=0.5,omegamin = -0.5
 float deta, s, kappa;
 kappa = 0;/*直线的曲率为0*/
 deta = vel_2D_sp * sinf(psi) - kappa*es*dotL;
 s = psi +  control_law_params.kpi * tanhf(control_law_params.kd * ed);

 ABS_FORMATION_CONTROLLER_INFO("控制律滑模控制参数deta = " << deta << ";" <<"s = "<< s << ";");
 angular_rate_sp = -control_law_params.komega * s + kappa * dotL - control_law_params.kpi * control_law_params.kd * deta * powf(sinhf(control_law_params.kd * ed),2) ;
 angular_rate_sp = sat_function(angular_rate_sp, -0.5, 0.5);  //最大航向角速度为1.2rad/s, 原来限幅出错了，角速度是有正负的，不能为0
 ABS_FORMATION_CONTROLLER_INFO("长机速度2D_vel= " << vel_2D_sp << ";" << "角速度yaw_rate= "<< angular_rate_sp <<";");
 fw_sp.scurve = dotL * _dt + scurve;
 
 /*把求得的虚拟目标点弧长赋值给_cmd*/
  _cmd.scurve = fw_sp.scurve;     /* 注意此处已经对_cmd中的scurve赋值 */

  //避障控制 
 obstacle_avoidance(leader_local[0],leader_local[1]);
 if (is_avoid){
    vel_2D_sp=vel_cmd;
    angular_rate_sp=angular_rate_cmd;
    ABS_FORMATION_CONTROLLER_INFO("碰撞规避长机速度2D_vel= " << vel_2D_sp << ";" << "角速度yaw_rate= "<< angular_rate_sp <<";");
  }

 //#############################控制律输出量#######################################
 airspd_sp_prev = airspd_sp;  //***待定
 fw_sp.air_speed = vel_2D_sp;

 //***TECS得到期望俯仰角和油门(直接把2D速度作期望值了)，不行的话就添加一个高度速度补偿看看
 //***想法是，通过PID由高度误差得到期望pitch，然后期望速度由2D速度通过三角函数变换为3D速度
 //***或者由高度误差通过PID产生垂直速度分量，然后变为期望空速
 if (rest_tecs) //重置TECS
 {
  rest_tecs = false; //重置后标志位设置为不重置
  _tecs.reset_state();
 }
//***参数可能要进一步调整
//***tecs可能有点问题，减速到最低速度的时候油门降的很慢
 _tecs.set_speed_weight(tecs_params.speed_weight);
 _tecs.set_time_const_throt(tecs_params.time_const_throt); //***影响油门，越大，kp越小
 _tecs.set_time_const(tecs_params.time_const); //***影响pitch， 越大，kp越小
 _tecs.enable_airspeed(true);

  //***发现问题，fw_sp.altitude没有正确传递进来，已解决！
//  fw_sp.altitude = fw_states_f.altitude + formation_offset.zb;
 fw_sp.altitude = 50; //***试试直接设定期望飞行高度为100 不为主机高度(不知道为什么，设置航点高度100，QGC显示也是100，GPS高度为147左右(应该用气压计))
 ABS_FORMATION_CONTROLLER_INFO("长机期望高度, 长机当前高度: " << fw_sp.altitude << ";" <<  fw_states_f.altitude );
 if(fw_sp.altitude - fw_states_f.altitude >= 10)  //***判断是否需要爬升 只判断什么时候开始爬升
 {
   tecs_params.climboutdem = true;
   ABS_FORMATION_CONTROLLER_INFO("爬升");
 }
 else
 {
   tecs_params.climboutdem = false;
   ABS_FORMATION_CONTROLLER_INFO("不爬升");
 }

 //***爬升高度很大时的垂直方向的速度补偿，失速也会掉高, 试验中发现，当爬升高度较大时TECS也会自动的做一个补偿(不过很容易接近最大速度)，这里就当双保险吧，去掉也不影响
//  airspd_sp = vel_2D_sp +  height_to_speed(fw_sp.altitude, fw_states_f.altitude, fw_states_f.pitch_angle); 
//  fw_sp.air_speed = sat_function(airspd_sp, fw_params.vmin, fw_params.vmax);
//  ABS_FORMATION_CONTROLLER_INFO("限幅空速设定值: " << fw_sp.air_speed);

 _tecs.update_vehicle_state_estimates(fw_states_f.air_speed, fw_states_f.rotmat, fw_states_f.body_acc,
                                    fw_states_f.altitude_lock, fw_states_f.in_air, fw_states_f.altitude,
                                    vz_valid, fw_states_f.ned_vel_z, fw_states_f.body_acc[2]);
 
//  fw_sp.air_speed = fw_params.vmax;   //***直接给最大速度，看看pitch效果, 为什么爬升很慢呢，这里应该和之前方案没区别阿，roll的影响？已解决！
 _tecs.update_pitch_throttle(fw_states_f.rotmat, fw_states_f.pitch_angle, fw_states_f.altitude,
                                       fw_sp.altitude, fw_sp.air_speed, fw_states_f.air_speed,
                                       tecs_params.EAS2TAS, tecs_params.climboutdem, tecs_params.climbout_pitch_min_rad,
                                       fw_params.throttle_min, fw_params.throttle_max, fw_params.throttle_cruise,
                                       fw_params.pitch_min_rad,fw_params.pitch_max_rad);

 _cmd.pitch = _tecs.get_pitch_setpoint();
 _cmd.thrust = _tecs.get_throttle_setpoint();

 //***BTT产生期望滚转角
 //***是否需要补偿滚转引起的掉高呢？不加效果倒也可以(稳定后高度下降不明显)，暂时先不加了
 roll_cmd = atanf((angular_rate_sp * fw_2D_vel) / CONSTANTS_ONE_G);

 roll_cmd = constrain(roll_cmd, -fw_params.roll_max, fw_params.roll_max);

 _cmd.roll = roll_cmd;

 _cmd.planeID = planeID;
 //ABS_FORMATION_CONTROLLER_INFO("################################ 控制律最终输出 ################################");
 ABS_FORMATION_CONTROLLER_INFO("最终期望俯仰角：" << _cmd.pitch);
 ABS_FORMATION_CONTROLLER_INFO("最终期望油门：" << _cmd.thrust);
 ABS_FORMATION_CONTROLLER_INFO("最终期望滚转角：" << _cmd.roll);
 ABS_FORMATION_CONTROLLER_INFO("领机当前俯仰角(度)和翻滚角: "<<rad_2_deg( fw_states_f.pitch_angle) << ";" <<rad_2_deg(fw_states_f.roll_angle));//***看一下无人机的当前状态

 //***误差记录，只是用于画图，可通过rosbag记录
 //***航向角误差
 fw_error.planeID = planeID;
 fw_error.led_fol_eta = psi;
 //***Frenet-serret坐标系位置误差
 fw_error.PXb = es;
 fw_error.PYb = ed;
 fw_error.PZb = fw_states_f.altitude - 50;
 //***水平线速度误差
 fw_error.Vb = fw_2D_vel - vel_2D_sp;
 //VXb = fw_gspeed_2d.x-;  //***x系速度误差
 //fw_error.VYb = fw_gspeed_2d.y-; //***y系速度误差
 //***角速度误差
 fw_error.Vk = fw_yaw_rate - angular_rate_sp;
}













void ABS_FORMATION_CONTROLLER::follower_control_law()
{
  //***第一阶段：规整长机和僚机的速度、角速度和GPS信息
  //***第二阶段：定义control（）内部所需输入变量，传入rel_xd,rel_yd；everyfw_states_f和fw_states_f
  //***第三阶段：定义最终输出控制信号vel_2D_sp , angular_rate_sp
  //***第四阶段：TECS转换,通过tecs类获得最终的控制量_cmd.pitch俯仰角, _cmd.thrust油门, _cmd.roll滚转角
  //***输入everyfw_states_f、fw_states_f
  //***中间控制律(19)(20)输出: 航向角速度angular_rate_sp、线速度(2D)vel_2D_sp
  //***最终输出: _cmd.pitch俯仰角, _cmd.thrust油门, _cmd.roll滚转角
  long now = get_sys_time();

  /* 对所有无人机信息滤波 */
  filter_fw_states();
  /*判断无人机起飞状态，若没则警告
  if(!identify_led_fol_states())
  {
    ABS_FORMATION_CONTROLLER_INFO("警告：领机或从机未在飞行中，无法执行编队控制");
    return;
  }*/
  /*###########################第一阶段：规整速度、角速度和GPS信息#######################*/

  /* 不考虑风速了 */
  float everyfw_states_f_airspd_x0,everyfw_states_f_airspd_y0;
  float everyfw_states_f_airspd_x1,everyfw_states_f_airspd_y1;
  float everyfw_states_f_airspd_x2,everyfw_states_f_airspd_y2;

  everyfw_states_f_airspd_x0=everyfw_states_f0.global_vel_x;
  everyfw_states_f_airspd_y0=everyfw_states_f0.global_vel_y;
  everyfw_states_f_airspd_x1=everyfw_states_f1.global_vel_x;
  everyfw_states_f_airspd_y1=everyfw_states_f1.global_vel_y;
  everyfw_states_f_airspd_x2=everyfw_states_f2.global_vel_x;
  everyfw_states_f_airspd_y2=everyfw_states_f2.global_vel_y;
  
  //***领机地速向量
  lead_gspeed_2d0.set_vec_ele(everyfw_states_f0.global_vel_x, everyfw_states_f0.global_vel_y);
  lead_gspeed_2d1.set_vec_ele(everyfw_states_f1.global_vel_x, everyfw_states_f1.global_vel_y);
  lead_gspeed_2d2.set_vec_ele(everyfw_states_f2.global_vel_x, everyfw_states_f2.global_vel_y);
  //***领机空速向量
  lead_arispd0.set_vec_ele(everyfw_states_f_airspd_x0, everyfw_states_f_airspd_y0); //***2D
  lead_arispd1.set_vec_ele(everyfw_states_f_airspd_x1, everyfw_states_f_airspd_y1); //***2D
  lead_arispd2.set_vec_ele(everyfw_states_f_airspd_x2, everyfw_states_f_airspd_y2); //***2D

  



 //***判断读取空速与计算空速的差距
  if((lead_arispd0.len() - everyfw_states_f0.air_speed) >= 3.0&&(lead_arispd1.len() - everyfw_states_f1.air_speed) >= 3.0&&(lead_arispd2.len() - everyfw_states_f2.air_speed) >= 3.0)
  {
    itsled_airspd_states_valid = false;
  }
  else
  {
    itsled_airspd_states_valid = true;
  }

  float lead_yaw_angle0,lead_yaw_angle1,lead_yaw_angle2; /* leader无人机偏航角 */
  //***地速大小判别 根据地速判别当前航向角(风的影响?)
  //***地速分量有正负
  //***计算无人机航向(全局坐标系下)  根据论文(-pi, pi]
  lead_yaw_angle0 = everyfw_states_f0.yaw_angle;
  lead_yaw_angle1 = everyfw_states_f1.yaw_angle;
  lead_yaw_angle2 = everyfw_states_f2.yaw_angle;
  
  
  /*###########################第一阶段：规整僚机速度、角速度和GPS信息#######################*/
  //***计算从机航向角
  float fw_airspd_x =  fw_states_f.global_vel_x;
  float fw_airspd_y =  fw_states_f.global_vel_y;

  //***从机地速向量
  fw_gspeed_2d.set_vec_ele(fw_states_f.global_vel_x, fw_states_f.global_vel_y);

  //***从机空速向量
  fw_arispd.set_vec_ele(fw_airspd_x, fw_airspd_y); //***2D

 //***判断读取空速与计算空速的差距
  if((fw_arispd.len() - fw_states_f.air_speed) >= 3.0)
  {
    fw_airspd_states_valid = false;
  }
  else
  {
    fw_airspd_states_valid = true;
  }


  float fw_yaw_angle;
  //***地速大小判别 根据地速判别当前航向角(不考虑风)
  //***地速分量有正负
  //***计算领机航向(全局坐标系下)  根据论文(-pi, pi]
  if(fw_gspeed_2d.len() <= 3.0)
  {
    if(fw_states_f.yaw_valid)  //***这句还未修改vir_sim_leader 暂时没用
    {
      fw_yaw_angle = fw_states_f.yaw_angle;
    }
    else
    {
      fw_yaw_angle = 0;
    }
  }
  else   //***由x和y向的地速来计算当前航向角，航向角速度使用imu获得的
  {          //***角度应该也可以直接使用imu获得的
    if(fw_gspeed_2d.x >0 && fw_gspeed_2d.y >0)
    {
      fw_yaw_angle = atanf(fw_gspeed_2d.y / fw_gspeed_2d.x);
    }
    else if(fw_gspeed_2d.x >0 && fw_gspeed_2d.y < 0)
    {
      fw_yaw_angle = atanf(fw_gspeed_2d.y / fw_gspeed_2d.x);
    }
    else if (fw_gspeed_2d.x <0 && fw_gspeed_2d.y >0)
    {
      fw_yaw_angle = atanf(fw_gspeed_2d.y / fw_gspeed_2d.x) + PI;
    }
    else if (fw_gspeed_2d.x <0 && fw_gspeed_2d.y <0)
    {
      fw_yaw_angle = atanf(fw_gspeed_2d.y / fw_gspeed_2d.x) - PI;
    }
    else if (fw_gspeed_2d.x == 0 && fw_gspeed_2d.y <0)
    {
      fw_yaw_angle = -PI/2;
    }
    else if (fw_gspeed_2d.x ==0 && fw_gspeed_2d.y >0)
    {
      fw_yaw_angle = PI/2;
    }
    else if (fw_gspeed_2d.x >0 && fw_gspeed_2d.y == 0)
    {
      fw_yaw_angle = 0;
    }
    else if (fw_gspeed_2d.x <0 && fw_gspeed_2d.y ==0)  //*** -PI和PI应该一样  原为PI
    {
      fw_yaw_angle = -PI;
    }
    else
    {
      fw_yaw_angle = 0;
      ABS_FORMATION_CONTROLLER_INFO("从机航向角计算有误");
    }
    ABS_FORMATION_CONTROLLER_INFO("从机地速正常，选用从机地速航向角");   //***这样也对应了全局系
  }

  //***其他输入领从机的角速度，线速度，位置(全局系下，这里直接使用GPS了)
  //***计算全局误差信息
  //***这里角度误差的规整要调整一下，看试验结果是当两个飞机角度一正一负的时候，从机角度的调整方向偏向于大的方向
  planeID = fw_states_f.planeID;





  //***位置坐标转换
  //***当前位置二维数组[纬度,经度]
  double everyfw_pos[2][control_law_params.NumUAV],fw_pos[2]; 
  everyfw_pos[0][0] = everyfw_states_f0.latitude;
  everyfw_pos[1][0] = everyfw_states_f0.longitude;
  everyfw_pos[0][1] = everyfw_states_f1.latitude;
  everyfw_pos[1][1] = everyfw_states_f1.longitude;
  everyfw_pos[0][2] = everyfw_states_f2.latitude;
  everyfw_pos[1][2] = everyfw_states_f2.longitude;
  everyfw_pos[0][3] = everyfw_states_f3.latitude;
  everyfw_pos[1][3] = everyfw_states_f3.longitude;
  everyfw_pos[0][4] = everyfw_states_f4.latitude;
  everyfw_pos[1][4] = everyfw_states_f4.longitude;
  everyfw_pos[0][5] = everyfw_states_f5.latitude;
  everyfw_pos[1][5] = everyfw_states_f5.longitude;
  

  fw_pos[0] = fw_states_f.latitude;
  fw_pos[1] = fw_states_f.longitude;
  //ABS_FORMATION_CONTROLLER_INFO("僚机编号uav"<<fw_states_f.planeID<<";"<<"纬度 = "<< fw_pos[0] <<";"<<"经度"<<fw_pos[0]<<";");
  /*经纬度原点[纬度，经度],根据QGC进行原点设置*/
  double origin[2]; 
  /*当前长机，僚机位置坐标为[x,y]*/
  double everyfw_local[2][control_law_params.NumUAV], fw_local[2], everyfw_pos1[2],result1[2], result2[2];

  origin[0] = 47.3977419;
  origin[1] = 8.5455946;

  /*长机二维位置：GPS经纬度坐标转换到NED距离*/
  for(int i=0;i<control_law_params.NumUAV;i++){
    everyfw_pos1[0]=everyfw_pos[0][i];
    everyfw_pos1[1]=everyfw_pos[1][i];
    cov_lat_long_2_m(origin,everyfw_pos1,result1); 
    everyfw_local[0][i] = result1[0];
    everyfw_local[1][i] = result1[1];
  }


  /*僚机二维位置：GPS经纬度坐标转换到NED距离*/
  cov_lat_long_2_m(origin,fw_pos,result2); 

  fw_local[0] = result2[0];
  fw_local[1] = result2[1];
 

  ABS_FORMATION_CONTROLLER_INFO("僚机编号uav"<<fw_states_f.planeID<<";"<<"二维位置x = " << fw_local[0] << ";" << "y = "<<fw_local[1]);
  ABS_FORMATION_CONTROLLER_INFO("长机编号uav"<<everyfw_states_f0.planeID<<";"<<"二维位置x = " << everyfw_local[0][0] << ";" << "y = "<<everyfw_local[1][0]);
  ABS_FORMATION_CONTROLLER_INFO("长机编号uav"<<everyfw_states_f1.planeID<<";"<<"二维位置x = " << everyfw_local[0][1] << ";" << "y = "<<everyfw_local[1][1]);
  ABS_FORMATION_CONTROLLER_INFO("长机编号uav"<<everyfw_states_f2.planeID<<";"<<"二维位置x = " << everyfw_local[0][2] << ";" << "y = "<<everyfw_local[1][2]);
 
  //***所需输入量
  //***-->长僚机偏航角速度
  float  leader_yaw_rate[control_law_params.NumLeader];
  float  fw_yaw_rate;
  //***-->长僚机空速标量vL，正值
  float  leader_2D_vel[control_law_params.NumLeader];
  float  fw_2D_vel;
  leader_yaw_rate[0] = everyfw_states_f0.yaw_rate;
  leader_yaw_rate[1] = everyfw_states_f1.yaw_rate;
  leader_yaw_rate[2] = everyfw_states_f2.yaw_rate;
  leader_2D_vel[0] = lead_gspeed_2d0.len();  //***使用地速来表示当前速度 都要为正值
  leader_2D_vel[1] = lead_gspeed_2d1.len();  //***使用地速来表示当前速度 都要为正值
  leader_2D_vel[2] = lead_gspeed_2d2.len();  //***使用地速来表示当前速度 都要为正值

  
  fw_yaw_rate = fw_states_f.yaw_rate;
  
  fw_2D_vel = fw_gspeed_2d.len();

  ABS_FORMATION_CONTROLLER_INFO("长机速度2D_vel[0]= " << leader_2D_vel[0] << ";" << "偏航角速度yaw_rate[0]= "<< leader_yaw_rate[0] <<";");
  ABS_FORMATION_CONTROLLER_INFO("长机速度2D_vel[1]= " << leader_2D_vel[1] << ";" << "偏航角速度yaw_rate[1]= "<< leader_yaw_rate[1] <<";");
  ABS_FORMATION_CONTROLLER_INFO("长机速度2D_vel[2]= " << leader_2D_vel[2] << ";" << "偏航角速度yaw_rate[2]= "<< leader_yaw_rate[2] <<";");
  ABS_FORMATION_CONTROLLER_INFO("当前是僚机控制律################################ 1控制律误差动力学模型 ################################");
  ABS_FORMATION_CONTROLLER_INFO("僚机平面内ENU位置:x = " << fw_local[0] << ";" <<"y = "<< fw_local[1] << ";");

  //***看一下长机垂直速度的变化，对期望速度有多大的影响
  // ABS_FORMATION_CONTROLLER_INFO("僚机z方向的速度: " << fw_states_f.global_vel_z);  //***gps的速度消息得不到z轴的速度信息

  //***控制律相关参数计算
  /*无人机相关角度计算*/
  float h_1[2][control_law_params.NumUAV],h_l[control_law_params.NumUAV],h_2[2][control_law_params.NumUAV];
  h_1[0][0] = cosf(everyfw_states_f0.yaw_angle);
  h_1[1][0] = sinf(everyfw_states_f0.yaw_angle);
  h_1[0][1] = cosf(everyfw_states_f1.yaw_angle);
  h_1[1][1] = sinf(everyfw_states_f1.yaw_angle);
  h_1[0][2] = cosf(everyfw_states_f2.yaw_angle);
  h_1[1][2] = sinf(everyfw_states_f2.yaw_angle);
  h_1[0][3] = cosf(everyfw_states_f3.yaw_angle);
  h_1[1][3] = sinf(everyfw_states_f3.yaw_angle);
  h_1[0][4] = cosf(everyfw_states_f4.yaw_angle);
  h_1[1][4] = sinf(everyfw_states_f4.yaw_angle);
  h_1[0][5] = cosf(everyfw_states_f5.yaw_angle);
  h_1[1][5] = sinf(everyfw_states_f5.yaw_angle);


  h_2[0][0] = -sinf(everyfw_states_f0.yaw_angle);
  h_2[1][0] =  cosf(everyfw_states_f0.yaw_angle);
  h_2[0][1] = -sinf(everyfw_states_f1.yaw_angle);
  h_2[1][1] =  cosf(everyfw_states_f1.yaw_angle);
  h_2[0][2] = -sinf(everyfw_states_f2.yaw_angle);
  h_2[1][2] =  cosf(everyfw_states_f2.yaw_angle);
  h_2[0][3] = -sinf(everyfw_states_f3.yaw_angle);
  h_2[1][3] =  cosf(everyfw_states_f3.yaw_angle);
  h_2[0][4] = -sinf(everyfw_states_f4.yaw_angle);
  h_2[1][4] =  cosf(everyfw_states_f4.yaw_angle);
  h_2[0][5] = -sinf(everyfw_states_f5.yaw_angle);
  h_2[1][5] =  cosf(everyfw_states_f5.yaw_angle);

  for(int i=0;i<control_law_params.NumLeader;i++){
    h_l[2*i] = h_1[0][i];
    h_l[2*i+1] = h_1[1][i];
  };
  

 /*领机相关速度计算*/
  float v_l[control_law_params.NumLeader]={0.0f};
  float v_l2[6][6]={0.0f};
  float v_l1[6]={0.0f};

  for(int i=0;i<control_law_params.NumLeader;i++){
    v_l[i] = leader_2D_vel[i];/*三架领机地速*/
  };

  for (int i = 0; i < control_law_params.NumLeader; i++) {   /*填充对角线元素*/
    v_l2[2*i][2*i] = v_l[i];
    v_l2[2*i+1][2*i+1] = v_l[i];
  };

  for(int i=0;i<6;i++){
    for(int j=0;j<6;j++){
      v_l1[i] = v_l1[i]+v_l2[i][j] * h_l[j];
    };
  };
  //***用于检查
  //***ABS_FORMATION_CONTROLLER_INFO("h_1 " << h_1[0][0] << ";" << h_1[1][0] <<";"<< h_1[0][1]<<";"<< h_1[1][1] <<";"<<h_1[0][2]<<";"<<h_1[1][2]<<";"<<h_1[0][3]<<";"<<h_1[1][3]<<";"<<h_1[0][4]<<";"<<h_1[1][4]<<";"<<h_1[0][5]<<";"<<h_1[1][5]);
  //***ABS_FORMATION_CONTROLLER_INFO("v_l1 " <<v_l1[0]<<";"<<v_l1[1]<<";"<<v_l1[2]<<";"<<v_l1[3]<<";"<<v_l1[4]<<";"<<v_l1[5]<<";" );
  
  
  /*应力矩阵相关计算*/
  float A[control_law_params.NumUAV][control_law_params.NumUAV]={0.0f};
  for(int i=0;i<control_law_params.NumUAV;i++){
    for(int j=0;j<control_law_params.NumUAV;j++){
      A[i][j] = -control_law_params.Omega[i][j];/*各边应力*/
      if(i==j){
        A[i][j] = 0.0;
      };
    };
  };
  
  /*中间计算输出*/
  double errorsum[2]={0.0f};
  float Kv_l1[2]={0.0f};
  for(int i=0;i<control_law_params.NumUAV;i++){
    errorsum[0]=errorsum[0]+A[planeID][i]*(fw_local[0]-everyfw_local[0][i]);
    errorsum[1]=errorsum[1]+A[planeID][i]*(fw_local[1]-everyfw_local[1][i]);
  }

  for (int i=0;i<6;i++){
    Kv_l1[0] = Kv_l1[0] + control_law_params.K[2*(planeID-control_law_params.NumLeader)][i] * v_l1[i];
    Kv_l1[1] = Kv_l1[1] + control_law_params.K[2*(planeID-control_law_params.NumLeader)+1][i] * v_l1[i];
  }

  float average_angular_rate;/*获取leader平均角速度*/
  average_angular_rate = (leader_yaw_rate[0] + leader_yaw_rate[1] + leader_yaw_rate[2]) / control_law_params.NumLeader;

    //*** 僚机控制律产生控制信号：2D线速度和航向角速度
  float vel_2D_sp{0.0};
  float angular_rate_sp{0.0};


  for(int i=0;i<2;i++){
    vel_2D_sp=vel_2D_sp+h_1[i][planeID]*(-errorsum[i]+Kv_l1[i]+average_angular_rate*h_2[i][planeID]);
    angular_rate_sp=angular_rate_sp+h_2[i][planeID]*(-errorsum[i]+Kv_l1[i]+average_angular_rate*h_2[i][planeID]);
    ABS_FORMATION_CONTROLLER_INFO("h_1 " << h_1[i][planeID] << ";" << "Kv_l1"<< Kv_l1[i]<<";"<< "h_2= "<< h_2[i][planeID] <<";");
  }
  ///**用于检查 */
  //***ABS_FORMATION_CONTROLLER_INFO("errorsum " << errorsum[0] << ";" << errorsum[1] <<";"<< "average_angular_rate= "<< average_angular_rate <<";");
  ABS_FORMATION_CONTROLLER_INFO("限制前计算线速度2D_vel= " << vel_2D_sp << ";" << "限制前计算角速度yaw_rate= "<< angular_rate_sp <<";");
  
  //**速度限制条件 */
  vel_2D_sp=sat_function(vel_2D_sp, fw_params.vmin, fw_params.vmax);
  angular_rate_sp=sat_function(angular_rate_sp, -0.5, 0.5);

  //避障控制
  obstacle_avoidance(fw_local[0],fw_local[1]);
  if (is_avoid){
    vel_2D_sp=vel_cmd;
    angular_rate_sp=angular_rate_cmd;
    ABS_FORMATION_CONTROLLER_INFO("碰撞规避僚机速度2D_vel= " << vel_2D_sp << ";" << "角速度yaw_rate= "<< angular_rate_sp <<";");
  }

  ABS_FORMATION_CONTROLLER_INFO("实际线速度2D_vel= " << vel_2D_sp << ";" << "实际角速度yaw_rate= "<< angular_rate_sp <<";");

  ABS_FORMATION_CONTROLLER_INFO("符合机体空速设定值：" << airspd_sp);    
  airspd_sp_prev = airspd_sp;  //***待定


  //#############################控制律输出量#######################################
  //ABS_FORMATION_CONTROLLER_INFO("############################# 4控制律计算后最终结果 ###############################" );
  ABS_FORMATION_CONTROLLER_INFO("僚机计算的原始线速度 = " << vel_2D_sp << ";"<<"控制律原始角速度" << angular_rate_sp);

  fw_sp.air_speed = vel_2D_sp;
  ABS_FORMATION_CONTROLLER_INFO("限幅空速设定值: " << fw_sp.air_speed);

 //***TECS得到期望俯仰角和油门(直接把2D速度作期望值了)，不行的话就添加一个高度速度补偿看看
 //***想法是，通过PID由高度误差得到期望pitch，然后期望速度由2D速度通过三角函数变换为3D速度
 //***或者由高度误差通过PID产生垂直速度分量，然后变为期望空速
 if (rest_tecs) //重置TECS
 {
  rest_tecs = false; //重置后标志位设置为不重置
  _tecs.reset_state();
 }
//***参数可能要进一步调整
//***tecs可能有点问题，减速到最低速度的时候油门降的很慢
 _tecs.set_speed_weight(tecs_params.speed_weight);
 _tecs.set_time_const_throt(tecs_params.time_const_throt); //***影响油门，越大，kp越小
 _tecs.set_time_const(tecs_params.time_const); //***影响pitch， 越大，kp越小
 _tecs.enable_airspeed(true);

  //***发现问题，fw_sp.altitude没有正确传递进来，已解决！
//  fw_sp.altitude = leader_f.altitude + formation_offset.zb;
 fw_sp.altitude = 50; //***试试直接设定期望飞行高度为100 不为主机高度(不知道为什么，设置航点高度100，QGC显示也是100，GPS高度为147左右(应该用气压计))
 ABS_FORMATION_CONTROLLER_INFO("僚机期望高度, 僚机当前高度: " << fw_sp.altitude << ";" <<  fw_states_f.altitude );
 if(fw_sp.altitude - fw_states_f.altitude >= 10)  //***判断是否需要爬升 只判断什么时候开始爬升
 {
   tecs_params.climboutdem = true;
   ABS_FORMATION_CONTROLLER_INFO("爬升");
 }
 else
 {
   tecs_params.climboutdem = false;
   ABS_FORMATION_CONTROLLER_INFO("不爬升");
 }

 //***爬升高度很大时的垂直方向的速度补偿，失速也会掉高, 试验中发现，当爬升高度较大时TECS也会自动的做一个补偿(不过很容易接近最大速度)，这里就当双保险吧，去掉也不影响
//  airspd_sp = vel_2D_sp +  height_to_speed(fw_sp.altitude, leader_f.altitude, leader_f.pitch_angle); 
//  fw_sp.air_speed = sat_function(airspd_sp, fw_params.vmin, fw_params.vmax);
//  ABS_FORMATION_CONTROLLER_INFO("限幅空速设定值: " << fw_sp.air_speed);

 _tecs.update_vehicle_state_estimates(fw_states_f.air_speed, fw_states_f.rotmat, fw_states_f.body_acc,
                                    fw_states_f.altitude_lock, fw_states_f.in_air, fw_states_f.altitude,
                                    vz_valid, fw_states_f.ned_vel_z, fw_states_f.body_acc[2]);
 
//  fw_sp.air_speed = fw_params.vmax;   //***直接给最大速度，看看pitch效果, 为什么爬升很慢呢，这里应该和之前方案没区别阿，roll的影响？已解决！
 _tecs.update_pitch_throttle(fw_states_f.rotmat, fw_states_f.pitch_angle, fw_states_f.altitude,
                                       fw_sp.altitude, fw_sp.air_speed, fw_states_f.air_speed,
                                       tecs_params.EAS2TAS, tecs_params.climboutdem, tecs_params.climbout_pitch_min_rad,
                                       fw_params.throttle_min, fw_params.throttle_max, fw_params.throttle_cruise,
                                       fw_params.pitch_min_rad,fw_params.pitch_max_rad);

 _cmd.pitch = _tecs.get_pitch_setpoint();
 _cmd.thrust = _tecs.get_throttle_setpoint();

 //***BTT产生期望滚转角
 //***是否需要补偿滚转引起的掉高呢？不加效果倒也可以(稳定后高度下降不明显)，暂时先不加了
 roll_cmd = atanf((angular_rate_sp * fw_2D_vel) / CONSTANTS_ONE_G);

 roll_cmd = constrain(roll_cmd, -fw_params.roll_max, fw_params.roll_max);

 _cmd.roll = roll_cmd;
 _cmd.planeID = planeID;
 ABS_FORMATION_CONTROLLER_INFO("最终期望俯仰角：" << _cmd.pitch);
 ABS_FORMATION_CONTROLLER_INFO("最终期望油门：" << _cmd.thrust);
 ABS_FORMATION_CONTROLLER_INFO("最终期望滚转角：" << _cmd.roll);
 ABS_FORMATION_CONTROLLER_INFO("领机uav0当前俯仰角(度)和翻滚角: "<<rad_2_deg( everyfw_states_f0.pitch_angle) << ";" <<rad_2_deg(everyfw_states_f0.roll_angle));//***看一下无人机的当前状态

 //***误差记录，只是用于画图，可通过rosbag记录
 fw_error.planeID = planeID;
 //***航向角误差
 //fw_error.led_fol_eta = tilde_psi;
 //***Frenet-serret坐标系位置误差
 //fw_error.PXb = tilde_x;
 //fw_error.PYb = tilde_y;
 //fw_error.PZb = leader_f.altitude - 50;
 //***水平线速度误差
 fw_error.Vb = fw_2D_vel - vel_2D_sp;
 //VXb = fw_gspeed_2d.x-;  //***x系速度误差
 //fw_error.VYb = fw_gspeed_2d.y-; //***y系速度误差
 //***角速度误差
 fw_error.Vk = fw_yaw_rate - angular_rate_sp;
}

