/*
 * @------------------------------------------1: 1------------------------------------------@
 *@修改自  lee-shun Email: 2015097272@qq.com
 */

//***高度的获取方式，使用气压计；将height_2_speed添加进去(昨晚试验的时候发现，如果最低速度设置的很接近stall_speed有时候会掉高严重)
//***此方案的改进点：探索带有加速度约束导航律？
//***建议添加分段控制器，当飞机起点状态差距较大时先靠近再进一步调整，若是直接采用下列方法，假如主机在转圈，从机距离主机较远，
//***则从机也会转弯，会极大的延长收敛时间
#include "abs_formation_controller.hpp"

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
 * @Input: void
 * @Output: void
 * @Description: 调用滤波器对输入的飞机原始状态进行滤波
 */
void ABS_FORMATION_CONTROLLER::filter_led_fol_states()
{
  fw_states_f = fw_states;
  leader_states_f = leader_states;

  if (use_the_filter)
  {
    leader_states_f.global_vel_x =
        led_gol_vel_x_filter.one_order_filter(leader_states.global_vel_x);

    leader_states_f.global_vel_y =
        led_gol_vel_y_filter.one_order_filter(leader_states.global_vel_y);

  }
}
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
 * 下边两个函数是测试滚转引起的掉高和高度变化的速度补偿的，后续试验后发现不加也可以，最终并未使用
*/
float ABS_FORMATION_CONTROLLER::roll_to_pitch(float aRoll)
{
  float aPitch;
        if(fabs(aRoll) < state_params.RollToPitchPara_OpenTh)
          return 0;

      aPitch = state_params.RollToPitchPara_KP * (fabs(aRoll) - state_params.RollToPitchPara_OpenTh);
      if(aPitch > state_params.RollToPitchPara_Limit)
      {
          aPitch = state_params.RollToPitchPara_Limit;
      }
      if(aPitch < -state_params.RollToPitchPara_Limit)
      {
          aPitch = -state_params.RollToPitchPara_Limit;
      }

      return aPitch;
}
float ABS_FORMATION_CONTROLLER::height_to_speed(float aHei, float cHei, float cPitch)
{
      float error=0;
      float  tempF32;

      if(state_params.HeiToSpeCtrlPara_IsOpen == false)
          return 0;
      //PitchOpenTh<0 俯冲时暂时不耦合，等到飞机平飞后再耦合
      if(cPitch < -state_params.HeiToSpeCtrlPara_PitchOpenTh)
          return 0;
      error = aHei - cHei;
      //只在爬升高度较大时才耦合到速度
      if(error < state_params.HeiToSpeCtrlPara_ErrTh)   //***也就是只在爬升的时候才耦合速度指令，好像TECS在俯冲的时候也不怎么会关注pitch角
          return 0;
      tempF32 = (error - state_params.HeiToSpeCtrlPara_ErrTh) * state_params.HeiToSpeCtrlPara_KP;
        // tempF32 = calfuzzy.getFuzzy_hei(error,cPitch);
      if(tempF32 > state_params.HeiToSpeCtrlPara_Limit)
          tempF32 = state_params.HeiToSpeCtrlPara_Limit;
      return tempF32;
}

//*********************************************************************
//***基于导航律的Unicycle模型的方法，这里期望速度直接用水平二维速度，事先飞到统一高度，再用TECS得到pitch与油门，roll根据偏航角速度，由BTT条件得到
//***10.23;问题：当从机在主机前面的时候，从机减速，等待主机追过他去(前提是航向一致)，这影响了效果？
//***QGC南向飞的时候效果不好，应该不是TECS的问题，因为起飞航线直接设置为南向直线，而不是转弯进入南向直线效果也不好(角度计算有误?)，并且这时候感觉制律产生的结果也不行
//***已解决 规整误差（-PI,PI）
//***vtol的螺旋也是可以了 注意，明天看一下pitch角的情况，为什么两架没问题，3架就有点问题了呢？可能是仿真太慢的原因，应该不是程序的问题
void ABS_FORMATION_CONTROLLER::control_law()
{
  //***input: 领机速度，x位置误差，朝向角误差 -> 线速度；领从机速度，x y位置误差，距离误差，朝向角误差，领机朝向角速度 -> 航向角速度；高度误差
  //***后续感觉也可以通过控制律得到2D速度，然后混合上一个高度所需速度，补偿为3D，然后用到TECS上
  //***control_law output: 航向角速度和线速度(2D)
  //***final output: 油门 + pitch(tecs), roll(航向角速度 BTT -> roll)
  //***步骤: 1,滤波; 2,计算所有无人机朝向；4，得到所需输入量；5，控制律；6，TECS；7，协调转弯；

  //***滤波(并未进行全部输入信息的滤波)
  filter_led_fol_states();

  if(!identify_led_fol_states())
  {
    ABS_FORMATION_CONTROLLER_INFO("警告：领机或从机未在飞行中，无法执行编队控制");
    return;
  }

  //***后续都没有考虑风速的问题, 可去掉风速
  float led_airspd_x = leader_states_f.wind_estimate_x + leader_states_f.global_vel_x;
  float led_airspd_y = leader_states_f.wind_estimate_y + leader_states_f.global_vel_y;

  //***领机地速向量
  led_gspeed_2d.set_vec_ele(leader_states_f.global_vel_x, leader_states_f.global_vel_y);
  //***领机空速向量
  led_arispd.set_vec_ele(led_airspd_x, led_airspd_y); //***2D

 //***判断读取空速与计算空速的差距
  if((led_arispd.len() - leader_states_f.air_speed) >= 3.0)
  {
    led_airspd_states_valid = false;
  }
  else
  {
    led_airspd_states_valid = true;
  }

  float leader_yaw_angle;
  float fw_yaw_angle;
  //***地速大小判别 根据地速判别当前航向角(风的影响?)
  //***地速分量有正负
  //***计算领机航向(全局坐标系下)  根据论文(-pi, pi]

  if(led_gspeed_2d.len() <= 3.0)
  {
    if(leader_states_f.yaw_valid)  
    {
      leader_yaw_angle = leader_states_f.yaw_angle;
    }
    else
    {
      leader_yaw_angle = 0;
    }
  }
  else   //***由x和y向的地速来计算当前航向角，航向角速度使用imu获得的
  {          //***角度应该也可以直接使用imu获得的
    if(led_gspeed_2d.x >0 && led_gspeed_2d.y >0)                                      
    {
      leader_yaw_angle = atanf(led_gspeed_2d.y / led_gspeed_2d.x); 
    }
    else if(led_gspeed_2d.x >0 && led_gspeed_2d.y < 0)
    {
      leader_yaw_angle = atanf(led_gspeed_2d.y / led_gspeed_2d.x);
    }
    else if (led_gspeed_2d.x <0 && led_gspeed_2d.y >0)
    {
      leader_yaw_angle = atanf(led_gspeed_2d.y / led_gspeed_2d.x) + PI;
    }
    else if (led_gspeed_2d.x <0 && led_gspeed_2d.y <0)
    {
      leader_yaw_angle = atanf(led_gspeed_2d.y / led_gspeed_2d.x) - PI;
    }
    else if (led_gspeed_2d.x == 0 && led_gspeed_2d.y <0)
    {
      leader_yaw_angle = -PI/2;
    }
    else if (led_gspeed_2d.x ==0 && led_gspeed_2d.y >0)
    {
      leader_yaw_angle = PI/2;
    }
    else if (led_gspeed_2d.x >0 && led_gspeed_2d.y == 0)
    {
      leader_yaw_angle = 0;
    }
    else if (led_gspeed_2d.x <0 && led_gspeed_2d.y ==0)  //*** -PI和PI应该一样，按照论文为PI
    {
      leader_yaw_angle = PI;
    }
    else
    {
      leader_yaw_angle = 0;
      ABS_FORMATION_CONTROLLER_INFO("领机航向角计算有误");
    }
    ABS_FORMATION_CONTROLLER_INFO("领机地速正常，选用领机地速航向角");   //***这样也对应论文里的全局系了吧
  }


  //***计算从机航向角
    //***计算从机朝向角(根据论文(-pi, pi])
  float fw_airspd_x = fw_states_f.wind_estimate_x + fw_states_f.global_vel_x;
  float fw_airspd_y = fw_states_f.wind_estimate_y + fw_states_f.global_vel_y;

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
  float x_f, y_f, theta_f;   //***根据论文的定义
  double f_pos[2],  m[2], l_pos[2];  
  f_pos[0] = fw_states_f.latitude;     
  f_pos[1] = fw_states_f.longitude;
  l_pos[0] = leader_states_f.latitude;
  l_pos[1] = leader_states_f.longitude;

  //***计算期望位置点(ned)的GPS信息, 期望位置相对于领机给出
  //***坐标系没有问题了 10.27：这时候应该不适用于这篇论文的结果
  // formation_offset.ned_n = formation_offset.xb * led_gspeed_2d.x / led_gspeed_2d.len() - formation_offset.yb * led_gspeed_2d.y / led_gspeed_2d.len();
  // formation_offset.ned_e = led_gspeed_2d.y / led_gspeed_2d.len() * formation_offset.xb + led_gspeed_2d.x / led_gspeed_2d.len() * formation_offset.yb;
  // formation_offset.ned_d = formation_offset.zb;

  formation_offset.ned_n = formation_offset.xb;      //***这样应该是嵌套圆类型的轨迹，成功，这种全局坐标应该可以
  formation_offset.ned_e = formation_offset.yb;
  formation_offset.ned_d = formation_offset.zb;
  double ref[3], result[3];
  ref[0] = leader_states_f.latitude;  //***参考点GPS信息
  ref[1] = leader_states_f.longitude;
  //***高度使用气压计的高度
  // ref[2] = leader_states_f.relative_alt;
  ref[2] = leader_states_f.altitude;
  cov_m_2_lat_long_alt(ref, formation_offset.ned_n, formation_offset.ned_e, formation_offset.ned_d, result);
  fw_sp.latitude = result[0];
  fw_sp.longitude = result[1];
  fw_sp.altitude = result[2];  //***期望高度为主机高度，后边直接改成一个定值了

 //***从机到期望点的GPS误差转化为NED误差
  cov_lat_long_2_m(result, f_pos,m); //***返回的m是距离 北东
  x_f = m[0];
  y_f = m[1];

  //***从机到期望点GPS误差转化为NED误差
  // cov_lat_long_2_m(l_pos, f_pos, m);
  // x_f = m[0] + formation_offset.xb;  //***编队期望位置按照ned给出，实际试验这里有问题，误差不是对应机体系的，未解决
  // y_f = m[1] + formation_offset.yb;   //***这样是按照全局位置来定义误差位置的吗？
  ABS_FORMATION_CONTROLLER_INFO("GPS-NED误差：" <<m[0] << ";" << m[1]);
  ABS_FORMATION_CONTROLLER_INFO("编队偏移: x, y: " << formation_offset.xb << ";" << formation_offset.yb);
  ABS_FORMATION_CONTROLLER_INFO("编队期望位置误差：x, y:  " << x_f << ";" << y_f);  //***这里x y 对应也反了 应该按照NE的方式去理解
  
  //***看一下垂直速度的变化，对期望速度有多大的影响
  ABS_FORMATION_CONTROLLER_INFO("从机z方向的速度: " << fw_states_f.global_vel_z);  //***gps的速度消息得不到z轴的速度信息


  theta_f = fw_yaw_angle - leader_yaw_angle;
  //***规整角度误差到(-pi,pi)上  试一下这种方式在南向的情况，可行
   //***这步是很有必要的：从机的控制有时候会有一定的滞后，所以当主机出现正负PI/2时，从机若没有及时跟上则会影响精度，这是因为导航律设计的原因(优先逆时针调整), 这样可以一定程度上减缓这种现象
 //***导航律的设计会优先向逆时针方向转
  if(abs(theta_f) > PI)
  {
    if(theta_f < 0)
    {
      theta_f = 2*PI + theta_f;
    }
    else
    {
      theta_f = -2*PI + theta_f;
    }
  }
  else
  {
    theta_f = theta_f;
  }

  //***剩余所需输入量
 float  leader_yaw_rate, fw_yaw_rate, leader_2D_vel, fw_2D_vel;

 leader_yaw_rate = leader_states_f.yaw_rate;
 fw_yaw_rate = fw_states_f.yaw_rate;
 leader_2D_vel = led_gspeed_2d.len();  //***使用地速来表示当前速度 都要为正值
 fw_2D_vel = fw_gspeed_2d.len();

 //***发现问题：yaw_rate为0 已解决；
  ABS_FORMATION_CONTROLLER_INFO("主从角度：led_yaw, fol_yaw: " << leader_yaw_angle << ";" << fw_yaw_angle);
  ABS_FORMATION_CONTROLLER_INFO("主机速度：2D_vel, yaw_rate: " << led_gspeed_2d.len() << ";" << leader_states_f.yaw_rate);
  ABS_FORMATION_CONTROLLER_INFO("从机速度：2D_vel, yaw_rate: " << fw_gspeed_2d.len() << ";" << fw_states_f.yaw_rate);
  ABS_FORMATION_CONTROLLER_INFO("全局误差：x, y, theta: " << x_f << ";" << y_f << ";" << theta_f);

 //***控制律模块
 //***1 坐标转换，转到D系
 float x_e, y_e, theat_e, r_e;

 x_e = x_f * cosf(leader_yaw_angle) + y_f * sinf(leader_yaw_angle);
 y_e = -x_f * sinf(leader_yaw_angle) + y_f * cosf(leader_yaw_angle);
 theat_e = theta_f;
 r_e = sqrt(x_e * x_e + y_e * y_e);
 ABS_FORMATION_CONTROLLER_INFO("D系误差: x, y, theta: " << x_e << ";" << y_e << ";" << theat_e);

 //***2 产生滑膜面
 float s;
 s = theat_e + control_law_params.k1 * y_e / (control_law_params.k2 + r_e);
 ABS_FORMATION_CONTROLLER_INFO("滑膜面s：" << s);

 //***3 产生控制信号：2D线速度和航向角速度
 float vel_2D_sp, angular_rate_sp, temp;

 //***线速度
 //***注意 速度正负 在论文里边速度始终是正值
 if(abs(theat_e) == PI/2 || abs(theat_e) == 3* PI / 2)
 {
   vel_2D_sp = fw_params.max_arispd_sp;
 }
 else
 {
   temp = (leader_2D_vel - control_law_params.k3 * x_e) / cosf(theat_e);
   vel_2D_sp = sat_function(temp,fw_params.min_arispd_sp, fw_params.max_arispd_sp);
 }


 //***角速度
 //***会在s=0附近震荡，属于正常现象，若想改进可以选择其他的函数形式代替sign(s)函数
//  angular_rate_sp = leader_yaw_rate - control_law_params.k4 * sign_function(s) - control_law_params.k1 * (fw_2D_vel * sinf(theat_e) - leader_yaw_rate * x_e) / (control_law_params.k2 + r_e)
//                                     - control_law_params.k1 * (leader_2D_vel * x_e - fw_2D_vel * cosf(theat_e) * x_e - fw_2D_vel * sinf(theat_e) * y_e) * y_e / (r_e * (control_law_params.k2 + r_e) * (control_law_params.k2 + r_e));

//***使用s/|s| + eplision函数代替 效果好点了 可以多试验几个不同的eplision值
 angular_rate_sp = leader_yaw_rate - control_law_params.k4 * sign_replace_function(s) - control_law_params.k1 * (fw_2D_vel * sinf(theat_e) - leader_yaw_rate * x_e) / (control_law_params.k2 + r_e)
                                    - control_law_params.k1 * (leader_2D_vel * x_e - fw_2D_vel * cosf(theat_e) * x_e - fw_2D_vel * sinf(theat_e) * y_e) * y_e / (r_e * (control_law_params.k2 + r_e) * (control_law_params.k2 + r_e));
 ABS_FORMATION_CONTROLLER_INFO("控制律产生的原始线速度与角速度：" << vel_2D_sp << ";" << angular_rate_sp);

 //***分段一下，这个方法速度会变化较快，飞机跟不上速度的指令 此论文可以在加速度约束上加以改进
//  if(sqrt(x_f * x_f + y_f * y_f) <= 70)
//  {
//    airspd_sp = 18;
//  }

 //***限幅  暂不考虑风速
 //***符合飞机加减速特性 慎用
//  if((airspd_sp - airspd_sp_prev) > fw_params.maxinc_acc * _dt)
//  {
//    airspd_sp = airspd_sp_prev + fw_params.maxinc_acc * _dt;
//  }
//  else if ((airspd_sp - airspd_sp_prev) < -fw_params.maxdec_acc * _dt)
//  {
//    airspd_sp = airspd_sp_prev - fw_params.maxdec_acc * _dt;
//  }
 
 ABS_FORMATION_CONTROLLER_INFO("符合机体空速设定值：" << airspd_sp);       
 airspd_sp_prev = airspd_sp;  //***待定

 fw_sp.air_speed = constrain(vel_2D_sp, fw_params.min_arispd_sp, fw_params.max_arispd_sp);
 ABS_FORMATION_CONTROLLER_INFO("限幅空速设定值: " << fw_sp.air_speed);

 angular_rate_sp = constrain(angular_rate_sp, -1.2, 1.2);  //最大航向角速度为1.2rad/s, 原来限幅出错了，角速度是有正负的，不能为0
 ABS_FORMATION_CONTROLLER_INFO("限幅角速度设定值: " << angular_rate_sp);

 //***TECS得到期望俯仰角和油门(直接把2D速度作期望值了)，不行的话就添加一个高度速度补偿看看
 //***想法是，通过PID由高度误差得到期望pitch，然后期望速度由2D速度通过三角函数变换为3D速度
 //***或者由高度误差通过PID产生垂直速度分量，然后变为期望空速
 if (rest_tecs)
 {
  rest_tecs = false;
  _tecs.reset_state();
 }
//***参数可能要进一步调整
//***tecs可能有点问题，减速到最低速度的时候油门降的很慢
 _tecs.set_speed_weight(tecs_params.speed_weight);
 _tecs.set_time_const_throt(tecs_params.time_const_throt); //***影响油门，越大，kp越小
 _tecs.set_time_const(tecs_params.time_const); //***影响pitch， 越大，kp越小
 _tecs.enable_airspeed(true);

  //***发现问题，fw_sp.altitude没有正确传递进来，已解决！
//  fw_sp.altitude = leader_states_f.altitude + formation_offset.zb;
 fw_sp.altitude = 50; //***试试直接设定期望飞行高度为100 不为主机高度(不知道为什么，设置航点高度100，QGC显示也是100，GPS高度为147左右(应该用气压计))
 ABS_FORMATION_CONTROLLER_INFO("从机期望高度, 当前高度，主机当前高度: " << fw_sp.altitude << ";" <<  fw_states_f.altitude << ";" << leader_states_f.altitude);
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
//  fw_sp.air_speed = constrain(airspd_sp, fw_params.min_arispd_sp, fw_params.max_arispd_sp);
//  ABS_FORMATION_CONTROLLER_INFO("限幅空速设定值: " << fw_sp.air_speed);

 _tecs.update_vehicle_state_estimates(fw_states_f.air_speed, fw_states_f.rotmat, fw_states_f.body_acc,
                                    fw_states_f.altitude_lock, fw_states_f.in_air, fw_states_f.altitude,
                                    vz_valid, fw_states_f.ned_vel_z, fw_states_f.body_acc[2]);
 
//  fw_sp.air_speed = fw_params.max_arispd_sp;   //***直接给最大速度，看看pitch效果, 为什么爬升很慢呢，这里应该和之前方案没区别阿，roll的影响？已解决！
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

 ABS_FORMATION_CONTROLLER_INFO("最终期望滚转角：" << _cmd.roll);
 ABS_FORMATION_CONTROLLER_INFO("从机当前俯仰角(度)和翻滚角: "<< rad_2_deg(fw_states_f.pitch_angle) << ";" << rad_2_deg(fw_states_f.roll_angle)); //***看一下无人机的当前状态
 ABS_FORMATION_CONTROLLER_INFO("领机当前俯仰角(度)和翻滚角: "<<rad_2_deg( leader_states_f.pitch_angle) << ";" <<rad_2_deg(leader_states_f.roll_angle));

 //***误差记录，只是用于画图，可通过rosbag记录
 //***角度误差
 fw_error.led_fol_eta = theta_f;
 //***D系位置误差
 fw_error.PXb = x_e;
 fw_error.PYb = y_e;
 fw_error.PZb = fw_states_f.altitude - 50;
 //***水平线速度误差
 fw_error.Vb = fw_2D_vel - leader_2D_vel;
 fw_error.VXb = fw_gspeed_2d.x - led_gspeed_2d.x;  //***x系速度误差
 fw_error.VYb = fw_gspeed_2d.y - led_gspeed_2d.y; //***y系速度误差
 //***角速度误差
 fw_error.Vk = fw_yaw_rate - leader_yaw_rate;
}
