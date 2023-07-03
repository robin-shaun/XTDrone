/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-02-14 18:07:32
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  滤波函数类
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors  : lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime : 2020-02-14 18:50:55
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */

#ifndef _FILTER_HPP_
#define _FILTER_HPP_
/**
 * 滤波函数类
*/
class FILTER
{
public:
    /**
    * 一阶低通滤波器
    */
    float one_order_filter(const float input);
    void set_one_order_filter_param(float input_param) { filter_param = input_param; }

private:
    /**
     * for one_order_filter
    */

  /* 直接输入前的系数,默认0.05 */
  float filter_param{0.1};
  /* 一阶低通滤波器上次滤波后值 */
  float last_output{0};
};

#endif
float FILTER::one_order_filter(const float input)
{
    float output = 0;

    output = filter_param * input + (1 - filter_param) * last_output; //

    last_output = output;

    return output;
}
