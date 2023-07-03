/*
 * @Description:  输入输出提示
 */

#ifndef _SYSLIB_HPP_
#define _SYSLIB_HPP_
#include <iostream>
#include <fstream>
#include <string.h>
#include <sys/time.h>

using namespace std;

long get_sys_time() //返回的时间为毫秒，使用需要除以1000，为秒
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

long get_time_from_begin(long begin_time) //进入、返回的时间为毫秒，使用需要除以1000，为秒
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000 + tv.tv_usec / 1000 - begin_time);
}

void write_to_files(string file_path_name, string flight_mode, float data[5]) //打开一个文件，将它的值以及时间戳写进去，文件命名为值的名字
{
    long time_stamp = get_sys_time(); //时间戳
    fstream oufile;                   //创建文件对象

    oufile.open(file_path_name.c_str(), ios::app | ios::out);
    oufile << fixed << time_stamp << "\t"
           << "\t" << flight_mode;
    for (int i = 0; i <= 4; i++)
    {
        oufile << "\t" << data[i];
    }
    oufile << endl;

    if (!oufile)
        cout << file_path_name << "-->"
             << "something wrong to open or write" << endl;
    oufile.close();
}
/**
 * @Input: string
 * @Output:string
 * @Description:将两个string合并
 */
 
string add2str(string a, string b)
{
    return a+b;
}

#endif
