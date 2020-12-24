#include<iostream>
#include<string>
#include <fstream>
 #include <math.h>
using namespace std;
int main()
{
    cout <<"\n---------------------------------------------------------------------\n";
    cout<<"\n A program to generate world file containing asphalt plane.\nThis program takes input dimension of a planar block unit in meter.\nAssumption is that each planar block is a square\nIt also takes number of coloumns and rows to generate a grid of planar block\nto create a large plane";
    
    
    cout <<"\n---------------------------------------------------------------------\n";
    
    cout<< "\nEnter the number of coloumn:";
    
    //ofstream fout1;
    ofstream fout2, fout3;
    //fout1.open("model1.xml");
   // fout2.open("model2.xml");
    
    string worldfilename;
    int p,q, m=400, n=400;
    cin>>p;
    cout<< "\nEnter the number of rows:";
    cin>>q;
    cout<<"\nEnter the dimension of each planar unit in meters:";
    cin >>m;
    
    cout <<"What name would you give to newly generated world file(filename should end .world):";
    cin >> worldfilename;
    fout3.open(worldfilename);
    
    cout <<"\n---------------------------------------------------------------------\n";
    n = m;
   // cout<<"Enter length of each plane block:";
   // cin>>m;
  // cout<<"Enter width of each plane block:";
  //  cin>>n;

    string PreAmble = "";
    PreAmble = PreAmble+"<sdf version='1.4'>\n"+
"<world name='default'>\n"+
"   <light name='sun' type='directional'>\n"+
"       <cast_shadows>1</cast_shadows>\n"+
"       <pose>0 0 10 0 -0 0</pose>\n"+
"       <diffuse>0.8 0.8 0.8 1</diffuse>\n"+
"       <specular>0.2 0.2 0.2 1</specular>\n"+
"       <attenuation>\n"+
"           <range>1000</range>\n"+
"           <constant>0.9</constant>\n"+
"           <linear>0.01</linear>\n"+
"           <quadratic>0.001</quadratic>\n"+
"       </attenuation>\n"+
"       <direction>-0.5 0.1 -0.9</direction>\n"+
"   </light>\n"+
"   <model name='ground_plane'>\n"+
"    <static>1</static>\n"+
"    <link name='link'>\n"+
"       <collision name='collision'>\n"+
"           <geometry>\n"+
"               <plane>\n"+
"                   <normal>0 0 1</normal>\n"+
"                   <size>100 100</size>\n"+
"               </plane>\n"+
"           </geometry>\n"+
"           <surface>\n"+
"            <friction>\n"+
"                <ode>\n"+
"                    <mu>100</mu>\n"+
"                    <mu2>50</mu2>\n"+
"                    </ode>\n"+
"            </friction>\n"+
"            <bounce/>\n"+
"            <contact>\n"+
"              <ode/>\n"+
"            </contact>\n"+
"          </surface>\n"+
"         <max_contacts>10</max_contacts>\n"+
"       </collision>\n"+
"       <visual name='visual'>\n"+
"           <cast_shadows>0</cast_shadows>\n"+
"            <geometry>\n"+
"               <plane>\n"+
"                   <normal>0 0 1</normal>\n"+
"                       <size>100 100</size>\n"+
"              </plane>\n"+
"           </geometry>\n"+
"           <material>\n"+
"           <script>\n"+
"               <uri>file://media/materials/scripts/gazebo.material</uri>\n"+
"               <name>Gazebo/Grey</name>\n"+
"           </script>\n"+
"           </material>\n"+
"        </visual>\n"+
"    <velocity_decay>\n"+
"       <linear>0</linear>\n"+
"       <angular>0</angular>\n"+
"   </velocity_decay>\n"+
"   <self_collide>0</self_collide>\n"+
"   <kinematic>0</kinematic>\n"+
"   <gravity>1</gravity>\n"+
" </link>\n"+
"</model>\n"+
"<physics type='ode'>\n"+
"   <max_step_size>0.01</max_step_size>\n"+
"   <real_time_factor>1</real_time_factor>\n"+
"   <real_time_update_rate>100</real_time_update_rate>\n"+
"   <gravity>0 0 -9.8</gravity>\n"+
"</physics>\n"+
"<scene>\n"+
"   <ambient>0.4 0.4 0.4 1</ambient>\n"+
"   <background>0.7 0.7 0.7 1</background>\n"+
"   <shadows>1</shadows>\n"+
"</scene>\n"+
"<spherical_coordinates>\n"+
"   <surface_model>EARTH_WGS84</surface_model>\n"+
"   <latitude_deg>0</latitude_deg>\n"+
"   <longitude_deg>0</longitude_deg>\n"+
"   <elevation>0</elevation>\n"+
"   <heading_deg>0</heading_deg>\n"+
"</spherical_coordinates>";



    string text1= "";
    
    string pretemplate = "<static>1</static>\n\t<link name='link'>\n\t\t<collision name='collision'>\n\t\t\t<geometry>\n\t\t\t\t<box><size>"
    +std::to_string(m)+" "+std::to_string(n)+" 0.6</size></box>"+"\n\t\t\t</geometry>\n\t\t\t"+
    "  <surface>\n"+
"            <friction>\n"+
"                <ode>\n"+
"                    <mu>10000</mu>\n"+
"                    <mu2>5000</mu2>\n"+
"                    </ode>\n"+
"            </friction>\n"+
"            <bounce/>\n"+
"            <contact>\n"+
"              <ode/>\n"+
"            </contact>\n"+
"          </surface>\n"+
    "<max_contacts>10</max_contacts>\n\t\t\t"+
    "<surface>\n\t\t\t\t<bounce/>\n\t\t\t\t<friction>\n\t\t\t\t\t<ode/>\n\t\t\t\t</friction> \n\t\t\t\t<contact> \n\t\t\t\t\t<ode/>\n\t\t\t\t</contact>\n\t\t\t</surface>\n\t\t</collision> \n\t\t<visual name='visual'>\n\t\t\t<cast_shadows>0</cast_shadows>"+
    "\n\t\t\t<geometry> \n\t\t\t\t<box><size>"+std::to_string(m)+" "+std::to_string(n)+" 0.6</size></box>\n\t\t\t</geometry>\n\t\t\t<material>\n\t\t\t\t<script>\n\t\t\t\t\t<uri>model://asphalt_plane/materials/scripts</uri>\n\t\t\t\t\t<uri>model://asphalt_plane/materials/textures</uri>\n\t\t\t\t\t<name>vrc/asphalt</name>\n\t\t\t\t</script>\n\t\t\t</material>\n\t\t</visual>"+
    "\n\t\t<velocity_decay><linear>0</linear><angular>0</angular></velocity_decay>"+
    "\n\t\t<self_collide>0</self_collide>"+
    "\n\t\t<kinematic>0</kinematic>"+
    "\n\t\t<gravity>1</gravity>"+        
    "\n\t</link>\n\t<pose>";
    string posttemplate = "0 0 -0 0</pose>";
          
    int k = 1;
    for(int i=-p/2; i <ceil(p/2.0) ;++i)
    {
        for(int j =-q/2; j<ceil(q/2.0);++j)
        {
          text1 =text1+"<model name='asphalt_plane_"+std::to_string(k) + "'>\n\t";
          text1 = text1 + pretemplate  + std::to_string(m*i) + " "+std::to_string(n*j) + " "+posttemplate  + "\n</model>\n"; 
          k++;
        }
    }
    
    
  string text2="";
   pretemplate = "\n\t<pose>";
   string posttemplate1 = "0 0 -0 0</pose>\n\t\t<link name='link'>\n\t\t\t<pose>";
   string posttemplate2 =  "0 0 -0 0</pose>\n\t\t\t<velocity>0 0 0 0 -0 0</velocity>\n\t\t\t<acceleration>0 0 0 0 -0 0</acceleration>\n\t\t\t<wrench>0 0 0 0 -0 0</wrench>\n\t\t</link>\n</model>\n";

    k = 1;
    for(int i=-p/2; i <ceil(p/2.0);++i)
    {
        for(int j = -q/2; j<ceil(q/2.0);++j)
        {
          text2=text2+"<model name='asphalt_plane_"+std::to_string(k) + "'>\n\t";
          text2 =text2 + pretemplate  + std::to_string(m*i) + " "+std::to_string(n*j) + " "+posttemplate1 + std::to_string(m*i) + " "+std::to_string(n*j) + " " +posttemplate2; 
          k++;
        }
    }
    
    
    
    string midAmble = "\n<state world_name='default'>\n";
    string postAmble = "";
    postAmble = postAmble+" </state>\n"+
"<gui fullscreen='0'>\n"+
"<camera name='user_camera'>\n"+
"<pose>23.0739 -33.1427 15.8356 0 0.3343 2.05704</pose>\n"+
"<view_controller>orbit</view_controller>\n"+
"</camera>\n"+
"</gui>\n"+
"</world>\n"+
"</sdf>";

    string Main  = PreAmble+text1+midAmble+text2+postAmble;
    
    fout3<< Main;
    
    cout <<"\nWorld file written to  \n";
    return 0;
}

    
