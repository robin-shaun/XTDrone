string info                                   # text info on this contact
string collision1_name                        # name of contact collision1
string collision2_name                        # name of contact collision2
geometry_msgs/Wrench[] wrenches               # list of forces/torques
geometry_msgs/Wrench total_wrench             # sum of forces/torques in every DOF
geometry_msgs/Vector3[] contact_positions     # list of contact position
geometry_msgs/Vector3[] contact_normals       # list of contact normals
float64[] depths                              # list of penetration depths
