# Added by Ronja Gueldenring
# For spawning agents dynamically in pedsim and forwarding them to flatland
int16 id
geometry_msgs/Point pos
int16 type
int16 number_of_peds

# regarding waypoints:
# x, y: position
# z: radius of waypoint (determines how close a ped needs to be to a waypoint in order to reach it)
geometry_msgs/Point[] waypoints

string yaml_file

