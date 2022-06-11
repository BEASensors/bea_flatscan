# Compilation
1. `mkdir -p ~/bea_ws/src/ && cd ~/bea_ws/src/`
2. `git clone git@github.com:huamingduo/bea_sensors.git` or `git clone https://github.com/huamingduo/bea_sensors.git`
3. `catkin build`
4. `source devel/setup.bash`
5. `roslaunch bea_sensors flat_scan.launch`

# Usage
## Parameters

## Published Messages
1. sensor_msgs/LaserScan

2. bea_sensors/Emergency  
   Published only when error occurs or the host requests.
3. bea_sensors/Heartbeat  
   Published periodically according to the parameter heartbeat_period
   
## Service
Used for configuring the flat scan sensor  
General form: `rosservice call /flat_scan/configure "command: <command> subcommand: <subcommand> value: <value>"`
