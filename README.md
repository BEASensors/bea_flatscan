# Quick Start Instruction
1. `mkdir -p ~/bea_ws/src/ && cd ~/bea_ws/src/`
2. `git clone git@github.com:huamingduo/bea_sensors.git` or `git clone https://github.com/huamingduo/bea_sensors.git`
3. `cd ~/bea_ws/ && catkin build`
4. `source devel/setup.bash`
5. `roslaunch bea_sensors flatscan.launch`

# Functionality Description
## Parameters
All the parameters are listed in the launch file
- **port**: the port connecting to the FlatScan sensor, e.g. "/dev/ttyUSB0"
- **baudrate**: the baudrate used for communication
- **scan_frame_id**: the frame_id of the published LaserScan message
- **scan_topic**: the topic of the LaserScan message published to
- **heartbeat_topic**: the topic of the heartbeat message published to
- **emergency_topic**: the topic of the emergency message published to
- **min_range**: the minimal valid range of the LaserScan, in meters
- **max_range**: the maximal valid range of the LaserScan, in meters
- **enable_temperature**: CTN field in MDI frames
- **information_in_mdi**: the required data in MDI frames, e.g. "2" indicates that both distances and intensities are requried
- **detection_field_mode**: the required detection mode, e.g. "HD" means "High Density" mode
- **optimization**: the sensitivity and immunity optimization setting, e.g. "0" means no optimization
- **angle_first**: the starting angle, in degrees
- **angle_last**: the ending angle, in degrees
- **enable_counter**: the MDI, heartbeat, emergency counters settings
- **heartbeat_period**: the heartbeat message publishing rate, setting to "0" disables the heartbeat publishing
- **enable_facet**: facet settings
- **averaging_setting**: averaging settings, "0" means no averaging

## Published Messages
1. sensor_msgs/LaserScan  
   Published when the MDI messages are received, the data included in the frame is set by the parameter *information_in_mdi*
2. bea_sensors/Emergency  
   Published only when error occurs or the host requests.
3. bea_sensors/Heartbeat  
   Published periodically according to the parameter heartbeat_period
   
## Service
Used for configuring the flat scan sensor  
General form: `rosservice call /flatscan/configure "command: <command> subcommand: <subcommand> value: <value>"`

<table width="1372" border="0" cellpadding="0" cellspacing="0" style='width:1029.00pt;border-collapse:collapse;table-layout:fixed;'>
   <col width="216" style='mso-width-source:userset;mso-width-alt:5529;'/>
   <col width="129" style='mso-width-source:userset;mso-width-alt:3302;'/>
   <col width="430" style='mso-width-source:userset;mso-width-alt:11008;'/>
   <col width="597" style='mso-width-source:userset;mso-width-alt:15283;'/>
   <tr height="21" style='height:15.75pt;'>
    <td height="21" width="216" style='height:15.75pt;width:162.00pt;' x:str>Command</td>
    <td width="129" style='width:96.75pt;' x:str>SubCommand</td>
    <td width="430" style='width:322.50pt;' x:str>Value</td>
    <td width="597" style='width:447.75pt;' x:str>Example</td>
   </tr>
   <tr height="105" style='height:78.75pt;'>
    <td height="105" style='height:78.75pt;' x:str>set_baudrate</td>
    <td x:str>-</td>
    <td class="xl65" x:str>57600<br/>115200<br/>230400<br/>460800<br/>921600</td>
    <td class="xl65" x:str>rosservice call /flatscan/configure &quot;command: 'set_baudrate'<br/>subcommand: ''<br/>value: '921600'&quot;</td>
   </tr>
   <tr height="63" style='height:47.25pt;'>
    <td height="63" style='height:47.25pt;' x:str>get_measurements</td>
    <td x:str>-</td>
    <td class="xl65" x:str>single shot<br/>continuous</td>
    <td class="xl65" x:str>rosservice call /flatscan/configure &quot;command: 'get_measurements'<br/>subcommand: ''<br/>value: 'continuous'&quot;</td>
   </tr>
   <tr height="63" style='height:47.25pt;'>
    <td height="63" style='height:47.25pt;' x:str>get_identity</td>
    <td x:str>-</td>
    <td x:str>-</td>
    <td class="xl65" x:str>rosservice call /flatscan/configure &quot;command: 'get_identity'<br/>subcommand: ''<br/>value: ''&quot;</td>
   </tr>
   <tr height="63" style='height:47.25pt;'>
    <td height="63" style='height:47.25pt;' x:str>get_emergency</td>
    <td x:str>-</td>
    <td x:str>-</td>
    <td class="xl65" x:str>rosservice call /flatscan/configure &quot;command: 'get_emergency'<br/>subcommand: ''<br/>value: ''&quot;</td>
   </tr>
   <tr height="63" style='height:47.25pt;'>
    <td height="63" style='height:47.25pt;' x:str>get_parameters</td>
    <td x:str>-</td>
    <td x:str>-</td>
    <td class="xl65" x:str>rosservice call /flatscan/configure &quot;command: 'get_parameters'<br/>subcommand: ''<br/>value: ''&quot;</td>
   </tr>
   <tr height="63" style='height:47.25pt;'>
    <td class="xl66" height="744" rowspan="10" style='height:558.00pt;border-right:none;border-bottom:none;' x:str>set_parameters</td>
    <td x:str>temperature</td>
    <td class="xl65" x:str>0: disable<br/>1: enable</td>
    <td class="xl65" x:str>rosservice call /flatscan/configure &quot;command: 'set_parameters'<br/>subcommand: 'temperature'<br/>value: '1'&quot;</td>
   </tr>
   <tr height="63" style='height:47.25pt;'>
    <td x:str>information</td>
    <td class="xl65" x:str>0: distances<br/>1: remissions<br/>2: distances and remissions</td>
    <td class="xl65" x:str>rosservice call /flatscan/configure &quot;command: 'set_parameters'<br/>subcommand: 'information'<br/>value: '0'&quot;</td>
   </tr>
   <tr height="63" style='height:47.25pt;'>
    <td x:str>mode</td>
    <td class="xl65" x:str>0: HS<br/>1: HD</td>
    <td class="xl65" x:str>rosservice call /flatscan/configure &quot;command: 'set_parameters'<br/>subcommand: 'mode'<br/>value: '1'&quot;</td>
   </tr>
   <tr height="112" style='height:84.00pt;mso-height-source:userset;mso-height-alt:1680;'>
    <td x:str>optimization</td>
    <td class="xl65" x:str>0: no optimization (maximum sensitivity)<br/>1: range = 0 to 2.5m (minimum sensitivity)<br/>2 : range = 0 to 3.0m<br/>3 : range = 0 to 3.5m<br/>4 : range longer than 3.5m (maximum sensitivity)</td>
    <td class="xl65" x:str>rosservice call /flatscan/configure &quot;command: 'set_parameters'<br/>subcommand: 'optimization'<br/>value: '0'&quot;</td>
   </tr>
   <tr height="63" style='height:47.25pt;'>
    <td x:str>angle_first</td>
    <td class="xl65" x:str>0 to 10800 (unit: 0.01 degrees)</td>
    <td class="xl65" x:str>rosservice call /flatscan/configure &quot;command: 'set_parameters'<br/>subcommand: 'angle_first'<br/>value: '0'&quot;</td>
   </tr>
   <tr height="63" style='height:47.25pt;'>
    <td x:str>angle_last</td>
    <td class="xl65" x:str>0 to 10800 (unit: 0.01 degrees)</td>
    <td class="xl65" x:str>rosservice call /flatscan/configure &quot;command: 'set_parameters'<br/>subcommand: 'angle_last'<br/>value: '10800'&quot;</td>
   </tr>
   <tr height="63" style='height:47.25pt;'>
    <td x:str>counter</td>
    <td class="xl65" x:str>0: disable<br/>1: enable</td>
    <td class="xl65" x:str>rosservice call /flatscan/configure &quot;command: 'set_parameters'<br/>subcommand: 'temperature'<br/>value: '1'&quot;</td>
   </tr>
   <tr height="63" style='height:47.25pt;'>
    <td x:str>heartbeat</td>
    <td class="xl65" x:str>0 to 255 (unit: seconds, 0 means disable)</td>
    <td class="xl65" x:str>rosservice call /flatscan/configure &quot;command: 'set_parameters'<br/>subcommand: 'heartbeat'<br/>value: '5'&quot;</td>
   </tr>
   <tr height="63" style='height:47.25pt;'>
    <td x:str>facet</td>
    <td class="xl65" x:str>0: disable<br/>1: enable</td>
    <td class="xl65" x:str>rosservice call /flatscan/configure &quot;command: 'set_parameters'<br/>subcommand: 'facet'<br/>value: '1'&quot;</td>
   </tr>
   <tr height="128" style='height:96.00pt;mso-height-source:userset;mso-height-alt:1920;'>
    <td x:str>averaging</td>
    <td class="xl65" x:str>0 : No averaging<br/>1 : averaging 3 points in time<br/>2 : averaging 3 points in time + 2 neighbours<br/>3 : averaging 5 points in time<br/>4 : averaging 5 points in time + 2 neighbours</td>
    <td class="xl65" x:str>rosservice call /flatscan/configure &quot;command: 'set_parameters'<br/>subcommand: 'averaging'<br/>value: '0'&quot;</td>
   </tr>
   <tr height="63" style='height:47.25pt;'>
    <td height="63" style='height:47.25pt;' x:str>store_parameters</td>
    <td x:str>-</td>
    <td x:str>-</td>
    <td class="xl65" x:str>rosservice call /flatscan/configure &quot;command: 'store_parameters'<br/>subcommand: ''<br/>value: ''&quot;</td>
   </tr>
   <tr height="63" style='height:47.25pt;'>
    <td height="63" style='height:47.25pt;' x:str>reset_mdi_counter</td>
    <td x:str>-</td>
    <td x:str>-</td>
    <td class="xl65" x:str>rosservice call /flatscan/configure &quot;command: 'reset_mdi_counter'<br/>subcommand: ''<br/>value: ''&quot;</td>
   </tr>
   <tr height="63" style='height:47.25pt;'>
    <td height="63" style='height:47.25pt;' x:str>reset_heartbeat_counter</td>
    <td x:str>-</td>
    <td x:str>-</td>
    <td class="xl65" x:str>rosservice call /flatscan/configure &quot;command: 'reset_heartbeat_counter'<br/>subcommand: ''<br/>value: ''&quot;</td>
   </tr>
   <tr height="63" style='height:47.25pt;'>
    <td height="63" style='height:47.25pt;' x:str>reset_emergency_counter</td>
    <td x:str>-</td>
    <td x:str>-</td>
    <td class="xl65" x:str>rosservice call /flatscan/configure &quot;command: 'reset_emergency_counter'<br/>subcommand: ''<br/>value: ''&quot;</td>
   </tr>
   <tr height="63" style='height:47.25pt;'>
    <td class="xl67" height="131" rowspan="2" style='height:98.25pt;border-right:none;border-bottom:none;' x:str>set_led</td>
    <td x:str>set</td>
    <td class="xl65" x:str>color<br/>(color could be off, red, green, or orange)</td>
    <td class="xl65" x:str>rosservice call /flatscan/configure &quot;command: 'set_led'<br/>subcommand: 'set'<br/>value: 'red'&quot;</td>
   </tr>
   <tr height="68" style='height:51.00pt;mso-height-source:userset;mso-height-alt:1020;'>
    <td x:str>blink</td>
    <td class="xl65" x:str>color1 color2 frequency<br/>(color1 and color2 could be off, red, green, or orange)<br/>(frequency could be 1 to 10)</td>
    <td class="xl65" x:str>rosservice call /flatscan/configure &quot;command: 'set_led'<br/>subcommand: 'blink'<br/>value: 'green off 4'&quot;</td>
   </tr>
   <![if supportMisalignedColumns]>
    <tr width="0" style='display:none;'>
     <td width="216" style='width:162;'></td>
     <td width="129" style='width:97;'></td>
     <td width="430" style='width:323;'></td>
     <td width="597" style='width:448;'></td>
    </tr>
   <![endif]>
  </table>

  
