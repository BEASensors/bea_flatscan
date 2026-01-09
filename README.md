# Introduction
This is a ROS2 Driver for the BEA LZR® Raw-F (Flatscan) laser scanner.

# Quick Start Instruction
1. `mkdir -p ~/bea_ws/src/ && cd ~/bea_ws/src/`
2. `git clone https://github.com/BEASensors/bea_flatscan2.git`
3. `cd ~/bea_ws/ && colcon build`
4. `source install/setup.bash`
5. (Serial) `ros2 launch bea_sensors flatscan_422.launch.py`
6. (Ethernet) `ros2 launch bea_sensors flatscan_tcp.launch.py`

# Functionality Description
## Parameters
All these parameters are listed in the launch file:
- `communication` connection type, `serial` or `ethernet`
- `port` serial device (when `communication=serial`), e.g. "/dev/ttyUSB0"
- `baudrate` serial baudrate (when `communication=serial`)
- `ip` device IP (when `communication=ethernet`)
- `port` device TCP port (when `communication=ethernet`)
- `scan_frame_id` the frame_id of the published LaserScan message
- `scan_topic` the topic of the LaserScan message published to
- `heartbeat_topic` the topic of the heartbeat message published to
- `emergency_topic` the topic of the emergency message published to
- `min_range` the minimal valid range of the LaserScan, in meters
- `max_range` the maximal valid range of the LaserScan, in meters
- `enable_temperature` CTN field in MDI frames
- `information_in_mdi` the required data in MDI frames, e.g. "2" indicates that both distances and intensities are requried
- `detection_field_mode` the required detection mode, e.g. "HD" means "High Density" mode
- `optimization` the sensitivity and immunity optimization setting, e.g. "0" means no optimization
- `angle_first` the starting angle, in degrees
- `angle_last` the ending angle, in degrees
- `enable_counter` the MDI, heartbeat, emergency counters settings
- `heartbeat_period` the heartbeat message publishing rate, setting to "0" disables the heartbeat publishing
- `enable_facet` facet settings
- `averaging_setting`: averaging settings, "0" means no averaging

## Published Messages
1. sensor_msgs/LaserScan  
   Published when the MDI messages are received, the data included in the frame is set by the parameter *information_in_mdi*
2. bea_sensors/Emergency  
   Published only when error occurs or the host requests.
3. bea_sensors/Heartbeat  
   Published periodically according to the parameter heartbeat_period
   
## Service
This package defines a `bea_sensors/srv/Configure` service type, but the current driver node does not provide a service server (no `create_service` in code). Therefore, `ros2 service call` to configure the device is not supported in this version.

Note: the examples in the following table are legacy ROS1 `rosservice` commands kept only as a reference of supported command names. They are not applicable to this ROS2 driver version.

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

## 串口与以太网(TCP)连接方式切换（中文说明 / English）
本驱动同时支持串口与以太网（TCP）两种连接方式，通过私有参数 `communication` 进行选择，并结合对应参数完成配置。

This driver supports both serial and Ethernet (TCP) connections. Select the connection type via the `communication` parameter and configure the corresponding parameters.

- `communication`：可取值 `serial`（串口）或 `ethernet`（以太网）
- 当为 `serial` 时：使用参数 `port`（如 `/dev/ttyUSB0`）与 `baudrate`（如 `921600`）
- 当为 `ethernet` 时：使用参数 `ip`（如 `192.168.1.254`）与 `port`（如 `30003`）

- `communication`: `serial` or `ethernet`
- When `serial`: use `port` (e.g. `/dev/ttyUSB0`) and `baudrate` (e.g. `921600`)
- When `ethernet`: use `ip` (e.g. `192.168.1.254`) and `port` (e.g. `30003`)

注意：上述参数为 ROS2 节点参数，参数作用域为节点实例。

Note: these are ROS2 node parameters and take effect within the node instance scope.

### 方式一：修改 launch 文件（推荐 / Recommended）
以 `bea_sensors/launch/flatscan_422.launch.py` 为例：

串口连接示例：修改 `flatscan_422.launch.py` 中 `parameters` 字典。

Example (serial): edit the `parameters` dictionary in `flatscan_422.launch.py`.

以太网（TCP）连接示例：使用 `flatscan_tcp.launch.py`，或修改 launch 中 `communication/ip/port`。

Example (Ethernet/TCP): use `flatscan_tcp.launch.py`, or edit `communication/ip/port` in the launch file.

提示：`launch` 中的其他功能参数（如 `scan_topic`、`heartbeat_period` 等）可按需调整。若需要显示 RViz，请在 WSL2 环境配置好 X 服务器，否则可暂时移除/注释 RViz 节点以避免 GUI 报错。

Tip: other parameters in the launch file (e.g. `scan_topic`, `heartbeat_period`) can be adjusted as needed. If you want to display RViz in WSL2, configure an X server first; otherwise remove/comment out the RViz node to avoid GUI errors.

### 方式二：使用 ros2 param 动态设置（不改文件 / Without editing files）
在启动节点前或之后（若已启动需重启以生效），通过 `ros2 param` 设置节点参数：

You can also set parameters via `ros2 param`. If the node is already running, restart it for the changes to take effect.

- 切换为串口：
```bash
ros2 param set /flatscan communication serial
ros2 param set /flatscan port /dev/ttyUSB0
ros2 param set /flatscan baudrate 921600
```

- 切换为以太网（TCP）：
```bash
ros2 param set /flatscan communication ethernet
ros2 param set /flatscan ip 192.168.1.254
ros2 param set /flatscan port 30003
```

设置完成后，重新启动对应节点或重新 `ros2 launch` 使参数生效。

After setting the parameters, restart the node (or re-run `ros2 launch`) for the changes to take effect.

### 运行与验证（Run & Verify）
1. 启动：
```bash
source ~/bea_ws/install/setup.bash
ros2 launch bea_sensors flatscan_422.launch.py
```

2. 验证话题与频率：
```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic echo -n 1 /heartbeat
```

3. 网络连通性（以太网模式）：
```bash
ping -c 3 192.168.1.254
# 如需端口探测：
nc -vz 192.168.1.254 30003
```

若网络可达但仍无数据，请检查设备输出配置、网段与防火墙设置。

If the network is reachable but there is still no data, check the device output configuration, subnet settings, and firewall rules.

## 快速上手（从零开始，适用于 ROS 新手 / Beginner Quick Start）
以下以 Ubuntu + ROS2 为例，WSL2 用户请在 Windows 侧准备 X 服务器用于 GUI（可选）。

The following steps assume Ubuntu + ROS2. If you are using WSL2 and need GUI tools (optional), prepare an X server on Windows.

### 1. 安装 ROS 与依赖（Install ROS2 & Dependencies）
```bash
# 略：参见 ROS2 官方安装指南
sudo apt-get update
sudo apt-get install -y \
  python3-colcon-common-extensions \
  netcat
```

### 2. 创建工作区并获取源码（Create workspace & get source）
```bash
mkdir -p ~/bea_ws/src
cd ~/bea_ws/src
# 若已有源码可忽略克隆步骤
# git clone https://github.com/BEASensors/bea_flatscan.git bea_sensors
```

将本仓库放在 `~/bea_ws/src/bea_sensors` 目录下。

Place this repository under `~/bea_ws/src/bea_sensors`.

### 3. 编译（Build）
```bash
cd ~/bea_ws
colcon build
source install/setup.bash
```

### 4. 选择连接方式并配置参数（Select connection & configure parameters）
驱动支持两种连接方式，请二选一：
- 串口（默认）：`communication=serial`，使用 `port` 与 `baudrate`
- 以太网（TCP）：`communication=ethernet`，使用 `ip` 与 `port`

This driver supports two connection methods:
- Serial (default): `communication=serial`, using `port` and `baudrate`
- Ethernet (TCP): `communication=ethernet`, using `ip` and `port`

方式一（推荐）：使用/编辑 launch 文件
- 串口：`ros2 launch bea_sensors flatscan_422.launch.py`
- 以太网：`ros2 launch bea_sensors flatscan_tcp.launch.py`

Option 1 (recommended): use/edit the launch files above.

方式二：用 ros2 param 设置（需重启节点生效）
```bash
# 串口
ros2 param set /flatscan communication serial
ros2 param set /flatscan port /dev/ttyUSB0
ros2 param set /flatscan baudrate 921600
# 以太网
ros2 param set /flatscan communication ethernet
ros2 param set /flatscan ip 192.168.1.254
ros2 param set /flatscan port 30003
```

Option 2: use `ros2 param` (restart the node for changes to take effect).

### 5. 启动与验证（Launch & verify）
```bash
source ~/bea_ws/install/setup.bash
ros2 launch bea_sensors flatscan_422.launch.py
```
另开终端验证：

Verify in another terminal:
```bash
source ~/bea_ws/install/setup.bash
ros2 topic list
ros2 topic hz /scan
ros2 topic echo -n 1 /heartbeat
```

以太网模式下连通性自检：
```bash
ping -c 3 192.168.1.254
nc -vz 192.168.1.254 30003
```

Connectivity check for Ethernet mode:

### 6. 在 WSL2 显示 RViz（可选 / RViz on WSL2 (Optional)）
- Windows 端安装并启动 X 服务器（VcXsrv/GWSL/X410）。
- 在 WSL2 设置环境后启动 RViz：

- Install and start an X server on Windows (VcXsrv/GWSL/X410).
- Then configure the environment in WSL2 and start RViz:
```bash
export DISPLAY=$(grep -m1 nameserver /etc/resolv.conf | awk '{print $2}'):0
export LIBGL_ALWAYS_INDIRECT=1
export QT_X11_NO_MITSHM=1
source ~/bea_ws/install/setup.bash
rviz2 -d ~/bea_ws/src/bea_sensors/launch/flatscan.rviz
```
若未配置 X 服务器，建议临时从 `flatscan_422.launch.py` 中移除/注释 RViz 节点。

If you don't have an X server configured, temporarily remove/comment out the RViz node in `flatscan_422.launch.py`.

### 7. 常见问题排查（Troubleshooting）
- 串口打不开：检查权限（可添加当前用户到 `dialout` 组），设备路径是否为 `/dev/ttyUSB*`。

- Serial port cannot be opened: check permissions (add user to `dialout` group) and confirm the device path is `/dev/ttyUSB*`.
```bash
sudo usermod -a -G dialout $USER
newgrp dialout
```
- 以太网无数据：检查 IP/端口、网络同网段、防火墙；设备侧是否启用了数据输出。
- 无法显示 RViz：确认已运行 X 服务器并正确设置 `DISPLAY`；或暂时不启动 RViz。

- No Ethernet data: check IP/port, subnet settings, firewall, and whether the device has data output enabled.
- RViz cannot be displayed: ensure the X server is running and `DISPLAY` is set correctly, or run without RViz.

  
