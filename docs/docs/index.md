# Dynamixel Hardware Interface

ROS2 hardware interface for Dynamixel Motors.
You can control Dynamixel motors like below.

<blockquote class="twitter-tweet"><p lang="ja" dir="ltr">いえーい <a href="https://t.co/PPQaBwk8uZ">pic.twitter.com/PPQaBwk8uZ</a></p>&mdash; 片岡大哉 (@hakuturu583) <a href="https://twitter.com/hakuturu583/status/1389880980769366021?ref_src=twsrc%5Etfw">May 5, 2021</a></blockquote> <script async src="https://platform.twitter.com/widgets.js" charset="utf-8"></script>

## How to use
### Write URDF/XACRO file for your robot
Example is [here](https://github.com/OUXT-Polaris/miniv_description/blob/4403e9e9af678ff150802422a89e8ee213684d8d/urdf/miniv.ros2_control.xacro#L5).
You can use dynamixel hardware interface by writing URDF like below.

```xml
<xacro:macro name="azimuth_thruster_control" params="left_joint right_joint enable_dummy">
    <ros2_control name="azimuth_thruster_control" type="system">
        <hardware>
            <plugin>dynamixel_hardware_interface/DynamixelHardwareInterface</plugin>
            <param name="port_name">/dev/ttyUSB0</param>
            <param name="baudrate">9600</param>
            <param name="enable_dummy">${enable_dummy}</param>
        </hardware>
        <joint name="${left_joint}">
            <param name="id">1</param>
            <param name="motor_type">XW540-T260</param>
            <command_interface name="position"/>
            <state_interface name="position"/>
        </joint>
        <joint name="${right_joint}">
            <param name="id">2</param>
            <param name="motor_type">XW540-T260</param>
            <command_interface name="position"/>
            <state_interface name="position"/>
        </joint>
    </ros2_control>
</xacro:macro>
```

#### Hardware parameters in URDF/XACRO

|     Name     |  Type  |                              Description                              |
| ------------ | ------ | --------------------------------------------------------------------- |
| port_name    | string | USB port name of the U2D2                                             |
| baudrate     | int    | baudrate of the RS485 communication                                   |
| enable_dummy | bool   | If true, this hardware interface runs without real dynamixel hardwre. |

#### Joint parameters in URDF/XACRO
|    Name    |  Type  |                   Description                   |
| ---------- | ------ | ----------------------------------------------- |
| id         | int    | id of the dynamixel motor attached to the joint |
| motor_type | string | type of the dynamixel motor                     |

### Connect motors to the U2D2
Connect motors to the [U2D2](https://e-shop.robotis.co.jp/product.php?id=190).
Power cable is also required.

### Setup motors
Use [dynamixel wizard](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/) and configure motor ID and baudrate

## Support Status

### State Interface
|                                Motor                                | Position Interface | Velocity Interface | Effort Interface | Tempelature Interface |
| ------------------------------------------------------------------- | ------------------ | ------------------ | ---------------- | --------------------- |
| [XW540-T260](https://emanual.robotis.com/docs/en/dxl/x/xw540-t260/) | ✔                  | ✔                  |                  | ✔                     |

### Command Interface
|                                Motor                                | Position Interface | Velocity Interface | Effort Interface |
| ------------------------------------------------------------------- | ------------------ | ------------------ | ---------------- |
| [XW540-T260](https://emanual.robotis.com/docs/en/dxl/x/xw540-t260/) | ✔                  |                    |                  |
