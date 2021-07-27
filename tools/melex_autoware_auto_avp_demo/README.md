# MELEX_AUTOWARE_AUTO_AVP_DEMO

## Description
### Messages design for simulator
LGSVL interface Pubs (simulator -> interface -> autoware stack). We need same output from our melex driver. Msgs marked as <font color="green">green</font> are crucial (behaviour planner input).

Publishers:
*  /lgsvl/vehicle_control_cmd: lgsvl_msgs/msg/VehicleControlData
    <details>
    <summary>Interface</summary>
    
    ```
    std_msgs/Header header
    
    float32 acceleration_pct  # 0 to 1
    float32 braking_pct  # 0 to 1
    float32 target_wheel_angle  # radians
    float32 target_wheel_angular_rate  # radians / second
    uint8 target_gear
    
    uint8 GEAR_NEUTRAL = 0
    uint8 GEAR_DRIVE = 1
    uint8 GEAR_REVERSE = 2
    uint8 GEAR_PARKING = 3
    uint8 GEAR_LOW = 4
    ```
    </details>

*  /lgsvl/vehicle_state_cmd: lgsvl_msgs/msg/VehicleStateData
    <details>
    <summary>Interface</summary>
    
    ```
    std_msgs/Header header

    uint8 blinker_state
    uint8 headlight_state
    uint8 wiper_state
    uint8 current_gear
    uint8 vehicle_mode
    bool hand_brake_active
    bool horn_active
    bool autonomous_mode_active
    
    uint8 BLINKERS_OFF = 0
    uint8 BLINKERS_LEFT = 1
    uint8 BLINKERS_RIGHT = 2
    uint8 BLINKERS_HAZARD = 3
    
    uint8 HEADLIGHTS_OFF = 0
    uint8 HEADLIGHTS_LOW = 1
    uint8 HEADLIGHTS_HIGH = 2
    
    uint8 WIPERS_OFF = 0
    uint8 WIPERS_LOW = 1
    uint8 WIPERS_MED = 2
    uint8 WIPERS_HIGH = 3
    
    uint8 GEAR_NEUTRAL = 0
    uint8 GEAR_DRIVE = 1
    uint8 GEAR_REVERSE = 2
    uint8 GEAR_PARKING = 3
    uint8 GEAR_LOW = 4
    
    uint8 VEHICLE_MODE_COMPLETE_MANUAL = 0
    uint8 VEHICLE_MODE_COMPLETE_AUTO_DRIVE = 1
    uint8 VEHICLE_MODE_AUTO_STEER_ONLY = 2
    uint8 VEHICLE_MODE_AUTO_SPEED_ONLY = 3
    uint8 VEHICLE_MODE_EMERGENCY_MODE = 4
    ```
    </details>

*  /parameter_events: rcl_interfaces/msg/ParameterEvent
*  /rosout: rcl_interfaces/msg/Log
*  /tf: tf2_msgs/msg/TFMessage
*  /vehicle/odom_pose: geometry_msgs/msg/PoseWithCovarianceStamped
    <details>
    <summary>Interface</summary>
    
    ```
    std_msgs/Header header
    PoseWithCovariance pose
    ```
    </details>   

*   /vehicle/odometry: autoware_auto_msgs/msg/VehicleOdometry
    <details>
    <summary>Interface</summary>
    
    ```
    #include "builtin_interfaces/msg/Time.idl"
    
    module autoware_auto_msgs {
      module msg {
        @verbatim (language="comment", text=
          " VehicleOdometry.msg")
        struct VehicleOdometry {
          builtin_interfaces::msg::Time stamp;
    
          @default (value=0.0)
          float velocity_mps;
    
          @default (value=0.0)
          float front_wheel_angle_rad;
    
          @default (value=0.0)
          float rear_wheel_angle_rad;
        };
      };
    };
    ```
    </details>   

*  <font color="green">/vehicle/state_report: autoware_auto_msgs/msg/VehicleStateReport</font>
    <details>
    <summary>Interface</summary>
    
    ```
    #include "builtin_interfaces/msg/Time.idl"

    module autoware_auto_msgs {
      module msg {
        module VehicleStateReport_Constants {
          const uint8 BLINKER_OFF = 1;
          const uint8 BLINKER_LEFT = 2;
          const uint8 BLINKER_RIGHT = 3;
          const uint8 BLINKER_HAZARD = 4;
          const uint8 HEADLIGHT_OFF = 1;
          const uint8 HEADLIGHT_ON = 2;
          const uint8 HEADLIGHT_HIGH = 3;
          const uint8 WIPER_OFF = 1;
          const uint8 WIPER_LOW = 2;
          const uint8 WIPER_HIGH = 3;
          const uint8 WIPER_CLEAN = 14; // Match WipersCommand::ENABLE_CLEAN
          const uint8 GEAR_DRIVE = 1;
          const uint8 GEAR_REVERSE = 2;
          const uint8 GEAR_PARK = 3;
          const uint8 GEAR_LOW = 4;
          const uint8 GEAR_NEUTRAL = 5;
          const uint8 MODE_AUTONOMOUS = 1;
          const uint8 MODE_MANUAL = 2;
          const uint8 MODE_DISENGAGED = 3;
          const uint8 MODE_NOT_READY = 4;
        };
    
        struct VehicleStateReport {
          builtin_interfaces::msg::Time stamp;
    
          @verbatim (language="comment", text=
            " 0 to 100")
          uint8 fuel;
    
          uint8 blinker;
    
          uint8 headlight;
    
          uint8 wiper;
    
          uint8 gear;
    
          uint8 mode;
    
          boolean hand_brake;
    
          boolean horn;
        };
      };
    };
    ```
    </details>
    <details>
    <summary>Example output</summary>
   
    ```
    ---
    stamp:
      sec: 0
      nanosec: 0
    fuel: 0
    blinker: 2
    headlight: 1
    wiper: 1
    gear: 1
    mode: 0
    hand_brake: false
    horn: false
    ---
    ```
    </details>  

*  <font color="green">/vehicle/vehicle_kinematic_state: autoware_auto_msgs/msg/VehicleKinematicState</font>
    <details>
    <summary>Interface</summary>
    
    ```
    #include "autoware_auto_msgs/msg/TrajectoryPoint.idl"
    #include "geometry_msgs/msg/Transform.idl"
    #include "std_msgs/msg/Header.idl"
    
    module autoware_auto_msgs {
      module msg {
        @verbatim (language="comment", text=
          " VehicleKinematicState.msg" "\n"
          " Representation of a trajectory point with timestamp for the controller")
        struct VehicleKinematicState {
          std_msgs::msg::Header header;
    
          autoware_auto_msgs::msg::TrajectoryPoint state;
    
          geometry_msgs::msg::Transform delta;
        };
      };
    };
    ```
    </details>
    <details>
    <summary>Example output</summary>
   Example acquired during driving.
   
   ```
   ---
   header:
     stamp:
       sec: 1627304034
       nanosec: 547711744
     frame_id: odom
   state:
     time_from_start:
       sec: 0
       nanosec: 0
     x: 46.602638244628906
     y: 44.43940734863281
     heading:
       real: 0.9631139039993286
       imag: 0.2690940499305725
     longitudinal_velocity_mps: 7.514681339263916
     lateral_velocity_mps: 0.0
     acceleration_mps2: 0.0
     heading_rate_rps: -0.004081131890416145
     front_wheel_angle_rad: 0.0
     rear_wheel_angle_rad: 0.0
   delta:
     translation:
       x: 0.0
       y: 0.0
       z: 0.0
     rotation:
       x: 0.0
       y: 0.0
       z: 0.0
       w: 1.0
   ---
   ```
    </details>  

### Signal flow 

![AVP_Architecture](images/AVP_Architecture.png)

We need to follow output signal from behavior planner to low level steering commands.

**Behavior planner** subs: 
* `/vehicle/state_report` (logical states including gear, brake etc.)
* semantic map
* `/vehicle/vehicle_kinematic_state` (velocity, steering angle, heading of car)
* pose request (we can use command input or rviz gui)

   As we can see, acceleration is not used within those topics. It appears in `/vehicle/vehicle_kinematic_state` but it 
   equals 0 all the time (see example mentioned above marked as green). 

**Behavior planner** pubs: 
* trajectory

**MPC** subs:
* trajectory
* `/vehicle/vehicle_kinematic_state` (velocity, steering angle, heading of car)

**MPC** pubs:
* `/vehicle/vehicle_command` <font color="orange">(velocity, acceleration, steering angle)</font>

   Unlike the acceleration within `/vehicle/vehicle_kinematic_state`, values changes depending on current trajectory

**Vehicle interface** subs:
* `/vehicle/vehicle_command` (velocity, acceleration, steering angle) given by MPC
* current steering angle and velocity

<font color="red">**Summarize:**</font>
MPC Controller node returns **acceleration**, velocity and wheel angle which is necessary to control vehicle in simulator.
In our case we can decide how to control melex.

Example MPC Controller output (vel & acc):
![AVP_Architecture](images/mpc_vel_acc_output.png)
Those values are converted to brake & throttle pedals signals (0-1). My idea: Let skip acceleration on the beginning 
and use velocity to control vehicle with PID controller. Focus on green marked msgs (state_report & vehicle_kinematic_state).


## Info
Check ssc (speed and steering control)
https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/src/tools/autoware_auto_avp_demo/launch/ms3_vehicle.launch.py/#L175-L193

https://github.com/Autoware-AI/autoware.ai/issues/1944

https://autonomoustuff.com/products/astuff-speed-steering-control-software



## ToDo
*

## Known issues
*