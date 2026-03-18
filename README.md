# eYRC_MazeBot_FPGA-Based Autonomous Maze Solver


## Project Overview

This project is part of the **e-Yantra Robotics Competition (EYRC)**, a unique robotics competition funded by the Ministry of Education and hosted at **IIT Bombay**.

This is a complete FPGA implementation of an autonomous maze-solving robot that uses the **DFS1 (Depth-First Search) algorithm** to navigate a 9×9 grid maze. The system combines wall-detection sensors, movement control, soil moisture sensing, and real-time UART communication.

## Key Features

- **Autonomous Navigation**: DFS1 algorithm for optimal maze solving
- **Multi-Sensor System**: IR sensors, ultrasonic distance sensors, DHT temperature/humidity sensors, soil moisture sensor
- **Wall Following**: Proportional control for smooth alignment during movement
- **Precise Turn Control**: 90° and 180° turns using rotary encoder feedback
- **Soil Sensing**: Automatic soil moisture measurement at designated locations (MPI points)
- **Real-Time UART Logging**: Transmits position, temperature, humidity, and soil moisture data over Bluetooth
- **Safety Features**: Ultrasonic-based collision avoidance during critical maneuvers
- **Clock Domain Crossing**: Proper CDC synchronization between 50MHz and 3.125MHz domains

## Architecture

### Core Modules

#### 1. **movement_logic.v** (976 lines)
Main FSM controlling robot movement and decision-making
- States: STOP_STATE, MOVE_STATE, LEFT_TURN, RIGHT_TURN, UTURN
- Sub-states for turns: FORWARD_BEFORE_TURN → TURN → FORWARD_AFTER_TURN
- Integrates DFS1 module for pathfinding
- Outputs motor commands and MPI trigger signals

#### 2. **dfs1.v** (DFS Algorithm Module)
Implements the Depth-First Search algorithm
- 4-pulse clock gating mechanism
- Position tracking (x, y coordinates on 9×9 grid)
- Movement decisions: forward, left turn, right turn, U-turn
- Outputs: `dfs_move`, `dfs_decision_valid`

#### 3. **uart_process.v** (470 lines)
Handles all UART communication and data transmission
- Real-time coordinate logging via UART
- **S_IDLE**: Main idle state with coordinate change detection
- **S_COORD_LOG**: Transmits position packets (POS-X-Y-#)
- **S_HOLD_SERVO**: 5-second sensor hold for readings
- **S_MPI/S_MOISTURE/S_TEMP_HUMID/S_COUNTER**: Multi-line MPI packet transmission
- CDC synchronization for MPI trigger signals
- Packet formats:
  - `MPIM-<id>-<x>-<y>-#`: MPI identification + position
  - `MM-<id>-M/D-#`: Soil moisture (Moist/Dry)
  - `TH-<id>-XX-XX-#`: Temperature and Humidity
  - `END-#`: Maze completion
  - `POS-X-Y-#`: Coordinate updates

#### 4. **top_module_merged.v** (217 lines)
Top-level integration of all subsystems
- Instantiates movement_logic, uart_process, motor drivers
- Frequency scaling (50MHz → 3.125KHz)
- Signal routing and clock distribution

#### 5. **ultrasonic_refined.v** + **median_filter.v**
Noise-filtered ultrasonic sensor output
- 5-sample shift register
- Combinational sorting network for fast median calculation
- Timeout detection (~2 seconds)
- Output registration for timing closure

### Sensor Modules

- **ultrasonic.v**: HC-SR04 ultrasonic sensor driver with echo synchronizer
- **dht.v**: DHT22 temperature/humidity sensor
- **moisture_sensor.v**: Capacitive soil moisture sensor with ADC interface
- **servo_control.v**: PWM servo control (up/down for sensor access)
- **pwm_generator.v**: PWM signal generation for motor speed control
- **rotory_encoder.v**: Quadrature encoder counting for distance/angle feedback

### Communication Modules

- **uart_rx.v**: Serial receiver (RX)
- **uart_tx.v**: Serial transmitter (TX)
- **ascii_decimal.v**: Binary to ASCII decimal conversion

## Clock Domains

- **clk_50M**: Main system clock (50 MHz)
  - Movement logic, encoder counting, motor control
  
- **clk_3125KHz**: UART and slow timing (3.125 MHz = 50MHz/16)
  - UART communication, servo control, sensor polling

**CDC (Clock Domain Crossing):**
- MPI trigger: 50MHz → 3.125MHz with 3-stage synchronizer
- Exit signal: 50MHz → 3.125MHz with 3-stage synchronizer
- Coordinate monitoring: Direct polling (ultrasonic domain independent)

## Movement States & Transitions

```
STOP_STATE (Initial/Exit)
    ↓
MOVE_STATE (DFS Decision)
    ├─ Forward → Continue
    ├─ Left Turn → LEFT_TURN
    ├─ Right Turn → RIGHT_TURN
    ├─ U-Turn → UTURN
    └─ Exit → STOP_STATE

LEFT_TURN / RIGHT_TURN (with sub-states)
    → FORWARD_BEFORE_TURN (move to center)
    → TURN (rotate 90°)
    → FORWARD_AFTER_TURN (move post-turn)
    → back to MOVE_STATE

UTURN (with sub-states)
    → FORWARD_BEFORE_TURN (move close to wall)
    → TURN (rotate 180°)
    → [Optional] MPI (soil sensing)
    → FORWARD_AFTER_TURN (move back to center)
    → back to MOVE_STATE
```

## Data Transmission Protocol

### Coordinate Update Packet
```
POS-X-Y-#\r\n
Example: POS-3-7-#\r\n  (Position at grid 3,7)
Triggered: On coordinate change detection (automatic)
```

### MPI Sensor Packet (Multi-line)
```
Line 0: MPIM-<id>-<x>-<y>-#\r\n         (MPI ID + position)
Line 1: MM-<id>-M/D-#\r\n                (Soil moisture status)
Line 2: TH-<id>-XX-XX-#\r\n              (Temperature-Humidity)
Line 3: \r\n\r\n                         (Spacing)

Example:
MPIM-1-2-3-#
MM-1-M-#
TH-1-25-65-#
\r\n
```

### Maze End Packet
```
END-#\r\n
Transmitted when exit detected after DFS completes
```

## Pin Assignments (Typical)

### Sensors
- IR sensors: ir_left, ir_right, ir_front
- Ultrasonic: distance_left, distance_right, distance_front
- Encoders: left_encoder_count, right_encoder_count
- DHT: dht_sensor (inout)
- Moisture: dout, adc_cs_n, din, adc_sck

### Motor Control
- Motor 1: in1, in2, ena
- Motor 2: in3, in4, enb

### Communication
- UART: uart_rx_in, uart_tx_out
- Servo: servo1_pwm, servo2_pwm

## Key Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| GRID_SIZE | 9×9 | Maze dimensions |
| SAFETY_THRESHOLD | 60mm | Collision avoidance distance |
| BEFORE_TURN_COUNT | 3150 | Encoder count before turning |
| TURN_90_COUNT | 1850 | Encoder count for 90° turn |
| TURN_180_COUNT | 2525 | Encoder count for 180° turn |
| TURN_SPEED | 13 | Duty cycle during turns |
| HOLD_SERVO_TIME | 250M cycles | 5-second sensor hold duration |

## Simple Usage

1. **Initialization**: Robot starts in STOP_STATE
2. **Start**: Trigger via UART command 'A' or debug_rx_start_cmd signal
3. **Navigation**: DFS1 algorithm automatically navigates maze
4. **Monitoring**: UART outputs position, temperature, humidity, soil moisture
5. **Completion**: Robot stops when maze exit is reached, transmits "END-#"

## File Structure

```
code/
├── top_module_merged.v          # Top-level module
├── movement_logic.v             # Main movement FSM
├── dfs1.v                        # DFS algorithm
├── uart_process.v               # UART communication
│
├── Motor & Control
├── motor_driver_l298n.v          # Motor driver logic
├── pwm_generator.v              # PWM signal generation
├── servo_control.v              # Servo PWM control
├── rotory_encoder.v             # Encoder counter
│
├── Sensors
├── ultrasonic.v                 # Ultrasonic sensor driver
├── ultrasonic_refined.v         # Noise-filtered version
├── median_filter.v              # Sorting network filter
├── dht.v                        # Temperature/humidity sensor
├── moisture_sensor.v            # Soil moisture sensor
├── adc_controller.v             # ADC interface
│
├── Communication
├── uart_rx.v                    # Serial receiver
├── uart_tx.v                    # Serial transmitter
├── ascii_decimal.v              # Binary to ASCII converter
│
├── Frequency Control
├── frequency_scaling.v          # Clock divider (50MHz → 3.125KHz)
│
└── README.md                    # This file
```

## Testing & Debugging

### Debug Outputs
- `state_debug`: Current movement state
- `turn_debug`: Current turn sub-state
- `op_debug`: Front wall detection status
- `debug_dfs_pos_x`, `debug_dfs_pos_y`: Current grid position

### Monitor Signals
- `mpi_trigger`: Active during soil sensing
- `mpi_complete`: Signals end of sensor reading
- `exit_maze`: Indicates maze completion

## Known Limitations & Notes

1. **Median Filtering**: Ultrasonic sensor uses 5-point median for noise reduction (adds ~5 sample latency)
2. **Timeout Handling**: MPI waits 300M cycles (~6 seconds) after trigger - intentional for stable sensor readings
3. **Coordinate Polling**: Detects changes every cycle in UART IDLE state (efficient for coordinate logging)
4. **Clock Skew**: CDC synchronizers ensure safe cross-domain signal handling

## Future Enhancements

- [ ] Adaptive turn speeds based on ultrasonic feedback
- [ ] Position history logging for backtracking
- [ ] Multi-point MPI measurement at each grid
- [ ] Wireless telemetry (beyond UART)
- [ ] Machine learning for wall-following optimization

## References

- HC-SR04 Ultrasonic Sensor Datasheet
- DHT22 Temperature/Humidity Sensor
- L298N Motor Driver Logic
- DFS Algorithm Reference: Iterative Deepening Depth-First Search (IDDFS)

---

**Author**: Maze Bot Development Team  
**Last Updated**: March 18, 2026  
**Version**: 2.0 - DFS1 Integration Complete
