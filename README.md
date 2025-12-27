# Decode - FTC Team Code Repository

This repository contains the code for the Decode FIRST Tech Challenge (FTC) team for the 2025 season.

## Contents

This repository includes:
- **TeleOp Code**: Driver-controlled operation programs
- **Autonomous Code**: Autonomous period programs

## Directory Structure

```
TeamCode/
└── src/
    └── main/
        └── java/
            └── org/
                └── firstinspires/
                    └── ftc/
                        └── teamcode/
                            ├── DecodeTeleOp.java      # TeleOp (driver-controlled) code
                            └── DecodeAutonomous.java  # Autonomous code
```

## OpModes

### TeleOp
- **DecodeTeleOp**: Main teleoperation program for driver-controlled period
  - Tank drive control using gamepad joysticks
  - Real-time telemetry display

### Autonomous
- **DecodeAutonomous**: Main autonomous program for the autonomous period
  - Encoder-based movement
  - Example autonomous routine included

## Getting Started

1. Clone this repository
2. Open the project in Android Studio
3. Build the project
4. Deploy to your robot's Control Hub or Robot Controller phone
5. Select and run the desired OpMode from the Driver Station

## Hardware Configuration

The code assumes the following hardware configuration:
- `left_drive`: Left drive motor
- `right_drive`: Right drive motor

Make sure to configure your robot's hardware to match these names in the Robot Controller app.

## Season

2025 Season - Decode Team
