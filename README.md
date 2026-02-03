# Decode - FTC Team Code Repository

This repository contains the code for the Lumens Cubed (30393) FIRST Tech Challenge (FTC) team for the 2025-2026 season.

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
                            ├── MecanumLimelightRelative # TeleOp (driver-controlled) code
                            ├── Red 12 # Autonomous code
                            ├── Red 9 # Autonomous code
                            ├── Blue 12 # Autonomous code
                            └── Blue 9 # Autonomous code etc...
```

## OpModes

### TeleOp
- **DecodeTeleOp**: Main teleoperation program for driver-controlled period
  - Mecanum drive control using gamepad joysticks
  - Real-time telemetry display
  - Autoaim and tracking of goal
  - Auto distance sensing to flywheel power

### Autonomous
- **DecodeAutonomous**: Main autonomous program for the autonomous period
  - Pedro Pathing used
  - 12 ball, 9 ball, and gate autos


## Season

2025-2026 Season - Lumens Cubed (30393)
