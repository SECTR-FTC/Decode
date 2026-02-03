package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
//@Disabled
@TeleOp(name = "Mecanum With Limelight (Brian Test)", group = "Test")

public class MecanumLimelightRelative extends OpMode {
    private Timer pathTimer, actionTimer, opmodeTimer, flywheelTimer, shootTimeTimer;
    private double GATE_OPEN_POS = 0.25;
    private double GATE_CLOSED_POS = 0.55;

    // This declares the motors/servos needed (pedro constants handles drive train)
    private DcMotorEx elevator = null;
    private DcMotor intake = null;
    private Servo IndexerR = null;
    private DcMotorEx flywheel1 = null;
    private DcMotorEx flywheel2 = null;

    private Limelight3A limelight = null;

    private boolean flywheelOn = false;
    private boolean slowDrive = false;


    // This declares the IMU needed to get the current direction the robot is facing
//    IMU imu;

    // FLYWHEEL COEFFICIENTS AND VARIABLES

    double Flywheel_Power = 0.48;
    double currentRPM = 0;
    double targetRPM = 1200;
    static double maxRPM = 5800;
    double kP = 0.0135;
    double kI = 0.0001;
    double kD = 0.00007;
    double kF = 0.001;
    double kFmultiplier = 2.49;
    static double ilimit = 500;
    double errorSum = 0;
    double lastError = 0;
    boolean launching = false;

    // AUTOMATED DRIVE COEFFICIENTS AND VARIABLES
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.3;
    // ---------------- GOAL ORIENTED DRIVE ----------------
    private boolean goalOriented = false;     // toggle: lock heading to goal
    private boolean redAlliance = true;       // toggle: choose goal
    // ---- MT2 yaw seeding ----
    private boolean mt2YawSeeded = false;
    private double yawOffsetDeg = 0.0; // fieldYawDeg = pedroYawDeg + yawOffsetDeg

    // Goal centers (pedro coordinates, origin at blue corner).
    private static final Pose RED_GOAL = new Pose(133, 135);
    private static final Pose BLUE_GOAL = new Pose(12, 135);

    // Heading PID (tune on-field)
    private double aim_kP = 0.35;
    private double aim_kI = 0.0;
    private double aim_kD = 0.00;
    private double aim_kF = 0.000;
    
    private double aimErrSum = 0.0;
    private double aimLastErr = 0.0;
    private double lastWrappedAngle = 0.0;
    private Timer aimTimer;


    @Override()
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        shootTimeTimer = new Timer();
        opmodeTimer.resetTimer();
        flywheelTimer = new Timer();
        aimTimer = new Timer();

        //Hardware Map: (pedro Constants handles Drivetrain)
        elevator = hardwareMap.get(DcMotorEx.class, "elevator");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "FlywheelLeft");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "FlywheelRight");
        IndexerR = hardwareMap.get(Servo.class, "IndexerR");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(45); // This sets how often we ask Limelight for data ]
        limelight.start(); // This tells Limelight to start looking!

        limelight.pipelineSwitch(8); // Switch to pipeline number 8

        // Set motor Direction - Currently set for Main chassis (pedro handles drivetrain)

        intake.setDirection(DcMotor.Direction.FORWARD);
        flywheel1.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);
        elevator.setDirection(DcMotorEx.Direction.FORWARD);

        IndexerR.setDirection(Servo.Direction.REVERSE);

        flywheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set motor encoder states (pedro handles drivetrain)

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        //Set Zero Power Behavior (pedro handles drivetrain)

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        elevator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(39, 33))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
        telemetry.addLine("Ready to start Tele");
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
        IndexerR.setPosition(GATE_CLOSED_POS);
        aimErrSum = 0.0;
        aimLastErr = 0.0;
        lastWrappedAngle = 0.0;
        aimTimer.resetTimer();
        // MT2 yaw seed reset
        mt2YawSeeded = false;
        yawOffsetDeg = 0.0;
    }

    @Override
    public void loop() {
        follower.update();

// Get Limelight result first (this frame)
        LLResult result = limelight.getLatestResult();

// Pedro heading (deg)
        double pedroYawDeg = Math.toDegrees(follower.getPose().getHeading());

// 1) One-time seed: when we first see tags, use MT1 yaw to compute an offset
        if (!mt2YawSeeded && result != null && result.isValid()) {
            Pose3D mt1 = result.getBotpose(); // MegaTag1 pose (field-space)
            if (mt1 != null) {
                double mt1YawDeg = mt1.getOrientation().getYaw();

                yawOffsetDeg = wrapDeg(mt1YawDeg - pedroYawDeg);
                mt2YawSeeded = true;
            }
        }

// 2) Feed MT2 the corrected yaw every loop (Limelight expects DEGREES)
        double yawForLLDeg = mt2YawSeeded ? wrapDeg(pedroYawDeg + yawOffsetDeg) : wrapDeg(pedroYawDeg);
        limelight.updateRobotOrientation(yawForLLDeg);

// 3) Telemetry: show MT1 and MT2 poses when available
        if (result != null && result.isValid()) {
            telemetry.addData("LL", "Valid");

            Pose3D mt1 = result.getBotpose();
            if (mt1 != null) {
                double mt1YawDeg = mt1.getOrientation().getYaw();
                telemetry.addData("MT1 yaw (deg)", mt1YawDeg);
            }

            Pose3D mt2 = result.getBotpose_MT2();
            if (mt2 != null) {
                telemetry.addData("MT2 x (m)", mt2.getPosition().x);
                telemetry.addData("MT2 y (m)", mt2.getPosition().y);
                telemetry.addData("MT2 yaw (deg)", mt2.getOrientation().getYaw());
            } else {
                telemetry.addData("MT2", "null (not computed yet / no tags)");
            }

            telemetry.addData("pedroYawDeg", pedroYawDeg);
            telemetry.addData("yawOffsetDeg", yawOffsetDeg);
            telemetry.addData("mt2YawSeeded", mt2YawSeeded);

        } else {
            telemetry.addData("Limelight", "No Targets");
            telemetry.addData("mt2YawSeeded", mt2YawSeeded);
        }
        
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }

//        telemetry.addData("servo actualPos",IndexerR.getPosition());
//        telemetry.addData("timer:",shootTimeTimer.getElapsedTimeSeconds());
//        telemetry.addData("Launching",launching);
//        telemetry.addData("Flywheel speed (encoder ticks)", flywheel1.getVelocity());
//        telemetry.addLine("Current Flywheel RPM: " + currentRPM + " Set RPM: " + targetRPM);
//        telemetry.addData("position", follower.getPose());
//        telemetry.addData("velocity", follower.getVelocity());
//        telemetry.addData("automatedDrive", automatedDrive);
        telemetry.addData("Aim kP", aim_kP);
        telemetry.addData("Aim kF", aim_kF);
        telemetry.addData("Aim kD", aim_kD);



        /* --------------------------- GAMEPAD 1 -- DRIVER CONTROLS --------------------------------*/

        //Autoaim (need to implement limelight)
        if (gamepad1.bWasPressed())     //Aim to Goal
        {
            goalOriented = !goalOriented;
            aimErrSum = 0.0;
            aimLastErr = 0.0;
            lastWrappedAngle = 0.0;
            aimTimer.resetTimer();
        }
        //-----------------------------TUNING CODE ONLY, DELETE LATER-------------------------------
        if (gamepad1.rightBumperWasPressed())
        {
            aim_kP +=0.01;
        }
        if (gamepad1.leftBumperWasPressed())
        {
            aim_kP -=0.01;
        }
        if (gamepad1.dpadRightWasPressed())
        {
            aim_kF +=0.01;
        }
        if (gamepad1.dpadLeftWasPressed())
        {
            aim_kF -=0.01;
        }
        if (gamepad1.dpadUpWasPressed())
        {
            aim_kD +=0.01;
        }
        if (gamepad1.dpadDownWasPressed())
        {
            aim_kD -=0.01;
        }
        //------------------------------------------------------------------------------------------
        // Slowmode toggle
        if (gamepad1.aWasPressed()) {
            slowDrive = !slowDrive;
        }

        // Get base drive commands
        double forwardCmd = -gamepad1.left_stick_y;
        double strafeCmd  =  gamepad1.left_stick_x;
        double rotateCmd;

        // Determine rotation command based on goal-oriented mode
        if (goalOriented) {
            Pose robot = follower.getPose();
            Pose goal  = redAlliance ? RED_GOAL : BLUE_GOAL;

            double dx = goal.getX() - robot.getX();
            double dy = goal.getY() - robot.getY();

            double desiredHeading = Math.atan2(dy, dx);           // radians
            double err = wrapRadians(desiredHeading - robot.getHeading());

            rotateCmd = headingPid(err);

            // Optional: if you want a "deadband" so it doesn't twitch when nearly aligned:
            if (Math.abs(err) < Math.toRadians(0.5)) {
                rotateCmd = 0.0;
            }

            telemetry.addData("GoalMode", "ON");
            telemetry.addData("Alliance", redAlliance ? "RED" : "BLUE");
            telemetry.addData("AimErrDeg", Math.toDegrees(err));

        }
        else
        {
            rotateCmd = -gamepad1.right_stick_x;
            telemetry.addData("GoalMode", "OFF");
        }

        // Apply slow mode multiplier if active
        if (slowDrive) {
            forwardCmd *= slowModeMultiplier;
            strafeCmd  *= slowModeMultiplier;
            rotateCmd  *= slowModeMultiplier;
        }

        // Apply drive commands
        follower.setTeleOpDrive(forwardCmd, -strafeCmd, rotateCmd);


        // Intake control
        intake.setPower(2 * gamepad1.right_trigger);


        /* -------------------------FLYWHEEL CONTROL SECTION----------------------------------------*/
        //Ensure reasonable values
        targetRPM = Range.clip(abs(targetRPM), 0, maxRPM);
        kP = Math.max(0, kP);
        kI = Math.max(0, kI);
        kD = Math.max(0, kD);
        currentRPM = (flywheel2.getVelocity() / 28) * 60.0; //converts Ticks per second into RPM
        double dt = flywheelTimer.getElapsedTimeSeconds(); //gets change in time since last loop
        dt = Math.max(dt, 1e-6); // avoid insane derivative at very small d
        flywheelTimer.resetTimer(); //resets timer to start counting again
        double error = targetRPM - currentRPM;
        errorSum += error * dt;
        double derivative = (error - lastError) / dt;
        kF = kFmultiplier * (targetRPM / maxRPM);
        double output =
                (kF) + (kP * error) + (kI * errorSum) + (kD * derivative);  //PID controller

        Flywheel_Power = Range.clip(output, 0.0, 1.0);

//        flywheel1.setPower(Flywheel_Power);
//        flywheel2.setPower(Flywheel_Power); //COMMENTED OUT FOR GOAL AIMING TESTING

        lastError = error;
        /* -------------------------END FLYWHEEL CONTROL SECTION------------------------------------*/

        /* ------------------------- SHOOTING CONTROL SECTION --------------------------------------*/
        if (gamepad1.xWasPressed() && !launching) {
            shootTimeTimer.resetTimer();
            IndexerR.setPosition(GATE_OPEN_POS); // open gate
            launching = true;
        }
        if (launching) {
            double t = shootTimeTimer.getElapsedTimeSeconds();

            if (t > 0.4) {
                elevator.setPower(0.6);
                intake.setPower(0.5);
            }

            if (t > 2.3) {
                elevator.setPower(0.0);
                intake.setPower(0.0);
                IndexerR.setPosition(GATE_CLOSED_POS); // close gate
                launching = false;
            }
        }
        /* ---------------------- END SHOOTING CONTROL SECTION -------------------------------------*/
        //Extake
        if (abs(gamepad1.left_trigger) > 0 && !launching) {
            intake.setPower(-gamepad1.left_trigger);
            elevator.setPower(-gamepad1.left_trigger);
        }

        /* GAMEPAD 2 -- ENGINEER CONTROLS */
        if (gamepad2.rightBumperWasPressed()) //if during match gate drifts upward, can adjust.
        {
            GATE_OPEN_POS += .01;
            GATE_CLOSED_POS += .01;
        }
        if (gamepad2.leftBumperWasPressed()) {
            GATE_OPEN_POS -= .01;
            GATE_CLOSED_POS -= .01;
        }
        if (gamepad2.yWasPressed()) //toggle which goal to aim at
        {
            redAlliance = !redAlliance;
        }
        //Add flywheel shutoff toggle, rpm setpoint override and manual control of gate here.
        //also add manual adjustments to how slow slow mode is

        telemetry.update();
    }

    private static double wrapDeg(double deg) {
        deg = ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
        return deg;
    }

    private double wrapRadians(double angle) {
        // Step 1: normalize to [-pi, pi]
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;

        // Step 2: enforce continuity (unwrap)
        double delta = angle - lastWrappedAngle;

        if (delta > Math.PI) {
            angle -= 2.0 * Math.PI;
        } else if (delta < -Math.PI) {
            angle += 2.0 * Math.PI;
        }

        lastWrappedAngle = angle;
        return angle;
    }


    private double headingPid(double errRad) {
        double dt = aimTimer.getElapsedTimeSeconds();
        dt = Math.max(dt, 1e-6);
        aimTimer.resetTimer();

        aimErrSum += errRad * dt;

        // Optional: anti-windup clamp (good idea once you start tuning)
        aimErrSum = Range.clip(aimErrSum, -1.0, 1.0);

        double derr = (errRad - aimLastErr) / dt;
        aimLastErr = errRad;

        double out = (aim_kP * errRad) + (aim_kI * aimErrSum) + (aim_kD * derr) + (aim_kF * Math.signum(errRad));

        // Clip to legal joystick-like rotate command
        return Range.clip(out, -1.0, 1.0);
    }
}
