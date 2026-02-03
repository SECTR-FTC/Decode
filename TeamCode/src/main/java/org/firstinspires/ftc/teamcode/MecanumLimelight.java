/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
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
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@Disabled
@TeleOp(name = "Mecanum With Limelight", group = "Test")

public class MecanumLimelight extends OpMode {
    private Timer pathTimer, actionTimer, opmodeTimer, flywheelTimer, shootTimeTimer;
    private double GATE_OPEN_POS = 0.25;
    private double GATE_CLOSED_POS = 0.55;

    // This declares the motors/servos needed
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
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
    double targetRPM = 500;
    static double maxRPM = 5800;
    double kP = 0.0135;
    double kI = 0.0001;
    double kD = 0.00007;
    double kF = 0.001;
    double kFmultiplier = 2.49;
    static double ilimit = 500;
    double errorSum = 0;
    double lastError = 0;


    // AUTOMATED DRIVE COEFFICIENTS AND VARIABLES
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.3;

    boolean launching = false;





    @Override()
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        shootTimeTimer = new Timer();
        opmodeTimer.resetTimer();
        flywheelTimer = new Timer();

        //Hardware Map:
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FR");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BL");
        backRightDrive = hardwareMap.get(DcMotor.class, "BR");
        elevator = hardwareMap.get(DcMotorEx.class, "elevator");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "FlywheelLeft");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "FlywheelRight");
        IndexerR = hardwareMap.get(Servo.class, "IndexerR");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        limelight.pipelineSwitch(8); // Switch to pipeline number 8

        // Set motor Direction - Currently set for Main chassis
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(DcMotor.Direction.FORWARD);
        flywheel1.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel2.setDirection(DcMotorEx.Direction.REVERSE);
        elevator.setDirection(DcMotorEx.Direction.FORWARD);

        IndexerR.setDirection(Servo.Direction.REVERSE);


        //stop and Reset encoders
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flywheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set motor encoder states
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        //Set Zero Power Behavior
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        elevator.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

//        imu = hardwareMap.get(IMU.class, "imu");
//        // This needs to be changed to match the orientation on your robot
//        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
//                RevHubOrientationOnRobot.LogoFacingDirection.UP;
//        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
//                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
//
//        RevHubOrientationOnRobot orientationOnRobot = new
//                RevHubOrientationOnRobot(logoDirection, usbDirection);
//        imu.initialize(new IMU.Parameters(orientationOnRobot));


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(39, 33))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
        IndexerR.setPosition(GATE_CLOSED_POS);
    }

    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();
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

        telemetry.addLine("servo actualPos:" + IndexerR.getPosition());
        telemetry.addLine("timer:" + shootTimeTimer.getElapsedTimeSeconds());
        telemetry.addLine("Launching:" + launching);
        telemetry.addLine("Flywheel speed (encoder ticks)" + flywheel1.getVelocity());
        telemetryM.debug("Current Flywheel RPM:", currentRPM, " Set RPM:", targetRPM);
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetryM.update();


        /* GAMEPAD 1 -- DRIVER CONTROLS */

        //Autoaim (not finished)
        if (gamepad1.rightBumperWasPressed()){
            drive(0, 0, 0);
        }

        //Slowmode
        if (!slowDrive){
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            drive(-gamepad1.left_stick_y*slowModeMultiplier, gamepad1.left_stick_x*slowModeMultiplier, gamepad1.right_stick_x*slowModeMultiplier);
        }
        if (gamepad1.aWasPressed()){
            slowDrive = !slowDrive;
        }

        //intake
        intake.setPower(2*gamepad1.right_trigger);


        /* -------------------------FLYWHEEL CONTROL SECTION----------------------------------------*/
        //Ensure reasonable values
        targetRPM = Range.clip(targetRPM, 0, maxRPM);
        kP = Math.max(0, kP);
        kI = Math.max(0, kI);
        kD = Math.max(0, kD);
        currentRPM = (flywheel2.getVelocity() / 28) * 60.0; //converts Ticks per second into RPM

        double dt = flywheelTimer.getElapsedTimeSeconds(); //gets change in time since last loop
        dt = Math.max(dt, 1e-6); // avoid insane derivative at very small dt

        flywheelTimer.resetTimer(); //resets timer to start counting again

        double error = targetRPM - currentRPM;
        errorSum += error * dt;

        double derivative = (error - lastError) / dt;

        kF = kFmultiplier * (targetRPM / maxRPM);

        double output =
                (kF) +
                        (kP * error) +
                        (kI * errorSum) +
                        (kD * derivative);

        Flywheel_Power = Range.clip(output, 0.0, 1.0);

        flywheel1.setPower(Flywheel_Power);
        flywheel2.setPower(Flywheel_Power);

        lastError = error;
        /* -------------------------END FLYWHEEL CONTROL SECTION------------------------------------*/

        //Shoot
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

        //Extake
        if (abs(gamepad1.left_trigger) > 0.15 && !launching) {                //can go to gamepad2
            intake.setPower(-gamepad1.left_trigger);
            elevator.setPower(-gamepad1.left_trigger);
        }

        /* GAMEPAD 2 -- ENGINEER CONTROLS */
        if (gamepad2.rightBumperWasPressed()) //if during match gate drifts upward, can adjust.
        {
            GATE_OPEN_POS += .01;
            GATE_CLOSED_POS += .01;
        }
        if (gamepad2.leftBumperWasPressed())
        {
            GATE_OPEN_POS -= .01;
            GATE_CLOSED_POS -= .01;
        }


    }


    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, abs(frontLeftPower));
        maxPower = Math.max(maxPower, abs(frontRightPower));
        maxPower = Math.max(maxPower, abs(backRightPower));
        maxPower = Math.max(maxPower, abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }
}
