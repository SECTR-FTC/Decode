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
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;


/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 */
@Configurable
@Disabled
@TeleOp(name = "Mecanum", group = "Match")

public class Mecanum extends OpMode {
    // This declares the four motors needed
    double GATE_OPEN_POS = 0.25;
    double GATE_CLOSED_POS = 0.55;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotorEx elevator = null;
    private DcMotor intake = null;

    private Timer timer = new Timer();

    private Servo IndexerR = null;
    private DcMotorEx flywheel1 = null;
    private DcMotorEx flywheel2 = null;
    private boolean flywheelOn = false;
    private boolean slowDrive = false;



    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;


    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;

    public boolean tuning = true;

    public static double kP = 10.0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 12.9;

    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    boolean launching = false;




    public static double setPos= 0.6;
    public static double actualPos= 0.0;


    @Override()
    public void init() {
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
        flywheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        elevator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

//        flywheel1.setVelocityPIDFCoefficients(kP, kI, kD, kF);
//        flywheel2.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        //Set Zero Power Behavior
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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

    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
        IndexerR.setPosition(0.55);
    }

    @Override
    public void loop() {
        //Call this once per loop
        actualPos = IndexerR.getPosition();
        telemetry.addLine("servo actualPos:" + actualPos);
        telemetry.addLine("timer:" + timer.getElapsedTimeSeconds());
        telemetry.addLine("Launching:" + launching);
        telemetry.addLine("Flywheel speed (encoder ticks)" + flywheel1.getVelocity());
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetryM.update();

        if (flywheelOn)
        {
            flywheel1.setPower(.45);
            flywheel2.setPower(.45);
        }
        else if (!flywheelOn)
        {
            flywheel1.setPower(0);
            flywheel2.setPower(0);

        }

        /* GAMEPAD 1 -- DRIVER CONTROLS */
        intake.setPower(2*gamepad1.right_trigger);
        if (!slowDrive){
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            drive(-gamepad1.left_stick_y*.3, gamepad1.left_stick_x*.3, gamepad1.right_stick_x*.3);
        }

        if (gamepad1.aWasPressed()){
            slowDrive = !slowDrive;
        }

        if (abs(gamepad1.left_trigger) > 0.15 && !launching) {                //can go to gamepad2
            intake.setPower(-gamepad1.left_trigger);
            elevator.setPower(-gamepad1.left_trigger);
        }

        if (gamepad1.bWasPressed()) {
            flywheelOn = !flywheelOn;
        }
        if (gamepad1.xWasPressed()) {                                          //can go to gamepad2
            timer.resetTimer();
            IndexerR.setPosition(0.3); //open gate
            launching = true;

        }
        if (launching && timer.getElapsedTimeSeconds() > 0.5) {
            elevator.setPower(1.0);
            intake.setPower(0.8);
        }
        if (launching && timer.getElapsedTimeSeconds() > 2.5)
        {
            elevator.setPower(0.0);
            IndexerR.setPosition(0.55);
            launching = false;
        }
        /* END DRIVER CONTROLS */
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
