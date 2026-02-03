
package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "FlywheelTuning", group = "test")
@Configurable // Panels
public class flywheelTuner extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance

    private Timer pathTimer, actionTimer, opmodeTimer, flywheelTimer, shootTimeTimer; //Create timers

    private DcMotor elevator = null;
    private DcMotor intake = null;
    private Servo IndexerR = null;
    private DcMotorEx flywheel1 = null;
    private DcMotorEx flywheel2 = null;

    private Limelight3A limelight;

    boolean launching = false;
    boolean indexersSpinning = false;
    //THIS IS TUNING CODE ONLy
    public boolean tuning = true;

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


    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        shootTimeTimer = new Timer();
        opmodeTimer.resetTimer();

        flywheelTimer = new Timer();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        elevator = hardwareMap.get(DcMotor.class, "elevator");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "FlywheelLeft");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "FlywheelRight");
        IndexerR = hardwareMap.get(Servo.class, "IndexerR");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(8); // Switch to pipeline number 0


        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel1.setDirection(DcMotor.Direction.FORWARD);
        flywheel2.setDirection(DcMotor.Direction.REVERSE);
        IndexerR.setDirection(Servo.Direction.REVERSE);
        elevator.setDirection(DcMotorSimple.Direction.FORWARD); //UPDATE THIS FOR RED

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Ready to run");
        telemetry.addData("Flywheel RPM", currentRPM);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("PID Output", "Ready to run");
        telemetry.update();
    }


    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        IndexerR.setPosition(0.55);
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

        if (tuning) {
            if (gamepad2.leftBumperWasPressed()) {
                kP -= .001;
            }
            if (gamepad2.rightBumperWasPressed()) {
                kP += .001;
            }
            if (gamepad2.xWasPressed()) {
                kFmultiplier -= .01;
            }
            if (gamepad2.yWasPressed()) {
                kFmultiplier += .01;
            }
            if (gamepad2.aWasPressed()) {
                kI += 0.0001;
            }
            if (gamepad2.bWasPressed()) {
                kI -= 0.0001;
            }
            if (gamepad2.rightStickButtonWasPressed()) {
                kFmultiplier *= 2;
            }
            if (gamepad2.leftStickButtonWasPressed()) {
                kFmultiplier *= .5;
            }
            if (gamepad2.dpadUpWasPressed()) {
                targetRPM += 100;
            }
            if (gamepad2.dpadDownWasPressed()) {
                targetRPM -= 100;
            }
            if (gamepad2.dpadLeftWasPressed()) {
                kD += 0.00001;
            }
            if (gamepad2.dpadRightWasPressed()) {
                kD -= 0.00001;
            }
        }

        if (gamepad1.aWasPressed() && !launching) {
            shootTimeTimer.resetTimer();
            IndexerR.setPosition(0.3); // open gate
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
                IndexerR.setPosition(0.55); // close gate
                launching = false;
            }
        }


        //Ensure reasonable values
        targetRPM = Range.clip(targetRPM, 0, maxRPM);
        kP = Math.max(0, kP);
        kI = Math.max(0, kI);
        kD = Math.max(0, kD);

        /* FLYWHEEL CONTROL SECTION*/
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

        /*END FLYWHEEL CONTROL SECTION*/


        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Current Flywheel RPM:", currentRPM, " Set Speed:", targetRPM);
        panelsTelemetry.addData("Flywheel RPM", currentRPM);
        panelsTelemetry.addData("Target RPM", targetRPM);
        panelsTelemetry.addData("PID Output", output);
        panelsTelemetry.addData("RPM Error", error);
        panelsTelemetry.addData("kP", kP);
        panelsTelemetry.addData("kI", kI);
        panelsTelemetry.addData("kD", kD);
        panelsTelemetry.addData("kF", kF);
        panelsTelemetry.addData("kFMultiplier", kFmultiplier);

        panelsTelemetry.update(telemetry);

    }

//    public void shootTime() {
//        //timer is reset before path is started
//
//        shootTimeTimer.resetTimer();
//        IndexerR.setPosition(0.3); //open gate
//        launching = true;
//
//
//        if (shootTimeTimer.getElapsedTimeSeconds() > 0.4) {
//                elevator.setPower(0.6);
//                intake.setPower(0.5);
//
//        }
//
//        /* This is the area where you should add in the steps to score the 3 preloads */
//        if (pathTimer.getElapsedTimeSeconds() > 3 || shootTimeTimer.getElapsedTimeSeconds() > 2.3) { //Wait 3 seconds to score balls before moving on
//            elevator.setPower(0.0);
//            IndexerR.setPosition(0.55);
//            intake.setPower(0);
//            launching = false;
//
//        }
//    }

//    public void shootVelocity(int Shots) {
//        //timer is reset before path is started
//        int shots = Shots;
//        while (shots > 0) {
//            if (flywheel2.getVelocity() < (currentRPM-25) ) { //while flywheel velocity is not within 25 tps of the target velocity
//                IndexerR.setPosition(0.55);
//                if (indexersSpinning) {
//                    indexersSpinning = false;
//                    shots--;
//                }
//            } else {
//                IndexerL.setPower(INDEXER_SPEED);
//                IndexerR.setPower(INDEXER_SPEED);
//                indexersSpinning = true;
//            }
//
//            }
//
//        /* This is the area where you should add in the steps to score the 3 preloads */
//        if (pathTimer.getElapsedTimeSeconds() > 2.5 || shots == 0) { //Wait 2.5 seconds to score balls before moving on remid
//            elevator.setPower(-0.2); //de-activate systems
//            IndexerL.setPower(0);
//            IndexerR.setPower(0);
//            intake.setPower(0);
//        }
//    }

}