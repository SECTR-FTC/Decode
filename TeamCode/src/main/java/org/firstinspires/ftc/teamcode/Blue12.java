
package org.firstinspires.ftc.teamcode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Blue 12", group = "Blue")
@Configurable // Panels
public class Blue12 extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private Timer pathTimer, actionTimer, opmodeTimer; //Create timers
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private DcMotor elevator = null;
    private DcMotor intake = null;
    private Servo IndexerR = null;
    private DcMotorEx flywheel1 = null;

    private DcMotorEx flywheel2 = null;


    private ElapsedTime shootTimeTimer = new ElapsedTime();

    boolean launching = false;
    boolean indexersSpinning = false;


    static final double LAUNCHER_COUNTS_PER_MOTOR_REV = 28;    // GoBilda 5202 Series (6000 RPM)
    static final double LAUNCHER_RPM = 4200;
    // Ticks per second = (RPM / 60) * Ticks per Revolution
    static final double LAUNCHER_VELOCITY_TICKS_PER_SEC = (LAUNCHER_RPM / 60) * LAUNCHER_COUNTS_PER_MOTOR_REV;
    static final double INTAKE_SPEED = 1.0;
    static final double ELEVATOR_SPEED = 0.5;
    static final double INDEXER_SPEED = 1.0;



    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(start);

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        elevator = hardwareMap.get(DcMotor.class, "elevator");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "FlywheelLeft");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "FlywheelRight");
        IndexerR = hardwareMap.get(Servo.class, "IndexerR");

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
        telemetry.update();
    }


    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        IndexerR.setPosition(0.55);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Current Flywheel Speed:", flywheel1.getVelocity()," Set Speed:", flywheel1.getPower());
        panelsTelemetry.update(telemetry);

    }

    private final Pose start = new Pose(25, 127.5, Math.toRadians(140));
    private final Pose shootPose = new Pose(42, 116,Math.toRadians(144));
    private final Pose pickup1start = new Pose(50, 86, Math.toRadians(0));
    private final Pose pickup1end = new Pose(19, 86, Math.toRadians(0));
    private final Pose pickup2start = new Pose(50, 62, Math.toRadians(0));
    private final Pose pickup2end = new Pose(18, 62, Math.toRadians(0));
    //private final Pose gate = new Pose();
    private final Pose pickup3start = new Pose(47, 40, Math.toRadians(0));

    private final Pose pickup3end = new Pose(12, 40, Math.toRadians(0)); //x = 12 to go further back
    private final Pose endPose = new Pose(45, 120, Math.toRadians(144));


    public class Paths {
        public PathChain StartToShoot;
        public PathChain ShootToPickup1Start;
        public PathChain Pickup1;
        public PathChain Pickup1EndToShoot;
        public PathChain ShootToPickup2Start;
        public PathChain Pickup2;
        public PathChain Pickup2EndToShoot;
        public PathChain ShootToPickup3Start;
        public PathChain Pickup3;
        public PathChain Pickup3EndToShoot;
        public PathChain Leave;

        public Paths(Follower follower) {
            StartToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    start,
                                    shootPose
                            )
                    ).setLinearHeadingInterpolation(start.getHeading(), shootPose.getHeading())

                    .build();

            ShootToPickup1Start = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    shootPose,
                                    new Pose(66.000, 84.000),
                                    pickup1start
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Pickup1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    pickup1start,
                                    pickup1end
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Pickup1EndToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    pickup1end,

                                    shootPose
                            )
                    ).setLinearHeadingInterpolation(pickup1end.getHeading(), shootPose.getHeading())

                    .build();

            ShootToPickup2Start = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    shootPose,
                                    new Pose(72.000, 60.000),
                                    pickup2start
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Pickup2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    pickup2start,

                                    pickup2end
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

//            Pickup2EndToGate = follower.pathBuilder().addPath(
//                            new BezierLine(
//                                    pickup2end,
//
//                                    gate
//                            )
//                ).setLinearHeadingInterpolation()
//                    .build();

            Pickup2EndToShoot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    pickup2end,
                                    new Pose(35.400, 67.644),
                                    shootPose
                            )
                    ).setLinearHeadingInterpolation(pickup2end.getHeading(), shootPose.getHeading())

                    .build();

            ShootToPickup3Start = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    shootPose,
                                    new Pose(78.000, 48),
                                    pickup3start
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Pickup3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    pickup3start,

                                    pickup3end
                            )
                    ).setLinearHeadingInterpolation(pickup3start.getHeading(), pickup3end.getHeading())

                    .build();

            Pickup3EndToShoot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    pickup3end,
                                    new Pose(32.811, 71.022),
                                    shootPose
                            )
                    ).setLinearHeadingInterpolation(pickup3end.getHeading(), shootPose.getHeading())

                    .build();

            Leave = follower.pathBuilder().addPath(
                            new BezierLine(
                                    shootPose,

                                    endPose
                            )
                    ).setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())

                    .build();
        } //To create the Red Auto, you will have to edit the control points here, but should be able to keep all the paths.
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: //Move from start to shoot
                follower.followPath(paths.StartToShoot, 1, true);
                flywheel1.setPower(.48);
                flywheel2.setPower(.48);
                setPathState(1); //setPathState resets timer
                break;
            case 1: //score preloads, move to pickup 1

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    /* Score Preload */
                    shootTime();
                    flywheel1.setPower(.48);
                    flywheel2.setPower(.48);
                    follower.followPath(paths.ShootToPickup1Start, 1, true);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the balls */
                    setPathState(2); //Once we're done scoring, we set the path state to the next state.

                }
                break;
            case 2: //grab balls at pickup 1
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1start's position */
                if(!follower.isBusy()) {
                    /* Activate intake, then move to pickup1end */
                    intake.setPower(1);
                    /* Since this is a pathChain, we can have Pedro hold the end point*/
                    follower.followPath(paths.Pickup1, .4, false);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Deactivate Intake */
                    intake.setPower(0);
                    /* Since this is a pathChain, we can have Pedro hold the end point */
                    follower.followPath(paths.Pickup1EndToShoot,1, true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /*shootTime*/
                    shootTime();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(paths.ShootToPickup2Start, 0.8, true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    intake.setPower(1);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(paths.Pickup2, .4,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    intake.setPower(0);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(paths.Pickup2EndToShoot,0.9, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /*shootTime*/
                    shootTime();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(paths.ShootToPickup3Start, 1, true);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    intake.setPower(1);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(paths.Pickup3, .4, false);
                    setPathState(9);
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    intake.setPower(0);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(paths.Pickup3EndToShoot,1, false);
                    setPathState(10);
                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {

                    shootTime();
                    follower.followPath(paths.Leave);
                    setPathState(11);
                }
                break;
            case 11:
                /* This case checks the robot's position and will wait unt+il the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }
    public void shootTime() {
        //timer is reset before path is started
        shootTimeTimer.reset();
        IndexerR.setPosition(0.3);
        launching = true;

        while (shootTimeTimer.seconds() < 2.3) {
            if (shootTimeTimer.seconds() > 0.4) {
                elevator.setPower(0.6);
                intake.setPower(0.5);
            }
        }
        /* This is the area where you should add in the steps to score the 3 preloads */
        if (pathTimer.getElapsedTimeSeconds() > 3 || shootTimeTimer.seconds() > 2.3) { //Wait 3 seconds to score balls before moving on remid
            elevator.setPower(0.0);
            IndexerR.setPosition(0.55);
            intake.setPower(0);
            launching = false;
        }
    }

//    public void shootVelocity(int Shots) {
//        //timer is reset before path is started
//        shots = Shots;
//        while (shots > 0) {
//            if (flywheel1.getVelocity() < (LAUNCHER_VELOCITY_TICKS_PER_SEC-25) ) { //while flywheel velocity is not within 25 tps of the target velocity
//                IndexerL.setPower(-0.1);
//                IndexerR.setPower(-0.1);
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

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}
