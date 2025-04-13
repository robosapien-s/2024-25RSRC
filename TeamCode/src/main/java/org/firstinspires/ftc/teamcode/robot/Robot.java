package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.AngleDrive.lineToConstantHeading;
import static org.firstinspires.ftc.teamcode.robot.AngleDrive.splineToConstantHeading;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.interfaces.IDrive;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot.State;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.states.*;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

public class Robot {
    public static Pose origin = new Pose(0,0,Math.toRadians(180));
    public static Pose middlePoseHang = new Pose(5.75, 22.75, Math.toRadians(180));
    public static Pose middlePoseWall = new Pose(7, 2, Math.toRadians(180));

    public static Pose bucketPose = new Pose(20.25, 130.15, Math.toRadians(-45));

    public static double leftPoseY = 40.5;

    public static int numCycles = 12;

    public static ArrayList<PathChain> pathChains = new ArrayList<>();

    public boolean isAuto;

    public static boolean resetEncoders = true;

    /**IMU offset in degrees*/
    public static double rotateAngleOffset = 0;


    public interface YawOverrride {
        double getYaw();
    }

    private IRobot currentState;
    private final IDrive drive;
    private final JoystickWrapper joystick;
    private final SlideRotationController slideRotationController;
    private SlideController slideController;


    private final Servo clawRotAndAngleServoRight;
    private final Servo clawRotAndAngleServoLeft;
    private final Servo clawServo;
    private final Servo intakeAngleServo;
    private final Servo hangServo;


    private final HardwareMap hardwareMap;

    private final Telemetry telemetry;

    private Limelight3A limelight;
    private boolean isAprilTagDetected = false;

    private final Map<State, Supplier<IRobot>> instanceStateMap = new HashMap<>();

    private boolean isAutoMode = false;
    private PIDEx xPid;
    private PIDEx yPid;

    private double targetX;
    private double targetY;
    private double targetHeading;


    private YawOverrride doOverrideYaw = null;

    private boolean isSlowMode = false;

    private boolean isDriftMode = false;
    private double driftXValue = 0;
    private double driftYValue = 0;


    private boolean driveTrainEnabled = true;

    DriveToPointController driveController = new DriveToPointController();

    public Robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        this(hardwareMap, gamepad1, gamepad2, telemetry, false, null);
    }

    public Robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, boolean isAuto) {
        this(hardwareMap, gamepad1, gamepad2, telemetry, isAuto, null);
    }


    public Robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, boolean isAuto, Follower inFollower) {

        this.isAuto = isAuto;

        joystick = new JoystickWrapper(gamepad1, gamepad2);
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        slideRotationController = new SlideRotationController(hardwareMap, "slideRotationController", RoboSapiensTeleOp.Params.SLIDE_ROTATION_MAX_POSITION, RoboSapiensTeleOp.Params.SLIDE_ROTATION_MIN_POSITION, false);
//hi

        slideController = new SlideController(hardwareMap, "verticalSlide1", "verticalSlide2", "clawSliderEncoder", true, RoboSapiensTeleOp.Params.SLIDE_MAX_POSITION, RoboSapiensTeleOp.Params.SLIDE_MIN_POSITION, false);
//        dualServoSlideController = new DualServoSlideController(hardwareMap, "clawSliderCR1","clawSliderCR2", "clawSliderEncoder", RoboSapiensTeleOp.Params.CLAW_SLIDER_FORWARD, RoboSapiensTeleOp.Params.CLAW_SLIDER_BACK);

        clawRotAndAngleServoRight = hardwareMap.get(Servo.class, "clawRotAndAngleServoRight");
        clawRotAndAngleServoLeft = hardwareMap.get(Servo.class, "clawRotAndAngleServoLeft");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        intakeAngleServo = hardwareMap.get(Servo.class, "intakeAngleServo");

        hangServo = hardwareMap.get(Servo.class, "hangServo");

//        leftHangServo = hardwareMap.get(CRServo.class, "leftHangServo");
//        rightHangServo = hardwareMap.get(CRServo.class, "rightHangServo");

//        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

//        intakeAngleServo = hardwareMap.get(Servo.class, "intakeAngleServo");
//        intakeKnuckleServo = hardwareMap.get(Servo.class, "intakeKnuckleServo");
//        intakeRotationServo = hardwareMap.get(Servo.class, "intakeRotationServo");
//        intakeClawServo = hardwareMap.get(Servo.class, "intakeClawServo");


        instanceStateMap.put(State.DROPPING_L1, () -> new DroppingL1State(joystick));
        instanceStateMap.put(State.DROPPING_L2, () -> new DroppingL2State(joystick));
        instanceStateMap.put(State.WALLPICKUP, () -> new WallPickUpState(joystick));
        instanceStateMap.put(State.SPECIMEN_HANG, () -> new SpecimenHangState(joystick));
        instanceStateMap.put(State.SERVO_TEST, () -> new ServoTestState(joystick));
        instanceStateMap.put(State.PID_TUNING, () -> new PidTuningState(joystick));
        instanceStateMap.put(State.INTAKINGCLAW, () -> new IntakingStateClaw(joystick));
        instanceStateMap.put(State.GO_TO_APRIL_TAG, () -> new GoToAprilTag(joystick));
        instanceStateMap.put(State.PICKUP_GROUND, () -> new PickUpGroundState(joystick));
        instanceStateMap.put(State.AUTO_DRIVE_TEST, () -> new AutoDriveTestState(joystick));
        instanceStateMap.put(State.PICKUP_GROUND_LEFT, () -> new PickUpGroundStateLeft(joystick));
        instanceStateMap.put(State.AUTO_PICKUP, () -> new AutoPickupState(joystick));
        instanceStateMap.put(State.SPECIMEN_HANG_FRONT, () -> new SpecimenHangFrontState(joystick));
        instanceStateMap.put(State.AUTO_WALLPICKUP, () -> new AutoWallPickUpState(joystick));
        instanceStateMap.put(State.AUTO_SPECIMEN_HANG, () -> new AutoSpecimenHangState(joystick));
        instanceStateMap.put(State.ROBOT_HANG, () -> new HangState(joystick));


        /*
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // Set pipeline for AprilTag detection
        limelight.start();
        */


//        WallPickUpState._lastPose = null;
//        SpecimenHangState._lastPose = null;
        DroppingL1State._lastPose = null;


        switchState(State.INTAKINGCLAW);

        setHangServo(RoboSapiensTeleOp.Params.HANG_SERVO_CLOSED);

        if (!isAuto) {
            double startingHeading = Math.toRadians(90);
            Follower follower;
            if (inFollower == null) {
                follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
            } else {
                follower = inFollower;
            }
            drive = new AngleDrive(hardwareMap, false, follower, rotateAngleOffset);
            createPathChains();
        } else {
            drive = new AngleDrive(hardwareMap, false, inFollower, rotateAngleOffset);
            setDriveTrainEnabled(false);
        }

        resetEncoders = true;
        rotateAngleOffset = 0;


        initPid();
    }

    public void pickUp() {
    }

    private void createPathChains() {
        pathChains.clear();

        Pose pose;
        for (int i = 0; i<numCycles; i++) {
            pose = new Pose(20.25, leftPoseY-1.5*i, Math.toRadians(180));
            pathChains.add(splineToConstantHeading(getFollower(), new Pose(1, 0.5, Math.toRadians(180)), middlePoseHang, pose, 1.5));
            pathChains.add(splineToConstantHeading(getFollower(), pose, middlePoseWall, new Pose(1, 0.5, Math.toRadians(180)), 1.5));
        }
        Pose startSwipePose = new Pose(20.25,22.5, Math.toRadians(180));
        Pose endSwipePose = new Pose(20.25, leftPoseY, Math.toRadians(180));

        PathChain firstDrop = getFollower().pathBuilder()
                .addPath(new BezierCurve(new Point(origin),new Point(new Pose(5.75, 20, Math.toRadians(180))), new Point(startSwipePose)))
                .setConstantHeadingInterpolation(startSwipePose.getHeading())
                .setZeroPowerAccelerationMultiplier(1.5)
                .addPath(new BezierLine(new Point(startSwipePose), new Point(endSwipePose)))
                .setConstantHeadingInterpolation(endSwipePose.getHeading())
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();

        pathChains.set(0, firstDrop);
    }


    public void start() {
        if (currentState != null) {
            currentState.start(this, telemetry);
        }
    }

    public void setYawOverride(YawOverrride inOverride) {
        doOverrideYaw = inOverride;
    }

    public Robot setSlideRotationTargetPosition(int target) {
        slideRotationController.setTargetPosition(target);
        return this;
    }

    public Robot setSlideTargetPosition(int target) {
        slideController.setTargetPosition(target);
        return this;
    }

    public Robot increaseSlideTargetPosition(int target) {
        slideController.increaseTargetPosition(target);
        return this;
    }

    public Robot increaseSlideRotationTargetPosition(int target) {
        slideRotationController.increaseTargetPosition(isSlowMode ? (int) (target * .3) : target);
        return this;
    }


    public Robot setRotAndAnglePosition(double[] positions) {
        clawRotAndAngleServoRight.setPosition(positions[0]);
        clawRotAndAngleServoLeft.setPosition(positions[1]);
        return this;
    }

    public double[] getClawAnglePosition() {
        return new double[]{clawRotAndAngleServoRight.getPosition(), clawRotAndAngleServoLeft.getPosition()};
    }

    public Robot setIntakeAnglePosition(double position) {
        intakeAngleServo.setPosition(position);
        return this;
    }

    public Robot autoHorizontalPosWall(Telemetry telemetry) {
        double angle = 0;

        if (doOverrideYaw == null) {
            angle = drive.getYaw();
        } else {
            angle = doOverrideYaw.getYaw();
        }

        double slope = (RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_LEFT - RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER) / (33.1458);
        double intercept = RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER;
        double pos = Range.clip(slope * angle + intercept, RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_LEFT, RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_RIGHT);
        telemetry.addData("clawHorizontalAngleServo min", slope * RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_RIGHT + intercept);
        telemetry.addData("clawHorizontalAngleServo max", slope * RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_LEFT + intercept);
        telemetry.addData("clawHorizontalAngleServo attempted pos", slope * angle + intercept);
        telemetry.addData("clawHorizontalAngleServo actual pos", pos);

        //clawHorizontalAngleServo.setPosition(pos);
        return this;
    }

    public Robot autoHorizontalPosHang() {
        double angle = 0;

        if (doOverrideYaw == null) {
            angle = drive.getYaw();
        } else {
            angle = doOverrideYaw.getYaw();
        }

        double slope = (RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_RIGHT - RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER) / (33.1458);
        double intercept = RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER;
        double pos = Range.clip(slope * angle + intercept, RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_LEFT, RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_RIGHT);

        //clawHorizontalAngleServo.setPosition(pos);
        return this;
    }


    public Robot autoHorizontalPosBucket(Telemetry telemetry) {
        double angle = 0;

        if (doOverrideYaw == null) {
            angle = drive.getYaw() + 45;
        } else {
            angle = doOverrideYaw.getYaw() + 45;
        }

        double slope = (RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_LEFT - RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER) / (33.1458);
        double intercept = RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER;
        double pos = Range.clip(slope * angle + intercept, RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_LEFT, RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_RIGHT);
        telemetry.addData("clawHorizontalAngleServo min", slope * RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_RIGHT + intercept);
        telemetry.addData("clawHorizontalAngleServo max", slope * RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_LEFT + intercept);
        telemetry.addData("clawHorizontalAngleServo attempted pos", slope * angle + intercept);
        telemetry.addData("clawHorizontalAngleServo actual pos", pos);

        //clawHorizontalAngleServo.setPosition(pos);
        return this;
    }

    public double getClawHorizontalAnglePostion() {
        return 0;//clawHorizontalAngleServo.getPosition();
    }


    public Robot setClawPosition(double position) {
        clawServo.setPosition(position);
        return this;
    }

    public double getClawPosition() {
        return clawServo.getPosition();
    }

    public void setHangServo(double position) {
        hangServo.setPosition(position);
    }

    public void disableHangServo() {
        hangServo.getController().pwmDisable();
    }

    public void enableHangServo() {
        hangServo.getController().pwmEnable();
    }

    public void setSlideRotationMaxPosition(int pos) {
        slideRotationController.setMaxPosition(pos);
    }

//    public void setLeftHangServo(double power) {
//        leftHangServo.setPower(power);
//    }
//
//    public void turnOffLeftHangServo() {
//        leftHangServo.getController().pwmDisable();
//    }
//
//
//
//    public void setRightHangServo(double power) {
//        rightHangServo.setPower(power);
//    }
//
//    public void turnOffRightHangServo() {
//        rightHangServo.getController().pwmDisable();
//    }


    public int getSlidePosition() {
        return slideController.getCurrentPosition();
    }

    public void setSlideMinPosition(int minPosition) {
        slideController.setMinPosition(minPosition);
    }

    public void setSlideMaxPosition(int maxPosition) {
        slideController.setMaxPosition(maxPosition);
    }

    public int getSlideRotationPosition() {
        return slideRotationController.getCurrentPosition();
    }

    public Robot setSlideRotationPosition(int pos) {
        slideRotationController.setTargetPosition(pos);
        return this;
    }

    public void newVerticalControlPidTuning() {
        slideController = new SlideController(hardwareMap, "slide2", "slide1", "sliderEncoder", true, RoboSapiensTeleOp.Params.SLIDE_DROP_L2, 0,false);
    }

    public Vector3D getDeadWheelLocation() {
        return drive.getRobotPosition();
    }

    public void toggleRobotSpeedMode() {
        isSlowMode = !isSlowMode;
    }
    public void setRobotSpeedSlow() {
        isSlowMode = true;
    }

    public void setRobotSpeedNormal() {
        isSlowMode = false;
    }

    public void setDriftMode(boolean enabled, double driftX, double driftY) {
        isDriftMode = enabled;
        driftXValue = driftX;
        driftYValue = driftY;
    }

    public boolean isDrifting() {
        return isDriftMode;
    }

    public HashMap<String, Servo> getServoForTesting() {
        HashMap<String, Servo> servoHashMap = new HashMap<>();
        servoHashMap.put("clawRotAndAngleServoRight", clawRotAndAngleServoRight);
        servoHashMap.put("clawRotAndAngleServoLeft", clawRotAndAngleServoLeft);
        servoHashMap.put("intakeAngleServo", intakeAngleServo);
        servoHashMap.put("clawServo", clawServo);
//        servoHashMap.put("intakeAngleServo", intakeAngleServo);
//        servoHashMap.put("intakeKnuckleServo", intakeKnuckleServo);
//        servoHashMap.put("intakeRotationServo", intakeRotationServo);
//        servoHashMap.put("intakeClawServo", intakeClawServo);

        return servoHashMap;
    }


    public void initPid() {
        PIDCoefficientsEx pidCoefficients = new PIDCoefficientsEx(
                0.1, 0, 0, 0, 0, 0
        );

        xPid = new PIDEx(pidCoefficients);
        yPid = new PIDEx(pidCoefficients);
    }

    public void setAutoTarget(double targetX, double targetY, double targetHeading) {

        //drive.setAutoMode(targetX, targetY);
       // drive.setTargetHeading(targetHeading);

        isAutoMode = true;
        this.targetX = targetX;
        this.targetY= targetY;
        this.targetHeading= targetHeading; //IN RADIANS
        drive.setAutoMode(targetX, targetY);
    }

    public void disableAutoMode() {
        isAutoMode = false;
        drive.disableAutoMode();
    }

    public boolean isAutoMode() {
        return isAutoMode;
    }

    public void setTargetHeading(double heading) {
        drive.setTargetHeading(heading);
    }

    public Follower getFollower() {
        return ((AngleDrive) drive).getFollower();
    }

    public void updateFollower() {
        ((AngleDrive) drive).updateFollower();
    }

    public void setPose(Pose pose) {
        drive.setPose(pose);
    }

    public Pose getPose() {
        return drive.getPose();
    }

    long dTime = System.currentTimeMillis();
    long minTime = dTime;

    public void setDriveTrainEnabled(boolean enabled) {
        driveTrainEnabled = enabled;
    }

    public void updateDriveTrainsRaw(Telemetry telemetry, boolean isLeftStickPressed, double leftStickX, double leftStickY, double rightStickX, double rightStickY, double speed, double rotSpeed) {
        drive.updateRaw(telemetry, false, leftStickX, -leftStickY, 0, 0, 1, 1);
    }

    public void moveMecanum(double xRotated, double yRotated, double anglePower){

        AngleDrive angleDrive = (AngleDrive) drive;
        angleDrive.MoveMecanumPidToPoint(xRotated, yRotated, anglePower);
    }

    public void execute(Telemetry telemetry) {
//        if (!isAprilTagDetected) {
//            pollForAprilTag(telemetry);
//        } else {
//            switchState(State.GO_TO_APRIL_TAG);
//        }
//        if (isAutoMode && limelight != null) {
//            Pose3D robotPos = pollForAprilTag(telemetry);
//            double xPower = 0;
//            double yPower = 0;
//            if (robotPos != null) {
//                xPower = xPid.calculate(targetX, robotPos.getPosition().x);
//                yPower = yPid.calculate(targetY, robotPos.getPosition().y);
//
//                if (xPower*xPower+yPower*yPower > 1) {
//                    double mag = Math.sqrt(xPower*xPower+yPower*yPower);
//                    xPower = xPower/mag;
//                    yPower = yPower/mag;
//                }
//            }
//
//            double rightStickX = Math.cos(targetHeading);
//            double rightStickY = -Math.sin(targetHeading);
//
//            telemetry.addData("X Power", xPower);
//            telemetry.addData("Y Power", yPower);
//            telemetry.addData("Right Stick X", rightStickX);
//            telemetry.addData("Right Stick Y", rightStickY);
//            telemetry.addData("Current Pos", robotPos);
//            telemetry.addData("Target X", targetX);
//            telemetry.addData("Target Y", targetY);
//
//
////            drive.updateRaw(telemetry, false, xPower, yPower, rightStickX, rightStickY, 1, 1);
//        } else {

           if(driveTrainEnabled) {
               boolean tempIgnoreDrift = false;
               if (Math.abs(joystick.gamepad1GetLeftStickX()) > 0 || Math.abs(joystick.gamepad1GetLeftStickY()) > 0 ||
                       Math.abs(joystick.gamepad1GetRightStickX()) > 0 || Math.abs(joystick.gamepad1GetRightStickY()) > 0) {
                   tempIgnoreDrift = true;
                   setDriftMode(false, 0, 0);
                   disableAutoMode();
               }

               if (isAutoMode) {

                   Vector3D powers = driveController.calculate(targetX, targetY, targetHeading, drive.getPose(), telemetry);

                   telemetry.addData("POSE Heading: ", Math.toDegrees(drive.getPose().getHeading()));
                   telemetry.addData("POSE Heading Power: ", powers.getZ());

                   telemetry.addData("POSE x: ", drive.getPose().getX());
                   telemetry.addData("POSE x Power: ", powers.getX());

                   telemetry.addData("POSE y: ", drive.getPose().getY());
                   telemetry.addData("POSE t Power: ", powers.getY());

                   AngleDrive angleDrive = (AngleDrive) drive;
                   angleDrive.MoveMecanumPidToPoint(powers.getX(), powers.getY(), powers.getZ());
                   //drive.updateRaw(telemetry, false, powers.getX(), powers.getY(), powers.getZ(), 0, 1, 1);

               } else if (isDriftMode && !tempIgnoreDrift) {
                   drive.updateRaw(telemetry, false, driftXValue, -driftYValue, joystick.gamepad1GetRightStickX(), joystick.gamepad1GetRightStickY(), 1, 1);
               } else {
                   telemetry.addData("yaw", drive.getYaw());
                   drive.update(telemetry, joystick, isSlowMode ? .4 : 1, isSlowMode ? .3 : 1);
               }
           }

        telemetry.addData("State:", getCurrentState().name());
        currentState.execute(this, telemetry);
        slideRotationController.update(telemetry);
        slideController.update(telemetry);
//        dualServoSlideController.update(telemetry);

        telemetry.addData("Delta Time", System.currentTimeMillis()-dTime);
        dTime=System.currentTimeMillis();
        if (dTime<minTime) {
            minTime=dTime;
        }

        //telemetry.addData("Min Time", minTime);

        telemetry.update();

    }



    public void executeAuto(Telemetry telemetry) {
        telemetry.addData("State:", getCurrentState().name());
        currentState.execute(this, telemetry);
        slideRotationController.update(telemetry);
        slideController.update(telemetry);
//        dualServoSlideController.update(telemetry);
        telemetry.update();
    }



    public State getCurrentState() {
        if (currentState != null) {
            return currentState.getState();
        } else {
            return State.INVALID;
        }
    }

    public MultiColorSampleDetector createColorSampleDetector(MultiColorSampleDetector.ClosestSamplePipeline.SampleColorPriority colorPriority) {
        return new MultiColorSampleDetector(hardwareMap, telemetry, colorPriority);
    }

    private Pose3D pollForAprilTag(Telemetry telemetry) {
        Pose3D botPose = null;
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            botPose = result.getBotpose();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();

            // Extract fiducial results (AprilTags)
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            if (!fiducialResults.isEmpty()) {
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    int tagId = fr.getFiducialId();
                    Pose3D robotPos = fr.getRobotPoseFieldSpace();
                    if (tagId != 0) {
                        botPose = robotPos;
                        telemetry.addData("AprilTag ID", tagId);
                        telemetry.addData("Bot Pose", robotPos.getPosition());
                        telemetry.addData("Capture Latency", "%.2f ms", captureLatency);
                        telemetry.addData("Targeting Latency", "%.2f ms", targetingLatency);
                    } else {
                        telemetry.addData("Tag ID", tagId);
                        telemetry.addData("Pose3D", "No pose data available.");
                    }
                }
            } else {
                telemetry.addData("AprilTag", "No AprilTag detected.");
            }
        } else {
            telemetry.addData("Limelight", "No valid result.");
        }

//        telemetry.update();
        return botPose;


    }

    public void switchState(State newState) {
        IRobot prevState = currentState;
        if(prevState != null) {
            prevState.dispose(this);
        }
        currentState = Objects.requireNonNull(instanceStateMap.get(newState)).get();
        currentState.initialize(this, prevState);
    }

    public boolean getIsAuto() {
        return isAuto;
    }
}
