package org.firstinspires.ftc.teamcode.robot;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.interfaces.IDrive;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot.State;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.states.*;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

public class Robot {

    public static boolean resetEncoders = true;


    public interface YawOverrride {
        double getYaw();
    }

    private IRobot currentState;
    private final IDrive drive;
    private final JoystickWrapper joystick;
    private final HorizontalSlideController horizontalSlideController;
    private VerticalSlideController verticalSlideController;
    private final ClawSlideController clawSlideController;
    private final DualServoSlideController dualServoSlideController;

    private final Servo clawAngleServo;
    private final Servo clawRotationServo;
    private final Servo clawServo;
    private final Servo clawHorizontalAngleServo;
    private final CRServo intakeServo;
    private final Servo intakeAngleServo;
    private final Servo intakeKnuckleServo;
    private final Servo intakeRotationServo;
    private final Servo intakeClawServo;

    private final HardwareMap hardwareMap;

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

    DriveToPointController driveController = new DriveToPointController();

    public Robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        this(hardwareMap, gamepad1, gamepad2, telemetry, false);
    }

    public Robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, boolean isAuto) {
        joystick = new JoystickWrapper(gamepad1, gamepad2);
        this.hardwareMap = hardwareMap;


        if (isAuto) {
            horizontalSlideController = new HorizontalSlideController(hardwareMap, "horizontalSlide1", RoboSapiensTeleOp.Params.HORIZONTAL_SLIDE_MAX_POSITION, 0, false);
        } else {
            horizontalSlideController = new HorizontalSlideController(hardwareMap, "horizontalSlide1", RoboSapiensTeleOp.Params.HORIZONTAL_SLIDE_MAX_POSITION-50, 0, false);
        }

        verticalSlideController = new VerticalSlideController(hardwareMap, "verticalSlide1", "verticalSlide2", "clawSliderEncoder", true, RoboSapiensTeleOp.Params.VERTICAL_SLIDE_MAX_POSITION, 0, false);
        clawSlideController = new ClawSlideController(hardwareMap, "clawSliderCR", "verticalSlide2", RoboSapiensTeleOp.Params.CLAW_SLIDER_FORWARD, RoboSapiensTeleOp.Params.CLAW_SLIDER_BACK);
        dualServoSlideController = new DualServoSlideController(hardwareMap, "clawSliderCR1","clawSliderCR2", "clawSliderEncoder", RoboSapiensTeleOp.Params.CLAW_SLIDER_FORWARD, RoboSapiensTeleOp.Params.CLAW_SLIDER_BACK);

        clawAngleServo = hardwareMap.get(Servo.class, "clawAngleServo");
        clawRotationServo = hardwareMap.get(Servo.class, "clawRotationServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawHorizontalAngleServo = hardwareMap.get(Servo.class, "clawHorizontalAngleServo");

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        intakeAngleServo = hardwareMap.get(Servo.class, "intakeAngleServo");
        intakeKnuckleServo = hardwareMap.get(Servo.class, "intakeKnuckleServo");
        intakeRotationServo = hardwareMap.get(Servo.class, "intakeRotationServo");
        intakeClawServo = hardwareMap.get(Servo.class, "intakeClawServo");

        instanceStateMap.put(State.INITIAL, () -> new InitialState(joystick));
        instanceStateMap.put(State.INTAKING, () -> new IntakingState(joystick));
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

        /*
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // Set pipeline for AprilTag detection
        limelight.start();
        */


        WallPickUpState._lastPose = null;
        SpecimenHangState._lastPose = null;
        DroppingL1State._lastPose = null;


        switchState(State.INTAKINGCLAW);
        if (!isAuto) {
            double startingHeading = Math.toRadians(90);
            Localizer localizer = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick, new Pose2d(0,0,startingHeading));
            drive = new AngleDrive(hardwareMap, false, localizer);
        } else {
            drive = null;
        }

        resetEncoders=true;


        initPid();
    }

    public void setYawOverride(YawOverrride inOverride) {
        doOverrideYaw = inOverride;
    }
    public Robot setHorizontalSlideTargetPosition(int target) {
        horizontalSlideController.setTargetPosition(target);
        return this;
    }

    public Robot setVerticalSlideTargetPosition(int target) {
        verticalSlideController.setTargetPosition(target);
        return this;
    }

    public Robot increaseVerticalSlideTargetPosition(int target) {
        verticalSlideController.increaseTargetPosition(target);
        return this;
    }

    public Robot increaseHorizontalSlideTargetPosition(int target) {
        horizontalSlideController.increaseTargetPosition(isSlowMode ? (int) (target * .3) : target);
        return this;
    }

    public Robot increaseClawSlideTargetPosition(int target) {
        dualServoSlideController.increaseTargetPosition(target);
        return this;
    }

    public Robot setClawSlideTargetPosition(int target) {
        clawSlideController.setTargetPosition(target);
        return this;
    }

    public Robot setDualSlideTargetPosition(int target) {
        dualServoSlideController.setTargetPosition(target);
        return this;
    }

    public int getDualSlideTargetPosition() {
        return dualServoSlideController.getCurrentPosition();
    }

    public Robot setClawAnglePosition(double position) {
        clawAngleServo.setPosition(position);
        return this;
    }

    public double getClawAnglePosition() {
        return clawAngleServo.getPosition();
    }

    public Robot setClawHorizontalAnglePosition(double position) {
        clawHorizontalAngleServo.setPosition(position);
        return this;
    }

    public Robot autoHorizontalPosWall(Telemetry telemetry) {
    double angle = 0;

        if(doOverrideYaw == null) {
            angle = drive.getYaw();
        } else {
            angle = doOverrideYaw.getYaw();
        }

        double slope = (RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_LEFT-RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER)/(33.1458);
        double intercept = RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER;
        double pos = Range.clip(slope*angle+intercept, RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_LEFT, RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_RIGHT);
        telemetry.addData("clawHorizontalAngleServo min", slope*RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_RIGHT+intercept);
        telemetry.addData("clawHorizontalAngleServo max", slope*RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_LEFT+intercept);
        telemetry.addData("clawHorizontalAngleServo attempted pos", slope*angle+intercept);
        telemetry.addData("clawHorizontalAngleServo actual pos", pos);

        clawHorizontalAngleServo.setPosition(pos);
        return this;
    }

    public Robot autoHorizontalPosHang() {
        double angle = 0;

        if(doOverrideYaw == null) {
            angle = drive.getYaw();
        } else {
            angle = doOverrideYaw.getYaw();
        }

        double slope = (RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_RIGHT-RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER)/(33.1458);
        double intercept = RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER;
        double pos = Range.clip(slope*angle+intercept, RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_LEFT, RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_RIGHT);

        clawHorizontalAngleServo.setPosition(pos);
        return this;
    }

    public Robot autoHorizontalPosBucket(Telemetry telemetry) {
        double angle = 0;

        if(doOverrideYaw == null) {
            angle = drive.getYaw()+45;
        } else {
            angle = doOverrideYaw.getYaw()+45;
        }

        double slope = (RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_LEFT-RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER)/(33.1458);
        double intercept = RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER;
        double pos = Range.clip(slope*angle+intercept, RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_LEFT, RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_RIGHT);
        telemetry.addData("clawHorizontalAngleServo min", slope*RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_RIGHT+intercept);
        telemetry.addData("clawHorizontalAngleServo max", slope*RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_LEFT+intercept);
        telemetry.addData("clawHorizontalAngleServo attempted pos", slope*angle+intercept);
        telemetry.addData("clawHorizontalAngleServo actual pos", pos);

        clawHorizontalAngleServo.setPosition(pos);
        return this;
    }

    public double getClawHorizontalAnglePostion() {
        return clawHorizontalAngleServo.getPosition();
    }

    public Robot setClawRotationPosition(double position) {
        clawRotationServo.setPosition(position);
        return this;
    }

    public double getClawRotationPosition() {
        return clawRotationServo.getPosition();
    }

    public Robot setClawPosition(double position) {
        clawServo.setPosition(position);
        return this;
    }



    public Robot setIntakeKnuckleServo(double position) {
        intakeKnuckleServo.setPosition(position);
        return this;
    }
    public double getIntakeKnuckleServo() {
        return intakeKnuckleServo.getPosition();
    }


    public Robot setIntakeRotationServo(double position) {
        intakeRotationServo.setPosition(position);
        return this;
    }
    public double getIntakeRotationServo() {
        return intakeRotationServo.getPosition();
    }

    public Robot setIntakeClawServo(double position) {
        intakeClawServo.setPosition(position);
        return this;
    }
    public double getIntakeClawServo() {
        return intakeClawServo.getPosition();
    }


    public Robot setIntakePower(double power) {
        intakeServo.setPower(power);
        return this;
    }

    public Robot setIntakeAngleServo(double position) {
        intakeAngleServo.setPosition(position);
        return this;
    }

    public double getIntakeAngleServo() {
        return intakeAngleServo.getPosition();
    }

    public int getVerticalSlidePosition() {
        return verticalSlideController.getCurrentPosition();
    }

    public int getHorizontalSlidePosition() {
        return horizontalSlideController.getCurrentPosition();
    }

    public int getClawSlidePosition() {
        return clawSlideController.getCurrentPosition();
    }
    public void newVerticalControlPidTuning() {
        verticalSlideController = new VerticalSlideController(hardwareMap, "verticalSlide2", "verticalSlide1", "clawSliderEncoder", true, RoboSapiensTeleOp.Params.VERTICAL_SLIDE_DROP_L2, 0,false);
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
        servoHashMap.put("clawAngleServo", clawAngleServo);
        servoHashMap.put("clawHorizontalAngleServo", clawHorizontalAngleServo);
        servoHashMap.put("clawRotationServo", clawRotationServo);
        servoHashMap.put("clawServo", clawServo);
        servoHashMap.put("intakeAngleServo", intakeAngleServo);
        servoHashMap.put("intakeKnuckleServo", intakeKnuckleServo);
        servoHashMap.put("intakeRotationServo", intakeRotationServo);
        servoHashMap.put("intakeClawServo", intakeClawServo);

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

    public void setPose(Pose2d pose) {
        drive.setPose(pose);
    }

    public Pose2d getPose() {
        return drive.getPose();
    }

    long dTime = System.currentTimeMillis();
    long minTime = dTime;

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

            boolean tempIgnoreDrift = false;
            if( Math.abs( joystick.gamepad1GetLeftStickX() ) > 0  ||  Math.abs( joystick.gamepad1GetLeftStickY() ) > 0 ||
                    Math.abs( joystick.gamepad1GetRightStickX() ) > 0  ||  Math.abs( joystick.gamepad1GetRightStickY() ) > 0) {
                tempIgnoreDrift = true;
                setDriftMode(false, 0, 0);
                disableAutoMode();
            }

            if(isAutoMode) {

                Vector3D powers = driveController.calculate( targetX, targetY, targetHeading, drive.getPose(), telemetry);

                telemetry.addData("POSE Heading: ", Math.toDegrees(drive.getPose().heading.toDouble()));
                telemetry.addData("POSE Heading Power: ", powers.getZ());

                telemetry.addData("POSE x: ", drive.getPose().position.x);
                telemetry.addData("POSE x Power: ", powers.getX());

                telemetry.addData("POSE y: ", drive.getPose().position.y);
                telemetry.addData("POSE t Power: ", powers.getY());

                AngleDrive angleDrive = (AngleDrive)drive;
                angleDrive.MoveMecanumPidToPoint(powers.getX(), powers.getY(), powers.getZ());
                //drive.updateRaw(telemetry, false, powers.getX(), powers.getY(), powers.getZ(), 0, 1, 1);

            } else if(isDriftMode &&  !tempIgnoreDrift) {
               drive.updateRaw(telemetry, false, driftXValue, driftYValue, joystick.gamepad1GetRightStickX(), joystick.gamepad1GetRightStickY(), 1, 1);
            } else {
                drive.update(telemetry, joystick, isSlowMode ? .4 : 1, isSlowMode ? .3 : 1);
            }

        telemetry.addData("State:", getCurrentState().name());
        currentState.execute(this, telemetry);
        horizontalSlideController.update(telemetry);
        verticalSlideController.update(telemetry);
        dualServoSlideController.update(telemetry);

        telemetry.addData("Delta Time", System.currentTimeMillis()-dTime);
        dTime=System.currentTimeMillis();
        if (dTime<minTime) {
            minTime=dTime;
        }

        telemetry.addData("Min Time", minTime);

        telemetry.update();

    }



    public void executeAuto(Telemetry telemetry) {
        telemetry.addData("State:", getCurrentState().name());
        currentState.execute(this, telemetry);
        horizontalSlideController.update(telemetry);
        verticalSlideController.update(telemetry);
        dualServoSlideController.update(telemetry);
        telemetry.update();
    }



    public State getCurrentState() {
        if (currentState != null) {
            return currentState.getState();
        } else {
            return State.INVALID;
        }
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
        currentState = Objects.requireNonNull(instanceStateMap.get(newState)).get();
        currentState.initialize(this, prevState);
    }
}
