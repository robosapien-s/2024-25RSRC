package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Color;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.interfaces.IDrive;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot.State;
import org.firstinspires.ftc.teamcode.opmodes.DriveTest;
import org.firstinspires.ftc.teamcode.states.*;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

public class Robot {

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





    public Robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        joystick = new JoystickWrapper(gamepad1, gamepad2);
        this.hardwareMap = hardwareMap;

        horizontalSlideController = new HorizontalSlideController(hardwareMap, "horizontalSlide1", DriveTest.Params.HORIZONTAL_SLIDE_MAX_POSITION, 0);
        verticalSlideController = new VerticalSlideController(hardwareMap, "verticalSlide2", "verticalSlide1", true, DriveTest.Params.VERTICAL_SLIDE_DROP_L2, 0);
        clawSlideController = new ClawSlideController(hardwareMap, "clawSliderCR", "verticalSlide1", DriveTest.Params.CLAW_SLIDER_FORWARD, DriveTest.Params.CLAW_SLIDER_BACK);
        dualServoSlideController = new DualServoSlideController(hardwareMap, "clawSliderCR1","clawSliderCR2", "verticalSlide1", DriveTest.Params.CLAW_SLIDER_FORWARD, DriveTest.Params.CLAW_SLIDER_BACK);
        clawAngleServo = hardwareMap.get(Servo.class, "clawAngleServo");
        clawRotationServo = hardwareMap.get(Servo.class, "clawRotationServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

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

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // Set pipeline for AprilTag detection
        limelight.start();

        switchState(State.GO_TO_APRIL_TAG);
        drive = new AngleDrive(hardwareMap);

        initPid();
    }
    public Robot setHorizontalSlideTargetPosition(int target) {
        horizontalSlideController.setTargetPosition(target);
        return this;
    }

    public Robot setVerticalSlideTargetPosition(int target) {
        verticalSlideController.setTargetPosition(target);
        return this;
    }

    public Robot increseVerticalSlideTargetPosition(int target) {
        verticalSlideController.increaseTargetPosition(target);
        return this;
    }

    public Robot increseHorizontalSlideTargetPosition(int target) {
        horizontalSlideController.increaseTargetPosition(target);
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

    public Robot setClawAnglePosition(double position) {
        clawAngleServo.setPosition(position);
        return this;
    }

    public double getClawAnglePosition() {
        return clawAngleServo.getPosition();
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
        verticalSlideController = new VerticalSlideController(hardwareMap, "verticalSlide2", "verticalSlide1", true, DriveTest.Params.VERTICAL_SLIDE_DROP_L2, 0);
    }
    public HashMap<String, Servo> getServoForTesting() {
        HashMap<String, Servo> servoHashMap = new HashMap<>();
        servoHashMap.put("clawAngleServo", clawAngleServo);
        servoHashMap.put("clawRotationServo", clawRotationServo);
        servoHashMap.put("clawServo", clawServo);
        servoHashMap.put("intakeAngleServo", intakeAngleServo);
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
        isAutoMode = true;
        this.targetX = targetX;
        this.targetY= targetY;
        this.targetHeading= targetHeading; //IN RADIANS
    }

    public void disableAutoMode() {
        isAutoMode = false;
    }


    public void execute(Telemetry telemetry) {
//        if (!isAprilTagDetected) {
//            pollForAprilTag(telemetry);
//        } else {
//            switchState(State.GO_TO_APRIL_TAG);
//        }
        if (isAutoMode) {
            Pose3D robotPos = pollForAprilTag(telemetry);
            double xPower = 0;
            double yPower = 0;
            if (robotPos != null) {
                xPower = xPid.calculate(targetX, robotPos.getPosition().x);
                yPower = yPid.calculate(targetY, robotPos.getPosition().y);

                if (xPower*xPower+yPower*yPower > 1) {
                    double mag = Math.sqrt(xPower*xPower+yPower*yPower);
                    xPower = xPower/mag;
                    yPower = yPower/mag;
                }
            }

            double rightStickX = Math.cos(targetHeading);
            double rightStickY = -Math.sin(targetHeading);

            telemetry.addData("X Power", xPower);
            telemetry.addData("Y Power", yPower);
            telemetry.addData("Right Stick X", rightStickX);
            telemetry.addData("Right Stick Y", rightStickY);
            telemetry.addData("Current Pos", robotPos);
            telemetry.addData("Target X", targetX);
            telemetry.addData("Target Y", targetY);

//            drive.updateRaw(telemetry, false, xPower, yPower, rightStickX, rightStickY, 1, 1);
        } else {
            drive.update(telemetry, joystick, 1, 1);
        }
        currentState.execute(this, telemetry);
        horizontalSlideController.update(telemetry);
        verticalSlideController.update(telemetry);
        clawSlideController.update(telemetry);
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

        telemetry.update();
        return botPose;


    }

    public void switchState(State newState) {
        IRobot prevState = currentState;
        currentState = Objects.requireNonNull(instanceStateMap.get(newState)).get();
        currentState.initialize(this, prevState);
    }
}
