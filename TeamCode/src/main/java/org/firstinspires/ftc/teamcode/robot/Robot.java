package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Color;

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

    public Robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
        joystick = new JoystickWrapper(gamepad1, gamepad2);
        this.hardwareMap = hardwareMap;

        horizontalSlideController = new HorizontalSlideController(hardwareMap, "horizontalSlide1", DriveTest.Params.HORIZONTAL_SLIDE_MAX_POSITION, 0);
        verticalSlideController = new VerticalSlideController(hardwareMap, "verticalSlide2", "verticalSlide1", true, DriveTest.Params.VERTICAL_SLIDE_DROP_L2, 0);
        clawSlideController = new ClawSlideController(hardwareMap, "clawSliderCR", "verticalSlide1", DriveTest.Params.CLAW_SLIDER_FORWARD, DriveTest.Params.CLAW_SLIDER_BACK);
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
        limelight.pipelineSwitch(1);
        limelight.start();

        switchState(State.INITIAL);
        drive = new AngleDrive(hardwareMap);
    }

    public void execute(Telemetry telemetry) {
        if (!isAprilTagDetected) {
            pollForAprilTag(telemetry);
        } else {
            switchState(State.GO_TO_APRIL_TAG);
        }

        currentState.execute(this, telemetry);
        drive.update(telemetry, joystick, 1, 1);
        horizontalSlideController.update(telemetry);
        verticalSlideController.update(telemetry);
        clawSlideController.update(telemetry);
    }

    private void pollForAprilTag(Telemetry telemetry) {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("AprilTag Found", "ID: %d, Family: %s", fr.getFiducialId(), fr.getFamily());
                telemetry.addData("Target Position", "X: %.2f, Y: %.2f", fr.getTargetXDegrees(), fr.getTargetYDegrees());
                isAprilTagDetected = true;
                break;
            }
        } else {
            telemetry.addData("AprilTag", "No tags detected");
        }
        telemetry.update();
    }

    public void switchState(State newState) {
        IRobot prevState = currentState;
        currentState = Objects.requireNonNull(instanceStateMap.get(newState)).get();
        currentState.initialize(this, prevState);
    }
}
