package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Color;

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

        switchState(State.INTAKINGCLAW);

        drive = new AngleDrive(hardwareMap);
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

    public State getCurrentState() {
        if (currentState != null) {
            return currentState.getState();
        } else {
            return State.INVALID;
        }
    }

    public void switchState(State newState) {
        IRobot prevState = currentState;
        currentState = Objects.requireNonNull(instanceStateMap.get(newState)).get();
        currentState.initialize(this, prevState);
    }

    public void execute(Telemetry telemetry) {
        currentState.execute(this, telemetry);
        drive.update(telemetry, joystick, 1, 1);
        horizontalSlideController.update(telemetry);
        verticalSlideController.update(telemetry);
        clawSlideController.update(telemetry);
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

    public String sensorColor() {
        // Create a color sensor reference
        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        // Variables for gain adjustment and HSV color values
        float gain = 2.0f;
        final float[] hsvValues = new float[3];

        // Enable the sensor's light if it supports SwitchableLight
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        // Adjust sensor gain (if needed)
        colorSensor.setGain(gain);

        // Retrieve the normalized colors from the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        // Convert the colors to HSV
        Color.colorToHSV(colors.toColor(), hsvValues);

        // Analyze the hue to determine the color
        float hue = hsvValues[0];
        String detectedColor;

        if ((hue >= 0 && hue <= 30) || (hue >= 330 && hue <= 360)) {
            detectedColor = "Red"; // Hue range for red
        } else if (hue >= 210 && hue <= 270) {
            detectedColor = "Blue"; // Hue range for blue
        } else if (hue >= 45 && hue <= 75) {
            detectedColor = "Yellow"; // Hue range for yellow
        } else {
            detectedColor = "None"; // No specified color detected
        }

        return detectedColor;
    }


}
