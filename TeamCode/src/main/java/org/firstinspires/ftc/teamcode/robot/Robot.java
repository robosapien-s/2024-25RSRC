package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

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

    //private static Robot instance;
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

    private final HardwareMap hardwareMap = null;


    private final Map<State, Supplier<IRobot>> instanceStateMap = new HashMap<>();
//    private IRobot drive;

    public Robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        joystick = new JoystickWrapper(gamepad1, gamepad2);
        hardwareMap = hardwareMap;

        horizontalSlideController = new HorizontalSlideController(hardwareMap, "horizontalSlide1", DriveTest.Params.HORIZONTAL_SLIDE_MAX_POSITION, 0);
        verticalSlideController = new VerticalSlideController(hardwareMap, "verticalSlide2", "verticalSlide1", true, DriveTest.Params.VERTICAL_SLIDE_DROP_L2, 0);
        clawSlideController = new ClawSlideController(hardwareMap,  "clawSliderCR", "verticalSlide1", DriveTest.Params.CLAW_SLIDER_FORWARD, DriveTest.Params.CLAW_SLIDER_BACK);
        clawAngleServo = hardwareMap.get(Servo.class, "clawAngleServo");
        clawRotationServo = hardwareMap.get(Servo.class, "clawRotationServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        intakeAngleServo = hardwareMap.get(Servo.class, "intakeAngleServo");

        instanceStateMap.put(State.INITIAL, () -> new InitialState(joystick));
        instanceStateMap.put(State.INTAKING, () -> new IntakingState(joystick));
        //instanceStateMap.put(State.EXTENDING, () -> new ExtendingState(joystick, motorController));
        instanceStateMap.put(State.DROPPING_L1, () -> new DroppingL1State(joystick));
        instanceStateMap.put(State.DROPPING_L2, () -> new DroppingL2State(joystick));
        instanceStateMap.put(State.WALLPICKUP, () -> new WallPickUpState(joystick));
        instanceStateMap.put(State.SPECIMEN_HANG, () -> new SpecimenHangState(joystick));
        instanceStateMap.put(State.SERVO_TEST, () -> new ServoTestState(joystick));
        instanceStateMap.put(State.PID_TUNING, () -> new PidTuningState(joystick));

        //drive = new FieldCentricDriveState(joystick, motorController);
        switchState(State.INITIAL);


        drive = new MecanumDrive(hardwareMap);
        //drive = new FDrive(hardwareMap);
        //drive.init();
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
        if(currentState != null) {
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

    public Robot setIntakePower(double power) {
        intakeServo.setPower(power);
        return this;
    }

    public Robot setIntakeAngleServoPosition(double position) {
        intakeAngleServo.setPosition(position);
        return this;
    }

    public int getVerticalSlidePosition(){
        return verticalSlideController.getCurrentPosition();
    }
    public int getHorizontalSlidePosition(){
        return horizontalSlideController.getCurrentPosition();
    }
    public int getClawSlidePosition(){
        return clawSlideController.getCurrentPosition();
    }

}
