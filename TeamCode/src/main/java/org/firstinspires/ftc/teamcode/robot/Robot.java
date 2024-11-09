package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot.State;
import org.firstinspires.ftc.teamcode.states.*;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

public class Robot {

    //private static Robot instance;
    private IRobot currentState;
    private final JoystickWrapper joystick;
    private final HorizontalSlideController horizontalSlideController;
    private final VerticalSlideController verticalSlideController;
    private final ClawSlideController clawSlideController;

    private final Servo clawAngleServo;
    private final Servo clawRotationServo;
    private final Servo clawServo;

    private final Map<State, Supplier<IRobot>> instanceStateMap = new HashMap<>();
    private IRobot drive;

    public Robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        joystick = new JoystickWrapper(gamepad1, gamepad2);

        horizontalSlideController = new HorizontalSlideController(hardwareMap, "horizontalSlide1");
        verticalSlideController = new VerticalSlideController(hardwareMap, "verticalSlide2", "verticalSlide1", true);
        clawSlideController = new ClawSlideController(hardwareMap,  "clawSliderCR", "verticalSlide1");
        clawAngleServo = hardwareMap.get(Servo.class, "clawAngleServo");
        clawRotationServo = hardwareMap.get(Servo.class, "clawRotationServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        instanceStateMap.put(State.INITIAL, () -> new InitialState(joystick));
        instanceStateMap.put(State.INTAKING, IntakingState::new);
        //instanceStateMap.put(State.EXTENDING, () -> new ExtendingState(joystick, motorController));
        instanceStateMap.put(State.DROPPING, DroppingState::new);
        //drive = new FieldCentricDriveState(joystick, motorController);
        switchState(State.INITIAL);
    }

    public State getCurrentState() {
        if(currentState != null) {
            return currentState.getState();
        } else {
            return State.INVALID;
        }
    }

    public void switchState(State newState) {
        currentState = Objects.requireNonNull(instanceStateMap.get(newState)).get();
    }

    public void execute(Telemetry telemetry) {
        currentState.execute(this);
        horizontalSlideController.update(telemetry);
        verticalSlideController.update(telemetry);
        clawSlideController.update(telemetry);
    }

    public void setHorizontalSlideTargetPosition(int target) {
        horizontalSlideController.setTargetPosition(target);
    }

    public void setVerticalSlideTargetPosition(int target) {
        verticalSlideController.setTargetPosition(target);
    }

    public void setClawSlideTargetPosition(int target) {
        clawSlideController.setTargetPosition(target);
    }

    public void setClawAnglePosition(double position) {
        clawAngleServo.setPosition(position);
    }

    public void setClawRotationPosition(double position) {
        clawRotationServo.setPosition(position);
    }

    public void setClawPosition(double position) {
        clawServo.setPosition(position);
    }
}
