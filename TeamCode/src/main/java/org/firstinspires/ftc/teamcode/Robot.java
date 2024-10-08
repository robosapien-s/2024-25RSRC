package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot.State;
import org.firstinspires.ftc.teamcode.states.*;
import org.firstinspires.ftc.teamcode.wrappers.MotorController;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

import java.util.HashMap;
import java.util.Map;

public class Robot implements IRobot {

    private static Robot instance;
    private State state = State.INITIAL;
    private final JoystickWrapper joystick;
    private final MotorController motorController;

    private final Map<State, IRobot> stateMap = new HashMap<>();

    public Robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        instance = this;

        joystick = new JoystickWrapper(gamepad1, gamepad2);

        motorController = new MotorController(hardwareMap);

        IntakingState intakingState = new IntakingState();
        InitialState initialState = new InitialState(joystick);
        ExtendingState extendingState = new ExtendingState(joystick, motorController);
        DroppingState droppingState = new DroppingState();
        MecanumDriveState mecanumDriveState = new MecanumDriveState(joystick, motorController);
        FieldCentricDriveState fieldCentricDriveState = new FieldCentricDriveState(joystick, motorController);

        stateMap.put(State.INTAKING, intakingState);
        stateMap.put(State.INITIAL, initialState);
        stateMap.put(State.EXTENDING, extendingState);
        stateMap.put(State.DROPPING, droppingState);
        stateMap.put(State.MECANUM_DRIVE, mecanumDriveState);
        stateMap.put(State.FIELD_CENTRIC_DRIVE, fieldCentricDriveState);
    }

    public static Robot getInstance() {
        return instance;
    }

    public State getCurrentState() {
        return state;
    }

    public void switchState(State newState) {
        state = newState;
    }

    @Override
    public void execute() {
        IRobot currentStateRobot = stateMap.get(state);
        if (currentStateRobot != null) {
            currentStateRobot.execute();
        }
    }
}
