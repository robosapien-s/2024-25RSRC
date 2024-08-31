package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.states.DroppingState;
import org.firstinspires.ftc.teamcode.states.ExtendingState;
import org.firstinspires.ftc.teamcode.states.InitialState;
import org.firstinspires.ftc.teamcode.states.IntakingState;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

import java.util.HashMap;
import java.util.Map;

public class Robot implements IRobot {
    public State state = State.INITIAL;
    IntakingState intakingState = new IntakingState();
    InitialState initialState = new InitialState();
    ExtendingState extendingState = new ExtendingState();
    DroppingState droppingState = new DroppingState();

    private final Map<State, IRobot> stateMap = new HashMap<>();
    public Robot(){
        stateMap.put(State.INTAKING, intakingState);
        stateMap.put(State.INITIAL, initialState);
        stateMap.put(State.EXTENDING, extendingState);
        stateMap.put(State.DROPPING, droppingState);
    }

    public State getState(){
        return state;

    }

    @Override
    public void execute(JoystickWrapper joystick) {
        IRobot currentStateRobot = stateMap.get(state);
        if (currentStateRobot != null) {
            currentStateRobot.execute(joystick);
        }
        currentStateRobot.execute(joystick);
    }
}
// change
