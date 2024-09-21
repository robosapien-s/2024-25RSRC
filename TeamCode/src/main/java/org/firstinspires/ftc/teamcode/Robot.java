package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.states.DroppingState;
import org.firstinspires.ftc.teamcode.states.ExtendingState;
import org.firstinspires.ftc.teamcode.states.InitialState;
import org.firstinspires.ftc.teamcode.states.IntakingState;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

import java.util.HashMap;
import java.util.Map;

public class Robot implements IRobot {

    private static Robot instance;
    public State state = State.INITIAL;
    private JoystickWrapper joystick;
    IntakingState intakingState = new IntakingState();
    InitialState initialState = new InitialState(joystick);
    ExtendingState extendingState = new ExtendingState(joystick);
    DroppingState droppingState = new DroppingState();

    private final Map<State, IRobot> stateMap = new HashMap<>();

    public Robot(){
        instance = this;
        stateMap.put(State.INTAKING, intakingState);
        stateMap.put(State.INITIAL, initialState);
        stateMap.put(State.EXTENDING, extendingState);
        stateMap.put(State.DROPPING, droppingState);
    }

    public static Robot getInstance() {
        return instance;
    }

    public State getCurrentState(){
        return state;
    }

    public void switchState(State newState){
        state = newState;
    }

    @Override
    public void execute() {
        IRobot currentStateRobot = stateMap.get(state);
        assert currentStateRobot != null;
        currentStateRobot.execute();
    }
}
