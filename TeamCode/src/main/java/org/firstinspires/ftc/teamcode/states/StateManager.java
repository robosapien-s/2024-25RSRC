package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.teamcode.IRobot;
import java.util.HashMap;
import java.util.Map;

public class StateManager implements IRobot {

    public State state = State.INITIAL;

    IntakingState intakingState = new IntakingState();
    InitialState initialState = new InitialState();
    ExtendingState extendingState = new ExtendingState();
    DroppingState droppingState = new DroppingState();

    private final Map<State, IRobot> stateMap = new HashMap<>();

    public StateManager() {
        stateMap.put(State.INTAKING, intakingState);
        stateMap.put(State.INITIAL, initialState);
        stateMap.put(State.EXTENDING, extendingState);
        stateMap.put(State.DROPPING, droppingState);
    }

    @Override
    public void execute() {
        IRobot currentStateRobot = stateMap.get(state);
        if (currentStateRobot != null) {
            currentStateRobot.execute();
        }
    }

    public void switchState(State newState) {
        state = newState;
    }

    public State returnState() {
        return state;
    }
}
