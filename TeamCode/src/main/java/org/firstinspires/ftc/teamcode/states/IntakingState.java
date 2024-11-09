package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;

public class IntakingState implements IRobot {

    @Override
    public void execute(Robot robot) {
        if (true) {
            robot.switchState(State.EXTENDING);
        }
    }

    @Override
    public State getState() {
        return State.INTAKING;
    }
}
