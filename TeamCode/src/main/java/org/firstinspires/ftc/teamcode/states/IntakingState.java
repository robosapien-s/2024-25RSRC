package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class IntakingState implements IRobot {

    @Override
    public void execute() {
        if (true) {
            Robot.getInstance().switchState(State.EXTENDING);
        }
    }
}
