package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;

public class DroppingState implements IRobot {

    @Override
    public void initialize(Robot robot) {

    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {
        if (true) {
            robot.switchState(State.EXTENDING);
        }
    }

    @Override
    public State getState() {
        return State.DROPPING;
    }

}
