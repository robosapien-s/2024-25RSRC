package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;

public class IntakingState implements IRobot {

    @Override
    public void initialize(Robot robot) {
        robot.setHorizontalSlideTargetPosition(Robot.HORIZONTAL_SLIDE_INTAKE_INITIAL);
        robot.setIntakePower(1);
    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {


    }

    @Override
    public State getState() {
        return State.INTAKING;
    }
}
