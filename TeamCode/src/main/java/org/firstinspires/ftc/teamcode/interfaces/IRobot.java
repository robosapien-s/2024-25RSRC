package org.firstinspires.ftc.teamcode.interfaces;

import org.firstinspires.ftc.teamcode.robot.Robot;

public interface IRobot {
    public enum State {
        INVALID,
        DROPPING,
        EXTENDING,
        INITIAL,
        INTAKING,
        MECANUM_DRIVE,
        FIELD_CENTRIC_DRIVE
    }
    void execute(Robot robot);

    IRobot.State getState();
}
