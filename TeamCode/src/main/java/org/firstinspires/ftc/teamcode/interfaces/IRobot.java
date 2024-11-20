package org.firstinspires.ftc.teamcode.interfaces;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;

public interface IRobot {
    public enum State {
        INVALID,
        DROPPING,
        EXTENDING,
        INITIAL,
        INTAKING,
        MECANUM_DRIVE,
        WALLPICKUP,
        FIELD_CENTRIC_DRIVE
    }


    void initialize(Robot robot);

    void execute(Robot robot, Telemetry telemetry);

    IRobot.State getState();
}
