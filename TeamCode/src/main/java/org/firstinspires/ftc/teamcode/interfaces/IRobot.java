package org.firstinspires.ftc.teamcode.interfaces;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;

public interface IRobot {
    public enum State {
        INVALID,
        DROPPING_L1,
        DROPPING_L2,
        SPECIMEN_HANG,
        EXTENDING,
        INITIAL,
        INTAKING,
        MECANUM_DRIVE,
        WALLPICKUP,
        FIELD_CENTRIC_DRIVE,
        SERVO_TEST,
        PID_TUNING
    }


    void initialize(Robot robot, IRobot prevState);

    void execute(Robot robot, Telemetry telemetry);

    IRobot.State getState();
}
