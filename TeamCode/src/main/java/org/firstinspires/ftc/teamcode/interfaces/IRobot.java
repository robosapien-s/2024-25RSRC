package org.firstinspires.ftc.teamcode.interfaces;

public interface IRobot {
    public enum State {
        DROPPING,
        EXTENDING,
        INITIAL,
        INTAKING,
        MECANUM_DRIVE,
        FIELD_CENTRIC_DRIVE
    }
    void execute();
}
