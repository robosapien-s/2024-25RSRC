package org.firstinspires.ftc.teamcode.interfaces;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public interface IRobot {
    public enum State {
        DROPPING,
        EXTENDING,
        INITIAL,
        INTAKING
    }
    void execute();

}