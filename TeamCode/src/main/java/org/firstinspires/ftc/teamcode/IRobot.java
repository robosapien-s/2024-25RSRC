package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public interface IRobot {
    public enum State {
        DROPPING,
        EXTENDING,
        INITIAL,
        INTAKING
    }
    public final DcMotorEx leftFront = hardwareMap.get(DcMotorEx .class, "fL");
    public final DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "bL");
    public final DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "bR");
    public final DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "fR");
    void execute(JoystickWrapper joystick);

}//
// initialState, intakingState, extendingState, droppingState