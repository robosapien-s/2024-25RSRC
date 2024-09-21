package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ExtendingState implements IRobot {

    private boolean motorExtended = false;
    private JoystickWrapper joystick;
    private DcMotor motor;

    public ExtendingState(JoystickWrapper joystick, DcMotor motor) {
        this.joystick = joystick;
        this.motor = motor;
    }

    @Override
    public void execute() {
        if (!motorExtended) {
            motor.setPower(1.0);
            motorExtended = true;
        }

        if (joystick.gamepad1GetB()) {
            motor.setPower(0);
            Robot.getInstance().switchState(State.INITIAL);
        }
    }
}
