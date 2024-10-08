package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;
import org.firstinspires.ftc.teamcode.wrappers.MotorController;

public class MecanumDriveState implements IRobot {

    private final JoystickWrapper joystick;
    private final MotorController motorController;

    public MecanumDriveState(JoystickWrapper joystick, MotorController motorController) {
        this.joystick = joystick;
        this.motorController = motorController;
    }

    @Override
    public void execute() {
        double y = -joystick.gamepad1GetLeftStickY();
        double x = joystick.gamepad1GetLeftStickX();
        double rx = joystick.gamepad1GetRightStickX();

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double flPower = (y + x + rx) / denominator;
        double blPower = (y - x + rx) / denominator;
        double brPower = (y + x - rx) / denominator;
        double frPower = (y - x - rx) / denominator;

        motorController.reqFlChange(flPower);
        motorController.reqBlChange(blPower);
        motorController.reqBrChange(brPower);
        motorController.reqRfChange(frPower);

        if (joystick.gamepad1GetB()) {
            Robot.getInstance().switchState(State.FIELD_CENTRIC_DRIVE);
        } else if (joystick.gamepad1GetY()) {
            Robot.getInstance().switchState(State.INITIAL);
        }
    }
}
