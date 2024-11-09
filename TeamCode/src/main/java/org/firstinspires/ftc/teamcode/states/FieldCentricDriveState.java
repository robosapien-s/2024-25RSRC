package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;
import org.firstinspires.ftc.teamcode.wrappers.MotorController;

public class FieldCentricDriveState implements IRobot {

    private final JoystickWrapper joystick;
    private final MotorController motorController;
    private final double robotHeading = 0;

    public FieldCentricDriveState(JoystickWrapper joystick, MotorController motorController) {
        this.joystick = joystick;
        this.motorController = motorController;
    }

    @Override
    public void execute(Robot robot) {
        double y = -joystick.gamepad1GetLeftStickY();
        double x = joystick.gamepad1GetLeftStickX();
        double rx = joystick.gamepad1GetRightStickX();

        double botHeading = Math.toRadians(robotHeading);

        double rotatedX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotatedY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotatedY) + Math.abs(rotatedX) + Math.abs(rx), 1);
        double flPower = (rotatedY + rotatedX + rx) / denominator;
        double blPower = (rotatedY - rotatedX + rx) / denominator;
        double brPower = (rotatedY + rotatedX - rx) / denominator;
        double frPower = (rotatedY - rotatedX - rx) / denominator;

        motorController.reqFlChange(flPower);
        motorController.reqBlChange(blPower);
        motorController.reqBrChange(brPower);
        motorController.reqRfChange(frPower);

        if (joystick.gamepad1GetA()) {
            robot.switchState(State.MECANUM_DRIVE);
        } else if (joystick.gamepad1GetY()) {
            robot.switchState(State.INITIAL);
        }
    }

    @Override
    public State getState() {
        return State.FIELD_CENTRIC_DRIVE;
    }
}
