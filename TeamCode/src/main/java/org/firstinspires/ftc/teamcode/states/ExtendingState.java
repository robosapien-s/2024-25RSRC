package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.wrappers.MotorController;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class ExtendingState implements IRobot {

    private boolean motorExtended = false;
    private final JoystickWrapper joystick;
    private final MotorController motorController;

    public ExtendingState(JoystickWrapper joystick, MotorController motorController) {
        this.joystick = joystick;
        this.motorController = motorController;
    }

    @Override
    public void execute() {
        if (!motorExtended) {
            motorController.reqEmChange(1.0);
            motorExtended = true;
        }

        if (joystick.gamepad1GetB()) {
            motorController.reqEmChange(0);
            Robot.getInstance().switchState(State.INITIAL);
            motorExtended = false;
        }
    }
}
