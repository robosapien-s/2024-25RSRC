package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class InitialState implements IRobot {

    private final JoystickWrapper joystick;

    public InitialState(JoystickWrapper joystick) {
        this.joystick = joystick;
    }

    @Override
    public void execute() {
        if (joystick.gamepad1GetA()) {
            Robot.getInstance().switchState(State.MECANUM_DRIVE);
        } else if (joystick.gamepad1GetB()) {
            Robot.getInstance().switchState(State.FIELD_CENTRIC_DRIVE);
        }
    }
}
