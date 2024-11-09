package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class InitialState implements IRobot {

    private final JoystickWrapper joystick;

    public InitialState(JoystickWrapper joystick) {
        this.joystick = joystick;
    }

    @Override
    public void execute(Robot robot) {
        if (joystick.gamepad1GetB()) {
            robot.setHorizontalSlideTargetPosition(540);
        } else if (joystick.gamepad1GetX()) {
            robot.setHorizontalSlideTargetPosition(0);
        } else if(joystick.gamepad1GetY()) {
            robot.setVerticalSlideTargetPosition(3000);
        } else if(joystick.gamepad1GetA()) {
            robot.setVerticalSlideTargetPosition(10);
        } else if(joystick.gamepad1GetDRight()) {
            robot.setClawSlideTargetPosition(-21110);
        } else if(joystick.gamepad1GetDLeft()) {
            robot.setClawSlideTargetPosition(0);
        } else if(joystick.gamepad1GetDUp()) {
            robot.setClawAnglePosition(0);
        } else if(joystick.gamepad1GetDDown()) {
            robot.setClawAnglePosition(1);
        }
    }

    @Override
    public State getState() {
        return State.INITIAL;
    }
}
