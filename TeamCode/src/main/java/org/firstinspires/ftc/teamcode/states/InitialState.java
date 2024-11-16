package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class InitialState implements IRobot {

    private final JoystickWrapper joystick;
    private static final double CLAW_SERVO_UP = 0.3;
    private static final double CLAW_SERVO_DOWN = 0.7;
    private static final double ROT_SERVO = 0.5;


    public InitialState(JoystickWrapper joystick) {
        this.joystick = joystick;
    }

    @Override
    public void execute(Robot robot) {
        if (joystick.gamepad1GetB()) {
            robot.setClawPosition(CLAW_SERVO_DOWN);
        } else if (joystick.gamepad1GetX()) {
            robot.setClawRotationPosition(ROT_SERVO);
        } else if(joystick.gamepad1GetY()) {
            robot.setVerticalSlideTargetPosition(3000);
        } else if(joystick.gamepad1GetA()) {
            robot.setClawPosition(CLAW_SERVO_UP);
        } else if(joystick.gamepad1GetDRight()) {
            robot.setClawSlideTargetPosition(-21110);
        } else if(joystick.gamepad1GetDLeft()) {
            robot.setClawSlideTargetPosition(0);
        } else if(joystick.gamepad1GetDUp()) {
            robot.setClawAnglePosition(0);
        } else if(joystick.gamepad1GetDDown()) {
            robot.setClawAnglePosition(1);
        }

        robot.setIntakePower(joystick.gamepad1GetRightTrigger()-joystick.gamepad1GetLeftTrigger());
    }

    @Override
    public State getState() {
        return State.INITIAL;
    }
}
