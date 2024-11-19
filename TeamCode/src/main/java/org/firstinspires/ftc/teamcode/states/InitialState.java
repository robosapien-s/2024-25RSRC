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
    public void initialize(Robot robot){
        robot.setClawPosition(Robot.CLAW_OPEN);
        robot.setClawRotationPosition(Robot.ROT_SERVO_DEFAULT);
        robot.setClawAnglePosition(Robot.CLAW_ANGLE_DOWN);
        robot.setClawSlideTargetPosition(Robot.CLAW_SLIDER_DOWN);
        robot.setVerticalSlideTargetPosition(120);
        robot.setHorizontalSlideTargetPosition(0);
        robot.setIntakeAngleServoPosition(Robot.INTAKE_ANGLE_TRANSFER);
    }


    @Override
    public void execute(Robot robot) {


        if(joystick.gamepad1GetA()) {
            robot.switchState(State.INTAKING);
        } else if(joystick.gamepad1GetX()) {
            robot.setIntakeAngleServoPosition(.48);
        } else if(joystick.gamepad1GetB()) {
            robot.setIntakeAngleServoPosition(.51);
        } else if(joystick.gamepad1GetRightBumperDown()) {
            robot.setClawPosition(Robot.CLAW_CLOSE);
        } else if(joystick.gamepad1GetLeftBumperDown()) {
            robot.setClawPosition(Robot.CLAW_OPEN);
        } else if(joystick.gamepad1GetDUp()) {
            robot.increseVerticalSlideTargetPosition(100);
        } else if(joystick.gamepad1GetDDown()) {
            robot.setVerticalSlideTargetPosition(120);
        } else if(joystick.gamepad1GetDRight()) {
            robot.increseHorizontalSlideTargetPosition(50);
        } else if(joystick.gamepad1GetDLeft()) {
            robot.increseHorizontalSlideTargetPosition(-50);
        }
        /*
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

         */

        robot.setIntakePower(joystick.gamepad1GetRightTrigger()-joystick.gamepad1GetLeftTrigger());
    }

    @Override
    public State getState() {
        return State.INITIAL;
    }
}
