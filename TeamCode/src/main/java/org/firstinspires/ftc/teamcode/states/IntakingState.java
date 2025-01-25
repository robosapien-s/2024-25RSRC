package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.opmodes.DriveTest;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class IntakingState extends BaseState {

    public IntakingState(JoystickWrapper joystick) {
        super(joystick);
    }

    @Override
    public void initialize(Robot robot, IRobot prevState) {
        if(prevState == null) {
            robot.setClawPosition(DriveTest.Params.CLAW_OPEN);
            robot.setClawRotationPosition(DriveTest.Params.ROT_SERVO_DEFAULT);
            robot.setClawAnglePosition(DriveTest.Params.CLAW_ANGLE_DOWN);
            robot.setClawSlideTargetPosition(DriveTest.Params.CLAW_SLIDER_DOWN);
            robot.setVerticalSlideTargetPosition(DriveTest.Params.VERTICAL_SLIDE_POSITION);
            robot.setHorizontalSlideTargetPosition(0);
            robot.setIntakeAngleServo(DriveTest.Params.INTAKE_ANGLE_TRANSFER);
        } else {
            RobotTaskSeries transferSeries = new RobotTaskSeries();
            transferSeries.add(createClawTask(robot, DriveTest.Params.CLAW_OPEN, 1, "ClawClose", false));
            transferSeries.add(createHorizontalSlideTask(robot, 0, 1, "HorizontalSlide", false));
            transferSeries.add(createClawSlideTask(robot, DriveTest.Params.CLAW_SLIDER_DOWN, 500, "CLAW_SLIDER_BACK", false));
            transferSeries.add(createClawAngleTask(robot, DriveTest.Params.CLAW_ANGLE_DOWN, 1, "CLAW_ANGLE_BACK", false));
            transferSeries.add(createClawRotationTask(robot, DriveTest.Params.ROT_SERVO_DEFAULT, 1, "ROT_SERVO_BACK", false));
            transferSeries.add(createVerticalSlideTask(robot, DriveTest.Params.VERTICAL_SLIDE_POSITION, 500, "VerticalSlide", false));
            taskArrayList.add(transferSeries);
        }
    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {

        if(joystick.gamepad1GetA()) {
            robot.switchState((State.WALLPICKUP));
        } else if(joystick.gamepad1GetB()) {
            robot.switchState(State.DROPPING_L1);
        }

        if(joystick.gamepad1GetRightBumperRaw()) {
            robot.increseHorizontalSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger()*-400));
        } else {
            robot.increseHorizontalSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger()*400));
        }


        if(joystick.gamepad1GetLeftBumperRaw()) {
            robot.setIntakePower(joystick.gamepad1GetLeftTrigger());
        } else {
            robot.setIntakePower(-joystick.gamepad1GetLeftTrigger());
        }

        executeTasks(telemetry);

    }

    @Override
    public State getState() {
        return State.INTAKING;
    }
}
