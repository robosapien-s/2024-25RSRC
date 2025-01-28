package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskParallel;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.DriveTest;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class SpecimenHangState extends BaseState {

    boolean didLowerHeight = false;
    int subState = 0;
    public SpecimenHangState(JoystickWrapper joystick) {
        super(joystick);
    }

    @Override
    public void initialize(Robot robot, IRobot prevState) {

        if (prevState.getState() == State.INTAKINGCLAW) {
            taskArrayList.add(createVerticalSlideTask(robot, robot.getVerticalSlidePosition() + 80, 100, "Vertical", false));
            taskArrayList.add(createClawAngleTask( robot, DriveTest.Params.CLAW_ANGLE_FORWARD_SPECIMEN, 200, "ClawAngle", false));
            taskArrayList.add(createClawSlideTask( robot, DriveTest.Params.CLAW_SLIDER_FORWARD, 200, "ClawSlide", false));

            RobotTaskParallel transferParallel = new RobotTaskParallel();

            //transferParallel.add(createClawAngleTask( robot, DriveTest.Params.CLAW_ANGLE_FORWARD_SPECIMEN, 1000, "ClawAngle", true));
            transferParallel.add(createClawRotationTask( robot, DriveTest.Params.ROT_SERVO_DEFAULT, 1, "ClawRotation", false));
            transferParallel.add(createHorizontalSlideTask(robot, 0, 1, "Horizontal", true));
            transferParallel.add(createVerticalSlideTask(robot, DriveTest.Params.VERTICAL_SLIDE_HANG_PREP_POSITION, 0, "Vertical", false));

            taskArrayList.add(transferParallel);
        } else {
            taskArrayList.add(createClawTask(robot, DriveTest.Params.CLAW_CLOSE, 500, "Claw", false));
            taskArrayList.add(createVerticalSlideTask(robot, robot.getVerticalSlidePosition() + 80, 500, "Vertical", false));
            taskArrayList.add(createClawHorizontalAngleTask(robot,DriveTest.Params.CLAW_HORIZONTAL_ANGLE_CENTER,0,"ClawHorizontalAngle",false));
            taskArrayList.add(createClawAngleTask( robot, DriveTest.Params.CLAW_ANGLE_FORWARD_SPECIMEN, 200, "ClawAngle", false));
            taskArrayList.add(createClawSlideTask( robot, DriveTest.Params.CLAW_SLIDER_FORWARD, 500, "ClawSlide", false));

            RobotTaskParallel transferParallel = new RobotTaskParallel();

            //transferParallel.add(createClawAngleTask( robot, DriveTest.Params.CLAW_ANGLE_FORWARD_SPECIMEN, 1000, "ClawAngle", true));
            transferParallel.add(createClawRotationTask( robot, DriveTest.Params.ROT_SERVO_DEFAULT, 1000, "ClawRotation", false));
            transferParallel.add(createHorizontalSlideTask(robot, 0, 1000, "Horizontal", true));
            transferParallel.add(createVerticalSlideTask(robot, DriveTest.Params.VERTICAL_SLIDE_HANG_PREP_POSITION, 1000, "Vertical", false));

            taskArrayList.add(transferParallel);
        }
    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {

        if(joystick.gamepad1GetA()) {
            //robot.setVerticalSlideTargetPosition(DriveTest.Params.VERTICAL_SLIDE_HANG_DROP_POSITION);
            robot.switchState(State.INTAKINGCLAW);
        } else if(joystick.gamepad1GetB()) {
            if(subState == 0) {
                subState++;
                robot.setClawPosition(DriveTest.Params.CLAW_OPEN);
            }
            else if(subState == 1) {

                subState++;
                //robot.setVerticalSlideTargetPosition(DriveTest.Params.VERTICAL_SLIDE_HANG_DROP_POSITION);
                robot.setVerticalSlideTargetPosition(DriveTest.Params.VERTICAL_SLIDE_HANG_DROP_POSITION);
                robot.setClawAnglePosition(DriveTest.Params.CLAW_ANGLE_FORWARD);
                robot.setClawHorizontalAnglePosition(DriveTest.Params.CLAW_HORIZONTAL_ANGLE_CENTER);
            } else {
                robot.switchState(State.WALLPICKUP);
            }
        } else if(joystick.gamepad1GetY()) {
            robot.switchState(State.DROPPING_L1);
//        } else if(joystick.gamepad1GetDUp()) {
//            robot.setVerticalSlideTargetPosition(DriveTest.Params.VERTICAL_SLIDE_HANG_PREP_POSITION);
//        } else if(joystick.gamepad1GetDDown()) {
//            robot.setVerticalSlideTargetPosition(DriveTest.Params.VERTICAL_SLIDE_HANG_DROP_POSITION);
//        }
        } else if (joystick.gamepad1GetDLeft() && subState == 0) {
            robot.setClawHorizontalAnglePosition(DriveTest.Params.CLAW_HORIZONTAL_ANGLE_LEFT);
        } else if (joystick.gamepad1GetDRight() && subState == 0) {
            robot.setClawHorizontalAnglePosition(DriveTest.Params.CLAW_HORIZONTAL_ANGLE_RIGHT);
        } else if (joystick.gamepad1GetDDown() && subState == 0) {
            robot.setClawHorizontalAnglePosition(DriveTest.Params.CLAW_HORIZONTAL_ANGLE_CENTER);
        }

        if(joystick.gamepad1GetLeftBumperRaw()) {
            robot.increaseClawSlideTargetPosition((int) (joystick.gamepad1GetLeftTrigger()*-500));
        } else {
            robot.increaseClawSlideTargetPosition((int) (joystick.gamepad1GetLeftTrigger()*500));
        }

        if(joystick.gamepad1GetRightBumperRaw()) {
            robot.increseVerticalSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger()*-100));
        } else {
            robot.increseVerticalSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger()*100));
        }

        executeTasks(telemetry);
    }

    @Override
    public State getState() {
        return State.SPECIMEN_HANG;
    }
}
