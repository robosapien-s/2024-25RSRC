package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskParallel;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
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
            taskArrayList.add(createVerticalSlideTask(robot, RoboSapiensTeleOp.Params.VERTICAL_SLIDE_HANG_PREP_POSITION, 100, "Vertical", false));
            taskArrayList.add(createClawAngleTask( robot, RoboSapiensTeleOp.Params.CLAW_ANGLE_FORWARD, 200, "ClawAngle", false));
            taskArrayList.add(createClawSlideTask( robot, RoboSapiensTeleOp.Params.CLAW_SLIDER_FORWARD, 200, "ClawSlide", false));

            RobotTaskParallel transferParallel = new RobotTaskParallel();

            //transferParallel.add(createClawAngleTask( robot, DriveTest.Params.CLAW_ANGLE_FORWARD_SPECIMEN, 1000, "ClawAngle", true));
            transferParallel.add(createClawRotationTask( robot, RoboSapiensTeleOp.Params.ROT_SERVO_DEFAULT, 1, "ClawRotation", false));
            transferParallel.add(createHorizontalSlideTask(robot, 0, 1, "Horizontal", true));

            taskArrayList.add(transferParallel);
        } else {
            taskArrayList.add(createClawTask(robot, RoboSapiensTeleOp.Params.CLAW_CLOSE, 250, "Claw", false));
            taskArrayList.add(createVerticalSlideTask(robot, RoboSapiensTeleOp.Params.VERTICAL_SLIDE_HANG_PREP_POSITION, 300, "Vertical", false));
            taskArrayList.add(createClawHorizontalAngleTask(robot, RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER,1,"ClawHorizontalAngle",false));
            taskArrayList.add(createClawRotationTask( robot, RoboSapiensTeleOp.Params.ROT_SERVO_DEFAULT, 1, "ClawRotation", false));
            taskArrayList.add(createClawAngleTask( robot, RoboSapiensTeleOp.Params.CLAW_ANGLE_FORWARD, 200, "ClawAngle", false));
            taskArrayList.add(createClawSlideTask( robot, RoboSapiensTeleOp.Params.CLAW_SLIDER_FORWARD, 0, "ClawSlide", false));
            taskArrayList.add(createHorizontalSlideTask(robot, 0, 0, "Horizontal", true));
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
                robot.setClawPosition(RoboSapiensTeleOp.Params.CLAW_OPEN);
            }
            else if(subState == 1) {

                subState++;
                //robot.setVerticalSlideTargetPosition(DriveTest.Params.VERTICAL_SLIDE_HANG_DROP_POSITION);
                robot.setVerticalSlideTargetPosition(RoboSapiensTeleOp.Params.VERTICAL_SLIDE_HANG_DROP_POSITION);
                robot.setClawAnglePosition(RoboSapiensTeleOp.Params.CLAW_ANGLE_FORWARD);
                robot.setClawHorizontalAnglePosition(RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER);
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
            robot.setClawHorizontalAnglePosition(RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_LEFT);
        } else if (joystick.gamepad1GetDRight() && subState == 0) {
            robot.setClawHorizontalAnglePosition(RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_RIGHT);
        } else if (joystick.gamepad1GetDDown() && subState == 0) {
            robot.setClawHorizontalAnglePosition(RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER);
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
