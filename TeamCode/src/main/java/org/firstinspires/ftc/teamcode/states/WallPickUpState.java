package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.ExecuteOnceTask;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class WallPickUpState extends BaseState {

    boolean angle_ready = false;
    boolean isDriftModeEnabled = false;
    public WallPickUpState(JoystickWrapper joystick) {
        super(joystick);
    }

    @Override
    public void initialize(Robot robot, IRobot prevState) {

        RobotTaskSeries transferParallel = new RobotTaskSeries();


        transferParallel.add(
                new ExecuteOnceTask(
                        new ExecuteOnceTask.ExecuteListener() {
                            @Override
                            public void execute() {
                                angle_ready = false;
                            }
                        }, "Substate Transition"
                )
        );

        taskArrayList.add(createClawHorizontalAngleTask(robot, RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER,1,"ClawHorizontalAngle",false));

        if(prevState.getState() == State.INTAKINGCLAW) {
            transferParallel.add(createClawTask(robot, RoboSapiensTeleOp.Params.CLAW_OPEN, 0, "Claw", false));
            transferParallel.add(createHorizontalSlideTask(robot, 0, 300, "Horizontal", false));
//            transferParallel.add(createVerticalSlideTask(robot, DriveTest.Params.VERTICAL_SLIDE_DOWN_POSITION, 300, "Vertical", false));
            transferParallel.add(createClawAngleTask( robot, RoboSapiensTeleOp.Params.CLAW_ANGLE_BACK, 0, "ClawAngle", false));
            transferParallel.add(createClawRotationTask(robot, RoboSapiensTeleOp.Params.ROT_SERVO_BACK,0,"ClawRotation",false));
            transferParallel.add(createClawSlideTask( robot, RoboSapiensTeleOp.Params.CLAW_SLIDER_BACK, 250, "ClawSlide", false));
//            transferParallel.add(createClawAngleTask( robot, RoboSapiensTeleOp.Params.CLAW_ANGLE_BACK, 500, "ClawAngle", false));
            transferParallel.add(
                    new ExecuteOnceTask(
                            new ExecuteOnceTask.ExecuteListener() {
                                @Override
                                public void execute() {
                                    angle_ready = true;
                                }
                            }, "Substate Transition"
                    )
            );
        transferParallel.add(createVerticalSlideTask(robot, RoboSapiensTeleOp.Params.VERTICAL_SLIDE_DOWN_POSITION, 0, "Vertical", false));
        } else if (prevState.getState() == State.SPECIMEN_HANG) {
            transferParallel.add(createClawTask(robot, RoboSapiensTeleOp.Params.CLAW_OPEN, 100, "claw open", false));//robot.setClawPosition(RoboSapiensTeleOp.Params.CLAW_OPEN);
            transferParallel.add(createVerticalSlideTask(robot,RoboSapiensTeleOp.Params.VERTICAL_SLIDE_HANG_DROP_POSITION,0, "slide halfway down", false));//robot.setVerticalSlideTargetPosition(RoboSapiensTeleOp.Params.VERTICAL_SLIDE_HANG_DROP_POSITION);
            transferParallel.add(createClawAngleTask(robot, RoboSapiensTeleOp.Params.CLAW_ANGLE_FORWARD, 0, "angle position", false));//robot.setClawAnglePosition(RoboSapiensTeleOp.Params.CLAW_ANGLE_FORWARD);
            transferParallel.add(createClawHorizontalAngleTask(robot, RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER, 200, "horizontal angle center", false));//robot.setClawHorizontalAnglePosition(RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER);
            transferParallel.add(createClawRotationTask(robot, RoboSapiensTeleOp.Params.ROT_SERVO_BACK,0,"ClawRotation",false));
            transferParallel.add(createClawTask(robot, RoboSapiensTeleOp.Params.CLAW_OPEN, 0, "Claw", false));
            transferParallel.add(createVerticalSlideTask(robot, RoboSapiensTeleOp.Params.VERTICAL_SLIDE_MIDDLE_POSITION, 0, "Vertical", false));
            transferParallel.add(createClawAngleTask( robot, RoboSapiensTeleOp.Params.CLAW_ANGLE_BACK, 0, "ClawAngle", false));

            transferParallel.add(createClawSlideTask( robot, RoboSapiensTeleOp.Params.CLAW_SLIDER_BACK, 375, "ClawSlide", false));
            transferParallel.add(createHorizontalSlideTask(robot, 0, 0, "Horizontal", false));
            transferParallel.add(createVerticalSlideTask(robot, RoboSapiensTeleOp.Params.VERTICAL_SLIDE_DOWN_POSITION, 375, "Vertical", false));
//            transferParallel.add(createClawAngleTask( robot, RoboSapiensTeleOp.Params.CLAW_ANGLE_BACK, 50, "ClawAngle", false));
            transferParallel.add(
                    new ExecuteOnceTask(
                            new ExecuteOnceTask.ExecuteListener() {
                                @Override
                                public void execute() {
                                    angle_ready = true;
                                }
                            }, "Substate Transition"
                    )
            );
        } else {
            transferParallel.add(createClawRotationTask(robot, RoboSapiensTeleOp.Params.ROT_SERVO_BACK,0,"ClawRotation",false));
            transferParallel.add(createClawTask(robot, RoboSapiensTeleOp.Params.CLAW_OPEN, 0, "Claw", false));
            transferParallel.add(createVerticalSlideTask(robot, RoboSapiensTeleOp.Params.VERTICAL_SLIDE_MIDDLE_POSITION, 0, "Vertical", false));
            transferParallel.add(createClawAngleTask( robot, RoboSapiensTeleOp.Params.CLAW_ANGLE_BACK, 0, "ClawAngle", false));
            transferParallel.add(createClawSlideTask( robot, RoboSapiensTeleOp.Params.CLAW_SLIDER_BACK, 375, "ClawSlide", false));
            transferParallel.add(createHorizontalSlideTask(robot, 0, 0, "Horizontal", false));
            transferParallel.add(createVerticalSlideTask(robot, RoboSapiensTeleOp.Params.VERTICAL_SLIDE_DOWN_POSITION, 375, "Vertical", false));
//            transferParallel.add(createClawAngleTask( robot, RoboSapiensTeleOp.Params.CLAW_ANGLE_BACK, 50, "ClawAngle", false));

            transferParallel.add(
                    new ExecuteOnceTask(
                            new ExecuteOnceTask.ExecuteListener() {
                                @Override
                                public void execute() {
                                    angle_ready = true;
                                }
                            }, "Substate Transition"
                    )
            );


//            transferParallel.add(createClawRotationTask(robot, RoboSapiensTeleOp.Params.ROT_SERVO_BACK,0,"ClawRotation",false));

//        transferParallel.add(createVerticalSlideTask(robot, DriveTest.Params.VERTICAL_SLIDE_DOWN_POSITION, 0, "Vertical", false));
        }


        taskArrayList.add(transferParallel);
    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {

        if(isDriftModeEnabled && robot.isDrifting() && !joystick.gamepad1GetBRaw()) {
            angle_ready = false;
            robot.setDriftMode(false, 0, 0);
            robot.switchState(State.SPECIMEN_HANG);
        } else {


            if (joystick.gamepad1GetB()) {
                //angle_ready = false;
                if(isDriftModeEnabled) {
                    robot.setDriftMode(true, 0, .7);
                } else {
                    robot.switchState(State.SPECIMEN_HANG);
                }
                //robot.switchState(State.SPECIMEN_HANG);
            } else if (joystick.gamepad1GetA()) {
                //robot.setDriftMode(false, 0, 0);
                robot.switchState(State.INTAKINGCLAW);
            } else if (joystick.gamepad1GetY()) {
                isDriftModeEnabled = !isDriftModeEnabled;
            }
        }

//        if(joystick.gamepad1GetLeftBumperRaw()) {
//            robot.increseClawSlideTargetPosition((int) (joystick.gamepad1GetLeftTrigger()*-500));
//        } else {
//            robot.increseClawSlideTargetPosition((int) (joystick.gamepad1GetLeftTrigger()*500));
//        }

        if(joystick.gamepad1GetRightBumperRaw()) {
            robot.increaseVerticalSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger()*-100));
        } else {
            robot.increaseVerticalSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger()*100));
        }

//        if (joystick.gamepad1GetDLeft()) {
//            robot.setClawHorizontalAnglePosition(RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_LEFT);
//        } else if (joystick.gamepad1GetDRight()) {
//            robot.setClawHorizontalAnglePosition(RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_RIGHT);
//        } else if (joystick.gamepad1GetDDown()) {
//            robot.setClawHorizontalAnglePosition(RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER);
//        }

        if (angle_ready)
            robot.autoHorizontalPosWall(telemetry);

        telemetry.addData("angle_ready", angle_ready);

        executeTasks(telemetry);
    }

    @Override
    public State getState() {
        return State.WALLPICKUP;
    }
}
