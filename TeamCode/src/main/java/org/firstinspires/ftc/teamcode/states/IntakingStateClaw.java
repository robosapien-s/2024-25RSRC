package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.ExecuteOnceTask;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class IntakingStateClaw extends BaseState {

//    int clawRotationStateHack = 0;

//    boolean leftVertical = true;

    int clawStateHack = 0; //0 = ready, 1 = pickup, 2 = transfer

     double[] clawRotationPositions = new double[]{


     };
    public IntakingStateClaw(JoystickWrapper joystick) {
        super(joystick);
    }

    @Override
    public void start(Robot robot, Telemetry telemetry) {
        robot.setIntakeAnglePosition(RoboSapiensTeleOp.Params.INTAKE_ANGLE_READY);
    }


    @Override
    public void initialize(Robot robot, IRobot prevState) {
        taskArrayList.add(new ExecuteOnceTask(
                new ExecuteOnceTask.ExecuteListener() {
                    @Override
                    public void execute() {
                        robot.setHangServo(RoboSapiensTeleOp.Params.HANG_SERVO_CLOSED);
                    }
                }, "hang servo"
        ));
        robot.setSlideMinPosition(110);
        robot.setSlideRotationMaxPosition(RoboSapiensTeleOp.Params.SLIDE_ROTATION_MAX_POSITION);

        if (prevState == null || (prevState.getState() != State.DROPPING_L2 && prevState.getState() != State.AUTO_BUCKET)) {
            robot.setSlideMaxPosition(RoboSapiensTeleOp.Params.SLIDE_MAX_POSITION);
        }

        if(prevState == null) {
            //use robot.set directly, not a task series
            robot.setSlideMaxPosition(RoboSapiensTeleOp.Params.SLIDE_MAX_POSITION);
            robot.setSlideTargetPosition(70);
            robot.setSlideRotationPosition(0);
            robot.setRotAndAnglePosition(RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PREP);
            robot.setClawPosition(RoboSapiensTeleOp.Params.CLAW_OPEN);
            robot.setIntakeAnglePosition(RoboSapiensTeleOp.Params.INTAKE_ANGLE_READY);

        } else if (prevState.getState() == State.SPECIMEN_HANG_FRONT) {
            taskArrayList.add(createClawTask(robot, RoboSapiensTeleOp.Params.CLAW_OPEN, 100, "Open Claw", false));

            taskArrayList.add(createIntakeAngleServoTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_PICKUP, 100, "IntakeAngle", false));

            taskArrayList.add(createSlideTask(robot, 0, 100, "Slide", false));
            taskArrayList.add(createIntakeAngleServoTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_READY, 0, "IntakeAngle", false));
            taskArrayList.add(createRotationAndAngleTask(robot, RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PREP, 100, "RotAndAngle", false));


            taskArrayList.add(createSlideRotationTask(robot, 0, 0, "Rotation", false));
        } else if (prevState.getState() == State.PICKUP_GROUND_LEFT) {
            robot.setSlideTargetPosition(70);
        } else if (prevState.getState() == State.DROPPING_L2 || prevState.getState() == State.AUTO_BUCKET) {

            taskArrayList.add(createIntakeAngleServoTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_READY, 0, "IntakeAngle", false));

            taskArrayList.add(createRotationAndAngleTask(robot, RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PREP, 0, "Rotation and Angle", false));

            taskArrayList.add(createSlideRotationTask(robot, RoboSapiensTeleOp.Params.SLIDE_ROTATION_MIDDLE_POSITION, 50, "Rotation", false));

            taskArrayList.add(createSlideTask(robot, 0, 300, "Slide", false));


            taskArrayList.add(createSlideRotationTask(robot, 0, 200, "Rotation", false));

            taskArrayList.add(new ExecuteOnceTask(
                    new ExecuteOnceTask.ExecuteListener() {
                        @Override
                        public void execute() {
                            robot.setSlideMaxPosition(RoboSapiensTeleOp.Params.SLIDE_MAX_POSITION);
                        }
                    }, "setting max slide position"
            ));


        } else if (prevState.getState() == State.PICKUP_GROUND) {
            robot.setClawPosition(RoboSapiensTeleOp.Params.CLAW_OPEN);
        } else if (prevState.getState() == State.SPECIMEN_HANG) {
            taskArrayList.add(createRotationAndAngleTask(robot, RoboSapiensTeleOp.Params.ROT_AND_ANGLE_SPECIMEN_DOWN, 0, "Rot and Angle", false));
            taskArrayList.add(createSlideTask(robot, RoboSapiensTeleOp.Params.SLIDE_SPECIMEN_DOWN_POSITION, 150, "Slide", false));
            taskArrayList.add(createClawTask(robot, RoboSapiensTeleOp.Params.CLAW_OPEN, 50, "Claw Open", false));
            taskArrayList.add(createSlideTask(robot, 0, 0, "Slide", false));
            taskArrayList.add(createIntakeAngleServoTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_READY, 0, "Intake Angle", false));
            taskArrayList.add(createRotationAndAngleTask(robot, RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PREP, 0, "Rot and Angle", false));
            taskArrayList.add(createSlideRotationTask(robot, 0, 0, "Rotation", false));
        } else if (prevState.getState() == State.ROBOT_HANG) {
            taskArrayList.add(new ExecuteOnceTask(
                    new ExecuteOnceTask.ExecuteListener() {
                        @Override
                        public void execute() {
                            robot.setHangServo(RoboSapiensTeleOp.Params.HANG_SERVO_OPEN);
                        }
                    }, "hang servo"
            ));
            taskArrayList.add(createRotationAndAngleTask(robot, RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PREP, 0, "rot and angle", false));
            taskArrayList.add(createIntakeAngleServoTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_READY, 250, "intake angle", false));
            taskArrayList.add(createSlideTask(robot, 0, 250, "slide", false));
            taskArrayList.add(createSlideRotationTask(robot, 0, 0, "slide rotation", false));
        }
        else {
            robot.setSlideTargetPosition(70);
            robot.setSlideRotationPosition(0);
            robot.setRotAndAnglePosition(RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PREP);
            robot.setIntakeAnglePosition(RoboSapiensTeleOp.Params.INTAKE_ANGLE_READY);
        }
    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {

        if (joystick.gamepad2GetA()) {
            robot.switchState(State.SERVO_TEST);
        } else if(joystick.gamepad2GetY()) {
            robot.switchState(State.ROBOT_HANG);
        }

        if(joystick.gamepad1GetY()) {
            robot.switchState(State.ROBOT_HANG);
        } else if(joystick.gamepad1GetA()) {

//            leftVertical = true;

            robot.setRobotSpeedNormal();

            RobotTaskSeries transferSeries = new RobotTaskSeries();

            if(clawStateHack == 0) {

                double clawPosition = robot.getClawPosition();
                if(Math.abs(  clawPosition - RoboSapiensTeleOp.Params.CLAW_OPEN ) > .02) {
                    transferSeries.add(createClawTask(robot, RoboSapiensTeleOp.Params.CLAW_OPEN, 250, "IntakeClawOpen", false));
                }

//                transferSeries.add(createRotationAndAngleTask(robot, RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PICKUP_HORIZONTAL, 50, "IntakeAngle", false));

                transferSeries.add(createIntakeAngleServoTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_PICKUP, 50, "IntakeAngle", false));

                transferSeries.add(createClawTask(robot, RoboSapiensTeleOp.Params.CLAW_CLOSE, 300, "IntakeClawClose", false));


                transferSeries.add(createRotationAndAngleTask(robot, RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PREP, 0, "IntakeAngle", false));

                transferSeries.add(createIntakeAngleServoTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_READY, 50, "IntakeAngle", false));

                clawStateHack = 1;
            } else if(clawStateHack == 1) {
                transferSeries.add(createClawTask(robot, RoboSapiensTeleOp.Params.CLAW_OPEN, 250, "IntakeClawOpen", false));
                clawStateHack = 0;
            }


            /*
            if(clawStateHack == 0) {

                transferSeries.add(createClawTask(robot, DriveTest.Params.CLAW_OPEN, 1, "Claw", false));
                transferSeries.add(createClawAngleTask(robot, DriveTest.Params.CLAW_ANGLE_TRANSFER, 1, "IntakeClawOpen", false));
                //transferSeries.add(createClawSlideTask( robot, DriveTest.Params.CLAW_SLIDER_TRANSFER, 1, "ClawSlide", false));


                transferSeries.add(createIntakeClawTask(robot, DriveTest.Params.INTAKE_CLAW_OPEN, 100, "IntakeClawOpen", false));
                transferSeries.add(createIntakeClawAngleTask(robot, DriveTest.Params.INTAKE_ANGLE_PICKUP, 100, "IntakeAngle", false));
                transferSeries.add(createIntakeKnuckleTask(robot, DriveTest.Params.INTAKE_KNUCKLE_PICKUP, 100, "KnucklePickUp", false));
                transferSeries.add(createIntakeClawTask(robot, DriveTest.Params.INTAKE_CLAW_CLOSE, 500, "IntakeClawClose", false));
                //transferSeries.add(createHorizontalSlideTask(robot, DriveTest.Params.HORIZONTAL_SLIDE_TRANSFER_POSITION, 1, "IntakeClawClose", false));
                transferSeries.add(createIntakeRotationTask(robot, DriveTest.Params.INTAKE_ROT_SERVO_DEFAULT, 1, "IntakeClawClose", false));

                transferSeries.add(createIntakeClawAngleTask(robot, DriveTest.Params.INTAKE_ANGLE_TRANSFER, 1, "IntakeClawClose", false));
                transferSeries.add(createIntakeKnuckleTask(robot, DriveTest.Params.INTAKE_KNUCKLE_TRANSFER, 400, "IntakeClawClose", false));


                transferSeries.add(createIntakeClawTask(robot, DriveTest.Params.INTAKE_CLAW_LOOSE, 300, "IntakeClawLoose", false));
                //transferSeries.add(createClawSlideTask( robot, DriveTest.Params.CLAW_SLIDER_TRANSFER+800, 200, "ClawSlide", false));
                transferSeries.add(createClawTask(robot, DriveTest.Params.CLAW_CLOSE, 200, "Claw", false));
                transferSeries.add(createIntakeClawTask(robot, DriveTest.Params.INTAKE_CLAW_OPEN, 1, "IntakeClawOpen", false));
                clawStateHack = 1;
            } else if(clawStateHack == 1) {
                //transferSeries.add(createClawSlideTask( robot, DriveTest.Params.CLAW_SLIDER_DOWN, 1, "ClawSlide", false));
                transferSeries.add(createIntakeClawTask(robot, DriveTest.Params.INTAKE_CLAW_OPEN, 1, "IntakeClawOpen", false));
                transferSeries.add(createIntakeClawAngleTask(robot, DriveTest.Params.INTAKE_ANGLE_READY, 1, "IntakeAngle", false));
                transferSeries.add(createIntakeKnuckleTask(robot, DriveTest.Params.INTAKE_KNUCKLE_PICKUP, 1000, "KnucklePickUp", false));
                transferSeries.add(createClawTask(robot, DriveTest.Params.CLAW_OPEN, 1, "Claw", false));
                clawStateHack = 0;
            }
            */

            //transferSeries.add(createIntakeClawAngleTask(robot, DriveTest.Params.INTAKE_ANGLE_TRANSFER, 1, "IntakeClawClose", false));
            taskArrayList.add(transferSeries);
        }


        else if(joystick.gamepad1GetB()) {
            robot.setRobotSpeedNormal();
            robot.switchState(State.WALLPICKUP);
        }

//        else if(joystick.gamepad1GetX()) {
//
//            robot.setRobotSpeedNormal();
//            ;
//
//            taskArrayList.add( new ExecuteOnceTask(new ExecuteOnceTask.ExecuteListener() {
//                @Override
//                public void execute() {
//                    robot.switchState(State.DROPPING_L2);
//                }
//            }, "Set Drop State"));
//
//
//        }


        else if(joystick.gamepad1GetX()) {

            robot.setRobotSpeedNormal();

            robot.switchState(State.DROPPING_L2);

//            if (!DroppingL1State.autoL2) {
//                robot.switchState(State.DROPPING_L2);
//            } else {
//                robot.switchState(State.AUTO_BUCKET);
//            }


        }

        if (joystick.gamepad1GetDUp()) {
            robot.setRotAndAnglePosition(RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PREP);
        } else if (joystick.gamepad1GetDRight()) {
            robot.setRotAndAnglePosition(RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PICKUP_RIGHT);
        } else if (joystick.gamepad1GetDLeft()) {
            robot.setRotAndAnglePosition(RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PICKUP_LEFT);
        } else if (joystick.gamepad1GetDDown()) {
//            if (!leftVertical) {
//                robot.setRotAndAnglePosition(RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PICKUP_VERTICAL_RIGHT);
//            } else {
//                robot.setRotAndAnglePosition(RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PICKUP_VERTICAL_LEFT);
//            }
//
//            leftVertical = !leftVertical;

            robot.setRotAndAnglePosition(RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PICKUP_VERTICAL);

        }

        robot.increaseSlideTargetPosition((int) (joystick.gamepad1GetLeftTrigger()*(-100)+joystick.gamepad1GetRightTrigger()*100));

        executeTasks(telemetry);

    }

    @Override
    public State getState() {
        return State.INTAKINGCLAW;
    }
}
