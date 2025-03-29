package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.ExecuteOnceTask;
import org.firstinspires.ftc.teamcode.controllers.IRobotTask;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class IntakingStateClaw extends BaseState {

    int clawRotationStateHack = 0;

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
        robot.setSlideMinPosition(110);
        robot.setSlideMaxPosition(880);

        if(prevState == null) {
            //use robot.set directly, not a task series
            robot.setSlideTargetPosition(70);
            robot.setSlideRotationPosition(0);
            robot.setRotAndAnglePosition(RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PREP);
            robot.setClawPosition(RoboSapiensTeleOp.Params.CLAW_OPEN);
            robot.setIntakeAnglePosition(RoboSapiensTeleOp.Params.INTAKE_ANGLE_START);

        } else if (prevState.getState() == State.SPECIMEN_HANG) {
            RobotTaskSeries transferSeries = new RobotTaskSeries();

//            transferSeries.add();

            taskArrayList.add(transferSeries);
        } else if (prevState.getState() == State.PICKUP_GROUND_LEFT) {
            robot.setSlideTargetPosition(70);
        } else if (prevState.getState() == State.DROPPING_L2) {

            taskArrayList.add(createRotationAndAngleTask(robot, RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PREP, 50, "IntakeAngle", false));

            taskArrayList.add(createSlideRotationTask(robot, RoboSapiensTeleOp.Params.SLIDE_ROTATION_MIDDLE_POSITION, 200, "Rotation", false));

            taskArrayList.add(createSlideTask(robot, 0, 300, "Slide", false));

            taskArrayList.add(createSlideRotationTask(robot, 0, 200, "Rotation", false));


        } else {
            RobotTaskSeries transferSeries = new RobotTaskSeries();

            taskArrayList.add(transferSeries);
        }
    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {

        if (joystick.gamepad2GetA()) {
            robot.switchState(State.SERVO_TEST);
        }

        if(joystick.gamepad1GetY()) {
//            robot.switchState(State.AUTO_PICKUP);
        } else if(joystick.gamepad1GetA()) {

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


        if(joystick.gamepad1GetB()) {
            robot.setRobotSpeedNormal();
            robot.switchState(State.WALLPICKUP);
        }

        if(joystick.gamepad1GetX()) {

            robot.setRobotSpeedNormal();
            RobotTaskSeries transferSeries = new RobotTaskSeries();

            transferSeries.add( new ExecuteOnceTask(new ExecuteOnceTask.ExecuteListener() {
                @Override
                public void execute() {
                    robot.switchState(State.DROPPING_L2);
                }
            }, "Set Drop State"));

            taskArrayList.add(transferSeries);



        }

        if (joystick.gamepad1GetDUp()) {
            robot.setRotAndAnglePosition(RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PREP);
        } else if (joystick.gamepad1GetDRight()) {
            robot.setRotAndAnglePosition(RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PICKUP_RIGHT);
        } else if (joystick.gamepad1GetDLeft()) {
            robot.setRotAndAnglePosition(RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PICKUP_LEFT);
        } else if (joystick.gamepad1GetDDown()) {
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
