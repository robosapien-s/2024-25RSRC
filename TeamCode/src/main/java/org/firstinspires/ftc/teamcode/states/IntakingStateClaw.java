package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.CallBackTask;
import org.firstinspires.ftc.teamcode.controllers.ExecuteOnceTask;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.DriveTest;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

import java.util.ArrayList;

public class IntakingStateClaw extends BaseState {

    int clawRotationStateHack = 0;

    int clawStateHack = 0; //0 = ready, 1 = pickup, 2 = transfer

     double[] clawRotationPositions = new double[]{
            DriveTest.Params.INTAKE_ROT_SERVO_DEFAULT,
            DriveTest.Params.INTAKE_ROT_SERVO_DEFAULT+.16,
            DriveTest.Params.INTAKE_ROT_SERVO_DEFAULT-.16
     };
    public IntakingStateClaw(JoystickWrapper joystick) {
        super(joystick);
    }

    @Override
    public void initialize(Robot robot, IRobot prevState) {
        if(prevState == null) {
            robot.setIntakeClawServo(DriveTest.Params.INTAKE_CLAW_OPEN);
            robot.setIntakeRotationServo(DriveTest.Params.INTAKE_ROT_SERVO_DEFAULT);
            robot.setIntakeAngleServo(DriveTest.Params.INTAKE_ANGLE_READY);
            robot.setIntakeKnuckleServo(DriveTest.Params.INTAKE_KNUCKLE_PICKUP);

            robot.setClawAnglePosition(DriveTest.Params.CLAW_ANGLE_FORWARD_SPECIMEN);
            robot.setClawPosition(DriveTest.Params.CLAW_OPEN);
            robot.setClawRotationPosition(DriveTest.Params.ROT_SERVO_DEFAULT);

            robot.setVerticalSlideTargetPosition(DriveTest.Params.VERTICAL_SLIDE_POSITION);
            robot.setHorizontalSlideTargetPosition(0);
            robot.setDualSlideTargetPosition(DriveTest.Params.CLAW_SLIDER_TRANSFER);

        } else {
            RobotTaskSeries transferSeries = new RobotTaskSeries();
            transferSeries.add(createIntakeClawTask(robot, DriveTest.Params.INTAKE_CLAW_OPEN, 1, "INTAKE_CLAW_OPEN", false));
            transferSeries.add(createIntakeRotationTask(robot, DriveTest.Params.INTAKE_ROT_SERVO_DEFAULT, 1, "INTAKE_ROT_SERVO_DEFAULT", false));
            transferSeries.add(createClawSlideTask(robot, DriveTest.Params.CLAW_SLIDER_TRANSFER, 500, "CLAW_SLIDER_TRANSFER", false));
            transferSeries.add(createClawAngleTask(robot, DriveTest.Params.CLAW_ANGLE_TRANSFER, 1, "CLAW_ANGLE_BACK", false));

            transferSeries.add(createIntakeClawAngleTask(robot, DriveTest.Params.INTAKE_ANGLE_READY, 1, "CLAW_ANGLE_BACK", false));
            transferSeries.add(createIntakeKnuckleTask(robot, DriveTest.Params.INTAKE_KNUCKLE_PICKUP, 1, "INTAKE_KNUCKLE_PICKUP", false));
            transferSeries.add(createVerticalSlideTask(robot, DriveTest.Params.VERTICAL_SLIDE_POSITION, 500, "VerticalSlide", false));
            taskArrayList.add(transferSeries);
        }
    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {


        if(joystick.gamepad1GetA()) {
            RobotTaskSeries transferSeries = new RobotTaskSeries();


            if(clawStateHack == 0) {

                double intakeClawServo = robot.getIntakeClawServo();
                if(Math.abs(  intakeClawServo - DriveTest.Params.INTAKE_CLAW_OPEN ) > .02) {
                    transferSeries.add(createIntakeClawTask(robot, DriveTest.Params.INTAKE_CLAW_OPEN, 500, "IntakeClawOpen", false));
                }

                transferSeries.add(createIntakeClawAngleTask(robot, DriveTest.Params.INTAKE_ANGLE_PICKUP, 100, "IntakeAngle", false));
                transferSeries.add(createIntakeKnuckleTask(robot, DriveTest.Params.INTAKE_KNUCKLE_PICKUP, 100, "KnucklePickUp", false));
                transferSeries.add(createIntakeClawTask(robot, DriveTest.Params.INTAKE_CLAW_CLOSE, 500, "IntakeClawClose", false));
                //transferSeries.add(createHorizontalSlideTask(robot, DriveTest.Params.HORIZONTAL_SLIDE_TRANSFER_POSITION, 1, "IntakeClawClose", false));
                transferSeries.add(createIntakeRotationTask(robot, DriveTest.Params.INTAKE_ROT_SERVO_DEFAULT, 1, "IntakeClawClose", false));
                transferSeries.add(createIntakeClawAngleTask(robot, DriveTest.Params.INTAKE_ANGLE_READY, 1, "CLAW_ANGLE_BACK", false));

                clawStateHack = 1;
            } else if(clawStateHack == 1) {
                transferSeries.add(createIntakeClawTask(robot, DriveTest.Params.INTAKE_CLAW_OPEN, 1, "IntakeClawOpen", false));
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

           robot.switchState(State.WALLPICKUP);
        }

        if(joystick.gamepad1GetX()) {


            RobotTaskSeries transferSeries = new RobotTaskSeries();

            transferSeries.add(createHorizontalSlideTask(robot, DriveTest.Params.HORIZONTAL_SLIDE_TRANSFER_POSITION, 1, "Claw", false));
            transferSeries.add(createClawTask(robot, DriveTest.Params.CLAW_OPEN, 1, "Claw", false));
            transferSeries.add(createClawSlideTask( robot, DriveTest.Params.CLAW_SLIDER_TRANSFER, 1, "ClawSlide", false));

            transferSeries.add(createVerticalSlideTask(robot, DriveTest.Params.VERTICAL_SLIDE_TRANSFER_POSITION, 1, "IntakeClawClose", false));
            transferSeries.add(createClawAngleTask(robot, DriveTest.Params.CLAW_ANGLE_TRANSFER, 1000, "IntakeClawOpen", false));


            transferSeries.add(createIntakeRotationTask(robot, DriveTest.Params.INTAKE_ROT_SERVO_DEFAULT, 1000, "IntakeClawClose", false));

            transferSeries.add(createIntakeKnuckleTask(robot, DriveTest.Params.INTAKE_KNUCKLE_TRANSFER, 400, "IntakeClawClose", false));
            transferSeries.add(createIntakeClawTask(robot, DriveTest.Params.INTAKE_CLAW_LOOSE, 300, "IntakeClawLoose", false));
            transferSeries.add(createIntakeClawAngleTask(robot, DriveTest.Params.INTAKE_ANGLE_TRANSFER, 1000, "IntakeClawClose", false));



            //transferSeries.add(createClawSlideTask( robot, DriveTest.Params.CLAW_SLIDER_TRANSFER+800, 200, "ClawSlide", false));
            transferSeries.add(createClawTask(robot, DriveTest.Params.CLAW_CLOSE, 200, "Claw", false));
            transferSeries.add(createIntakeClawTask(robot, DriveTest.Params.INTAKE_CLAW_OPEN, 100, "IntakeClawOpen", false));

            transferSeries.add( new ExecuteOnceTask(new ExecuteOnceTask.ExecuteListener() {
                @Override
                public void execute() {
                    robot.switchState(State.DROPPING_L1);
                }
            }, "Set Drop State"));

            taskArrayList.add(transferSeries);



        }

//        if(joystick.gamepad1GetLeftBumperDown()) {
//            clawRotationStateHack++;
//            if(clawRotationStateHack>2) {
//                clawRotationStateHack = 0;
//            }
//            robot.setIntakeRotationServo(clawRotationPositions[clawRotationStateHack]);
//        }

        if(joystick.gamepad1GetDUp()) {
            robot.setIntakeRotationServo(clawRotationPositions[0]);
        } else if (joystick.gamepad1GetDRight()) {
            robot.setIntakeRotationServo(clawRotationPositions[1]);
        } else if(joystick.gamepad1GetDLeft()) {
            robot.setIntakeRotationServo(clawRotationPositions[2]);
        }

        /*
        if(joystick.gamepad1GetRightBumperRaw()) {
            robot.increseHorizontalSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger()*-10));
        } else {
            robot.increseHorizontalSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger()*10));
        }
        */

        if(joystick.gamepad1GetLeftBumperRaw()) {
            robot.increseHorizontalSlideTargetPosition((int) (joystick.gamepad1GetLeftTrigger()*-20));
        } else {
            robot.increseHorizontalSlideTargetPosition((int) (joystick.gamepad1GetLeftTrigger()*20));
        }

        if(joystick.gamepad1GetRightBumperRaw()) {
            robot.increseVerticalSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger()*-100));
        } else {
            robot.increseVerticalSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger()*100));
        }



        /* Intake class angle
        0. Closed   .4
        1. Transfer .5383
        2. Pickup Ready  .6183
        3. Pikcup .6983
        */


        /* Intake class Knuckle
        1. Transfer .84
        3. Pikcup .1
        */



        /* Intake claw rotation
        1. intake .52
        3. leftt1.  .64
        4. left2.   .82
        4. right1. .34
        5. right2.d  .24
        */


        /*
            claw open .46
            claw closed .53

         */








        if(joystick.gamepad1GetA()) {
          //  robot.switchState(State.SERVO_TEST);
        }

//        if(joystick.gamepad1GetA()) {
//            robot.switchState((State.WALLPICKUP));
//        } else if(joystick.gamepad1GetB()) {
//            robot.switchState(State.DROPPING_L1);
//        }
//
//        if(joystick.gamepad1GetRightBumperRaw()) {
//            robot.increseHorizontalSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger()*-10));
//        } else {
//            robot.increseHorizontalSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger()*10));
//        }
//
//
//        if(joystick.gamepad1GetLeftBumperRaw()) {
//            robot.setIntakePower(joystick.gamepad1GetLeftTrigger());
//        } else {
//            robot.setIntakePower(-joystick.gamepad1GetLeftTrigger());
//        }

        executeTasks(telemetry);

    }

    @Override
    public State getState() {
        return State.INTAKINGCLAW;
    }
}
