package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class InitialState extends BaseState {


    public InitialState(JoystickWrapper joystick) {
        super(joystick);
    }

    @Override
    public void initialize(Robot robot, IRobot prevState){

        if(prevState == null) {
            robot.setClawPosition(RoboSapiensTeleOp.Params.CLAW_OPEN);
            robot.setClawRotationPosition(RoboSapiensTeleOp.Params.ROT_SERVO_DEFAULT);
            robot.setClawAnglePosition(RoboSapiensTeleOp.Params.CLAW_ANGLE_DOWN);
            robot.setClawSlideTargetPosition(RoboSapiensTeleOp.Params.CLAW_SLIDER_DOWN);
            robot.setVerticalSlideTargetPosition(RoboSapiensTeleOp.Params.VERTICAL_SLIDE_POSITION);
            robot.setHorizontalSlideTargetPosition(0);
            robot.setIntakeAngleServo(RoboSapiensTeleOp.Params.INTAKE_ANGLE_TRANSFER);
        } else {
            RobotTaskSeries transferSeries = new RobotTaskSeries();
            transferSeries.add(createClawTask(robot, RoboSapiensTeleOp.Params.CLAW_OPEN, 1000, "ClawClose", true));
            transferSeries.add(createHorizontalSlideTask(robot, 0, 1000, "HorizontalSlide", true));
            transferSeries.add(createClawSlideTask(robot, RoboSapiensTeleOp.Params.CLAW_SLIDER_DOWN, 1000, "CLAW_SLIDER_BACK", true));
            transferSeries.add(createClawAngleTask(robot, RoboSapiensTeleOp.Params.CLAW_ANGLE_DOWN, 1000, "CLAW_ANGLE_BACK", true));
            transferSeries.add(createClawRotationTask(robot, RoboSapiensTeleOp.Params.ROT_SERVO_DEFAULT, 1000, "ROT_SERVO_BACK", true));
            transferSeries.add(createVerticalSlideTask(robot, RoboSapiensTeleOp.Params.VERTICAL_SLIDE_POSITION, 1000, "HorizontalSlide", false));
            taskArrayList.add(transferSeries);
        }
    }


    @Override
    public void execute(Robot robot, Telemetry telemetry) {


        if(joystick.gamepad1GetA()) {
            robot.switchState(State.WALLPICKUP);
        } else if(joystick.gamepad1GetB()) {
            robot.switchState(State.INTAKINGCLAW);
        } else if(joystick.gamepad1GetY()) {
            robot.switchState(State.SERVO_TEST);
        }
        //robot.switchState(State.DROPPING_L1);


//        else if(joystick.gamepad1GetX()) {
//            robot.setIntakeAngleServoPosition(.48);
//        } else if(joystick.gamepad1GetB()) {
//            robot.switchState(State.SERVO_TEST);
//        } else if(joystick.gamepad1GetRightBumperDown()) {
//            robot.setClawPosition(DriveTest.Params.CLAW_CLOSE);
//        } else if(joystick.gamepad1GetLeftBumperDown()) {
//            robot.setClawPosition(DriveTest.Params.CLAW_OPEN);
//        } else if(joystick.gamepad1GetDUp()) {
//            robot.increseVerticalSlideTargetPosition(DriveTest.Params.VERTICAL_SLIDE_TRANSFER_POSITION);
//        } else if(joystick.gamepad1GetDDown()) {
//            robot.setVerticalSlideTargetPosition(DriveTest.Params.VERTICAL_SLIDE_POSITION);
//        } else if(joystick.gamepad1GetDRight()) {
//            robot.increseHorizontalSlideTargetPosition(50);
//        } else if(joystick.gamepad1GetDLeft()) {
//            robot.increseHorizontalSlideTargetPosition(-50);
//        }
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

       // robot.setIntakePower(joystick.gamepad1GetRightTrigger()-joystick.gamepad1GetLeftTrigger());


        executeTasks(telemetry);
    }



    @Override
    public State getState() {
        return State.INITIAL;
    }
}
