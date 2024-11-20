package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.CallBackTask;
import org.firstinspires.ftc.teamcode.controllers.IRobotTask;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.opmodes.DriveTest;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

import java.util.ArrayList;

public class InitialState implements IRobot {

    private final JoystickWrapper joystick;


    ArrayList<IRobotTask> taskArrayList = new ArrayList<IRobotTask>();

    public InitialState(JoystickWrapper joystick) {
        this.joystick = joystick;
    }

    @Override
    public void initialize(Robot robot){
        robot.setClawPosition(DriveTest.Params.CLAW_OPEN);
        robot.setClawRotationPosition(DriveTest.Params.ROT_SERVO_DEFAULT);
        robot.setClawAnglePosition(DriveTest.Params.CLAW_ANGLE_DOWN);
        robot.setClawSlideTargetPosition(DriveTest.Params.CLAW_SLIDER_DOWN);
        robot.setVerticalSlideTargetPosition(120);
        robot.setHorizontalSlideTargetPosition(0);
        robot.setIntakeAngleServoPosition(DriveTest.Params.INTAKE_ANGLE_TRANSFER);
    }


    @Override
    public void execute(Robot robot, Telemetry telemetry) {


        if(joystick.gamepad1GetA()) {
            //robot.switchState(State.INTAKING);

            RobotTaskSeries transferSeries = new RobotTaskSeries();
            transferSeries.add(new CallBackTask(new CallBackTask.CallBackListener() {
                @Override
                public void setPosition(double value) {
                    robot.setClawPosition(value);
                }

                @Override
                public double getPosition() {
                    return DriveTest.Params.CLAW_CLOSE;
                }
            }, DriveTest.Params.CLAW_CLOSE, 1, "", true));


            transferSeries.add(new CallBackTask(new CallBackTask.CallBackListener() {
                @Override
                public void setPosition(double value) {
                    robot.setHorizontalSlideTargetPosition((int) value);

                }

                @Override
                public double getPosition() {

                    return robot.getHorizontalSlidePosition();
                }
            }, DriveTest.Params.HORIZONTAL_SLIDE_TRANSFER_POSITION, 1, "", true));

            transferSeries.add(new CallBackTask(new CallBackTask.CallBackListener() {
                @Override
                public void setPosition(double value) {
                    robot.setVerticalSlideTargetPosition((int) value);

                }

                @Override
                public double getPosition() {

                    return robot.getVerticalSlidePosition();
                }
            }, DriveTest.Params.VERTICAL_SLIDE_TRANSFER_POSITION, 1, "", true));

            taskArrayList.add(transferSeries);

        } else if(joystick.gamepad1GetX()) {
            robot.setIntakeAngleServoPosition(.48);
        } else if(joystick.gamepad1GetB()) {
            robot.setIntakeAngleServoPosition(.51);
        } else if(joystick.gamepad1GetRightBumperDown()) {
            robot.setClawPosition(DriveTest.Params.CLAW_CLOSE);
        } else if(joystick.gamepad1GetLeftBumperDown()) {
            robot.setClawPosition(DriveTest.Params.CLAW_OPEN);
        } else if(joystick.gamepad1GetDUp()) {
            robot.increseVerticalSlideTargetPosition(DriveTest.Params.VERTICAL_SLIDE_TRANSFER_POSITION);
        } else if(joystick.gamepad1GetDDown()) {
            robot.setVerticalSlideTargetPosition(DriveTest.Params.VERTICAL_SLIDE_POSITION);
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


        executeTasks(telemetry);
    }

    public void executeTasks(Telemetry telemetry) {

        if(!taskArrayList.isEmpty()) {

            boolean isStarted = taskArrayList.get(0).hasStarted();
            boolean isRunning = taskArrayList.get(0).isRunning();
            boolean isComplete = taskArrayList.get(0).isComplete();

            taskArrayList.get(0).execute(telemetry);


            if(isComplete){
                taskArrayList.remove(0);
            }
        }
    }

    @Override
    public State getState() {
        return State.INITIAL;
    }
}
