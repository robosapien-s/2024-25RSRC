package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.CallBackTask;
import org.firstinspires.ftc.teamcode.controllers.IRobotTask;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.DriveTest;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

import java.util.ArrayList;

public abstract class BaseState implements IRobot {

    protected final JoystickWrapper joystick;
    ArrayList<IRobotTask> taskArrayList = new ArrayList<IRobotTask>();

    protected BaseState(JoystickWrapper joystick) {
        this.joystick = joystick;
    }


    protected IRobotTask createClawRotationTask(Robot robot, double position, int duration, String name, boolean steps) {

        return new CallBackTask(new CallBackTask.CallBackListener() {
            @Override
            public void setPosition(double value) {
                robot.setClawRotationPosition(value);
            }

            @Override
            public double getPosition() {
                return position;
            }
        }, position, duration, name, steps);
    }

    protected IRobotTask createClawAngleTask(Robot robot, double position, int duration, String name, boolean steps) {

        return new CallBackTask(new CallBackTask.CallBackListener() {
            @Override
            public void setPosition(double value) {
                robot.setClawAnglePosition(value);
            }

            @Override
            public double getPosition() {
                return position;
            }
        }, position, duration, name, steps);
    }

    protected IRobotTask createClawTask(Robot robot, double position, int duration, String name, boolean steps) {

        return new CallBackTask(new CallBackTask.CallBackListener() {
            @Override
            public void setPosition(double value) {
                robot.setClawPosition(value);
            }

            @Override
            public double getPosition() {
                return position;
            }
        }, position, duration, name, steps);
    }

    protected IRobotTask createVerticalSlideTask(Robot robot, int position, int duration, String name, boolean steps) {

        return new CallBackTask(new CallBackTask.CallBackListener() {
            @Override
            public void setPosition(double value) {
                robot.setVerticalSlideTargetPosition((int) value);

            }

            @Override
            public double getPosition() {

                return robot.getVerticalSlidePosition();
            }
        }, position, duration, name, steps);
    }

    protected IRobotTask createHorizontalSlideTask(Robot robot, int position, int duration, String name, boolean steps) {

        return new CallBackTask(new CallBackTask.CallBackListener() {
            @Override
            public void setPosition(double value) {
                robot.setHorizontalSlideTargetPosition((int) value);

            }

            @Override
            public double getPosition() {

                return robot.getHorizontalSlidePosition();
            }
        }, position, duration, name, steps);
    }

    protected IRobotTask createClawSlideTask(Robot robot, int position, int duration, String name, boolean steps) {

        return new CallBackTask(new CallBackTask.CallBackListener() {
            @Override
            public void setPosition(double value) {
                robot.setDualSlideTargetPosition((int) value);

            }

            @Override
            public double getPosition() {

                return robot.getDualSlideTargetPosition();
            }
        }, position, duration, name, steps);
    }



    protected IRobotTask createIntakeClawTask(Robot robot, double position, int duration, String name, boolean steps) {

        return new CallBackTask(new CallBackTask.CallBackListener() {
            @Override
            public void setPosition(double value) {
                robot.setIntakeClawServo(value);
            }

            @Override
            public double getPosition() {
                return robot.getIntakeClawServo();
            }
        }, position, duration, name, steps);
    }


    protected IRobotTask createIntakeClawAngleTask(Robot robot, double position, int duration, String name, boolean steps) {

        return new CallBackTask(new CallBackTask.CallBackListener() {
            @Override
            public void setPosition(double value) {
                robot.setIntakeAngleServo(value);
            }

            @Override
            public double getPosition() {
                return robot.getIntakeAngleServo();
            }
        }, position, duration, name, steps);
    }

    protected IRobotTask createIntakeRotationTask(Robot robot, double position, int duration, String name, boolean steps) {

        return new CallBackTask(new CallBackTask.CallBackListener() {
            @Override
            public void setPosition(double value) {
                robot.setIntakeRotationServo(value);
            }

            @Override
            public double getPosition() {
                return robot.getIntakeRotationServo();
            }
        }, position, duration, name, steps);
    }

    protected IRobotTask createIntakeKnuckleTask(Robot robot, double position, int duration, String name, boolean steps) {

        return new CallBackTask(new CallBackTask.CallBackListener() {
            @Override
            public void setPosition(double value) {
                robot.setIntakeKnuckleServo(value);
            }

            @Override
            public double getPosition() {
                return robot.getIntakeKnuckleServo();
            }
        }, position, duration, name, steps);
    }




    @Override
    public void execute(Robot robot, Telemetry telemetry) {

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



}
