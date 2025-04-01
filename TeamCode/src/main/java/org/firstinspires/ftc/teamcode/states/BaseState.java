package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.CallBackTask;
import org.firstinspires.ftc.teamcode.controllers.IRobotTask;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

import java.util.ArrayList;

public abstract class BaseState implements IRobot {

    protected final JoystickWrapper joystick;
    ArrayList<IRobotTask> taskArrayList = new ArrayList<IRobotTask>();

    protected BaseState(JoystickWrapper joystick) {
        this.joystick = joystick;
    }

    @Override
     public void dispose(Robot robot){}

    public static IRobotTask createWaitTask(Robot robot, int duration, String name) {

        return new CallBackTask(new CallBackTask.CallBackListener() {
            @Override
            public void setPosition(double value) {
            }

            @Override
            public double getPosition() {
                return 0;
            }
        }, 0, duration, name, false);
    }

    public static IRobotTask createRotationAndAngleTask(Robot robot, double[] positions, int duration, String name, boolean steps) {

        return new CallBackTask(new CallBackTask.CallBackListener() {
            @Override
            public void setPosition(double value) {
                robot.setRotAndAnglePosition(positions);
            }

            @Override
            public double getPosition() {
                return 0;
            }
        }, 0, duration, name, steps);
    }


    public static IRobotTask createIntakeAngleServoTask(Robot robot, double position, int duration, String name, boolean steps) {

        return new CallBackTask(new CallBackTask.CallBackListener() {
            @Override
            public void setPosition(double value) {
                robot.setIntakeAnglePosition(value);
            }

            @Override
            public double getPosition() {
                return position;
            }
        }, position, duration, name, steps);
    }


    public static IRobotTask createClawTask(Robot robot, double position, int duration, String name, boolean steps) {

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

    public static IRobotTask createSlideTask(Robot robot, int position, int duration, String name, boolean steps) {

        return new CallBackTask(new CallBackTask.CallBackListener() {
            @Override
            public void setPosition(double value) {
                robot.setSlideTargetPosition((int) value);

            }

            @Override
            public double getPosition() {

                return robot.getSlidePosition();
            }
        }, position, duration, name, steps);
    }

    public static IRobotTask createSlideRotationTask(Robot robot, int position, int duration, String name, boolean steps) {

        return new CallBackTask(new CallBackTask.CallBackListener() {
            @Override
            public void setPosition(double value) {
                robot.setSlideRotationTargetPosition((int) value);

            }

            @Override
            public double getPosition() {

                return robot.getSlideRotationPosition();
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
                taskArrayList.get(0).dispose();
                taskArrayList.remove(0);
            }
        }
    }

    @Override
    public void start(Robot robot, Telemetry telemetry) {

    }




}
