package org.firstinspires.ftc.teamcode.states;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.ExecuteOnceTask;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class DroppingL1State extends BaseState {
    boolean angle_ready = false;

    public static Pose2d _lastPose = null;


//    int substate = 0;

    public DroppingL1State(JoystickWrapper joystick) {
        super(joystick);
    }

    public int getHeight() {
        return RoboSapiensTeleOp.Params.SLIDE_DROP_L1;
    }

    public int getWait() {return 1000;}
    @Override
    public void initialize(Robot robot, IRobot prevState) {
        robot.setSlideMaxPosition(getHeight());

        RobotTaskSeries transferSeries = new RobotTaskSeries();

//        transferSeries.add(new ExecuteOnceTask(
//                new ExecuteOnceTask.ExecuteListener() {
//                    @Override
//                    public void execute() {
//                        angle_ready = false;
//                    }
//                }, "Substate Transition"
//        ));
        transferSeries.add(createRotationAndAngleTask(robot, RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PREP, 0, "IntakeAngle", false));

        transferSeries.add(createIntakeAngleServoTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_BUCKET, 0, "IntakeAngle", false));

        transferSeries.add(createClawTask(robot, RoboSapiensTeleOp.Params.CLAW_CLOSE, 1, "ClawClose", false));

        transferSeries.add(createSlideTask(robot, 0, 1, "VerticalSlide", false));


//        transferSeries.add(createSlideRotationTask(robot, RoboSapiensTeleOp.Params.SLIDE_ROTATION_MIDDLE_POSITION, 100, "HorizontalSlide", false));
        transferSeries.add(createSlideRotationTask(robot, RoboSapiensTeleOp.Params.SLIDE_ROTATION_DROP_POSITION, 400, "HorizontalSlide", false));
        transferSeries.add(createSlideTask(robot, getHeight(), 600, "VerticalSlide", false));


        transferSeries.add(createRotationAndAngleTask(robot, RoboSapiensTeleOp.Params.ROT_AND_ANGLE_BASKET, 50, "IntakeAngle", false));



//        transferSeries.add(new ExecuteOnceTask(
//                new ExecuteOnceTask.ExecuteListener() {
//                    @Override
//                    public void execute() {
//                        angle_ready = true;
//                    }
//                }, "Substate Transition"
//        ));


        taskArrayList.add(transferSeries);

    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {


        if(joystick.gamepad1GetX()) {

            taskArrayList.add(createClawTask(robot, RoboSapiensTeleOp.Params.CLAW_OPEN, 300, "claw open", false));

            taskArrayList.add(new ExecuteOnceTask(
                    new ExecuteOnceTask.ExecuteListener() {
                        @Override
                        public void execute() {
                            robot.switchState(State.INTAKINGCLAW);
                        }
                    }, "Substate Transition"
            ));

        } else if(joystick.gamepad1GetA()) {

            taskArrayList.add(new ExecuteOnceTask(
                    new ExecuteOnceTask.ExecuteListener() {
                        @Override
                        public void execute() {
                            robot.switchState(State.INTAKINGCLAW);
                        }
                    }, "Substate Transition"
            ));
        }

        executeTasks(telemetry);
    }

    @Override
    public State getState() {
        return State.DROPPING_L1;
    }

}
