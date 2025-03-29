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


        transferSeries.add(createClawTask(robot, RoboSapiensTeleOp.Params.CLAW_CLOSE, 1, "ClawClose", false));

        transferSeries.add(createVerticalSlideTask(robot, 0, 1, "VerticalSlide", false));


        transferSeries.add(createHorizontalSlideTask(robot, RoboSapiensTeleOp.Params.SLIDE_ROTATION_TRANSFER_POSITION, 200, "HorizontalSlide", false));
        transferSeries.add(createVerticalSlideTask(robot, getHeight(), 500, "VerticalSlide", false));

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
//            taskArrayList.add(new ExecuteOnceTask(
//                    new ExecuteOnceTask.ExecuteListener() {
//                        @Override
//                        public void execute() {
//                            angle_ready = false;
//                        }
//                    }, "Substate Transition"
//            ));


            taskArrayList.add(createRotationAndAngleTask(robot, RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PREP, 50, "IntakeAngle", false));


            taskArrayList.add(createVerticalSlideTask(robot, 0, 300, "vertical", false));

            taskArrayList.add(createHorizontalSlideTask(robot, 0, 200, "HorizontalSlide", false));


            taskArrayList.add(new ExecuteOnceTask(
                    new ExecuteOnceTask.ExecuteListener() {
                        @Override
                        public void execute() {
                            robot.switchState(State.INTAKINGCLAW);
                        }
                    }, "Substate Transition"
            ));

        } else if(joystick.gamepad1GetA()) {
            angle_ready = false;
            taskArrayList.add(createVerticalSlideTask(robot, 0, 300, "horizontal claw center", false));

            //taskArrayList.add(createClawHorizontalAngleTask(robot, RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER, 0, "horizontal claw center", false));

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
