package org.firstinspires.ftc.teamcode.states;

import com.acmerobotics.roadrunner.Pose2d;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.DriveToPointTask;
import org.firstinspires.ftc.teamcode.controllers.ExecuteOnceTask;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskParallel;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class SpecimenHangState extends BaseState {
    boolean angle_ready = false;

    public static Pose2d _lastPose = null;
    boolean didLowerHeight = false;
//    int subState = 0;
    public SpecimenHangState(JoystickWrapper joystick) {
        super(joystick);
    }

    @Override
    public void initialize(Robot robot, IRobot prevState) {

        taskArrayList.add(
                new ExecuteOnceTask(
                        new ExecuteOnceTask.ExecuteListener() {
                            @Override
                            public void execute() {
                                angle_ready = false;
                            }
                        }, "Substate Transition"
                )
        );

        if (prevState.getState() == State.INTAKINGCLAW) {
            taskArrayList.add(createVerticalSlideTask(robot, RoboSapiensTeleOp.Params.VERTICAL_SLIDE_HANG_PREP_POSITION, 100, "Vertical", false));
            taskArrayList.add(createClawAngleTask( robot, RoboSapiensTeleOp.Params.CLAW_ANGLE_FORWARD, 200, "ClawAngle", false));
            taskArrayList.add(createClawSlideTask( robot, RoboSapiensTeleOp.Params.CLAW_SLIDER_FORWARD, 200, "ClawSlide", false));

            RobotTaskParallel transferParallel = new RobotTaskParallel();

            //transferParallel.add(createClawAngleTask( robot, DriveTest.Params.CLAW_ANGLE_FORWARD_SPECIMEN, 1000, "ClawAngle", true));
            transferParallel.add(createClawRotationTask( robot, RoboSapiensTeleOp.Params.ROT_SERVO_DEFAULT, 1, "ClawRotation", false));
            transferParallel.add(createHorizontalSlideTask(robot, 0, 500, "Horizontal", true));

            taskArrayList.add(transferParallel);


            taskArrayList.add(
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


            RobotTaskSeries transferMainSeries = new RobotTaskSeries();

            RobotTaskParallel subParallel = new RobotTaskParallel();

            if(_lastPose != null) {
                subParallel.add(new DriveToPointTask(robot, new Vector3D(_lastPose.position.x, _lastPose.position.y-8, _lastPose.heading.toDouble()), 2000, 2, 1000));

            }


            transferMainSeries.add(createClawTask(robot, RoboSapiensTeleOp.Params.CLAW_CLOSE, 250, "Claw", false));
            transferMainSeries.add(createClawHorizontalAngleTask(robot, RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER,1,"ClawHorizontalAngle",false));
            transferMainSeries.add(createVerticalSlideTask(robot, RoboSapiensTeleOp.Params.VERTICAL_SLIDE_HANG_PREP_POSITION, 300, "Vertical", false));
            transferMainSeries.add(createClawRotationTask( robot, RoboSapiensTeleOp.Params.ROT_SERVO_DEFAULT, 1, "ClawRotation", false));
            transferMainSeries.add(createClawAngleTask( robot, RoboSapiensTeleOp.Params.CLAW_ANGLE_FORWARD, 200, "ClawAngle", false));
            transferMainSeries.add(createClawSlideTask( robot, RoboSapiensTeleOp.Params.CLAW_SLIDER_FORWARD, 0, "ClawSlide", false));
            transferMainSeries.add(createHorizontalSlideTask(robot, 0, 500, "Horizontal", true));


            transferMainSeries.add(
                    new ExecuteOnceTask(
                            new ExecuteOnceTask.ExecuteListener() {
                                @Override
                                public void execute() {
                                    angle_ready = true;
                                }
                            }, "Substate Transition"
                    )
            );


            subParallel.add(transferMainSeries);


            taskArrayList.add(subParallel);

            if(_lastPose!= null) {
                //taskArrayList.add(createWaitTask(robot, 100, "Wait for final move"));
                taskArrayList.add(new DriveToPointTask(robot, new Vector3D(_lastPose.position.x, _lastPose.position.y, _lastPose.heading.toDouble()), 2000, 1, 0));
            }






        }


    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {

        if(joystick.gamepad1GetA()) {
            //robot.setVerticalSlideTargetPosition(DriveTest.Params.VERTICAL_SLIDE_HANG_DROP_POSITION);
            robot.switchState(State.INTAKINGCLAW);
        } else if(joystick.gamepad1GetB()) {
//            if(subState == 0) {
//                subState++;
//                robot.setClawPosition(RoboSapiensTeleOp.Params.CLAW_OPEN);
//            }
//            else if(subState == 1) {
//
//                subState++;
//                angle_ready = false;
//                //robot.setVerticalSlideTargetPosition(DriveTest.Params.VERTICAL_SLIDE_HANG_DROP_POSITION);
//
//            } else {
//
//            }


            _lastPose = robot.getPose();
            _lastPose = new Pose2d(_lastPose.position.x, _lastPose.position.y, _lastPose.heading.toDouble());
            robot.switchState(State.WALLPICKUP);

        } else if(joystick.gamepad1GetY()) {

            robot.switchState(State.DROPPING_L1);
//        } else if(joystick.gamepad1GetDUp()) {
//            robot.setVerticalSlideTargetPosition(DriveTest.Params.VERTICAL_SLIDE_HANG_PREP_POSITION);
//        } else if(joystick.gamepad1GetDDown()) {
//            robot.setVerticalSlideTargetPosition(DriveTest.Params.VERTICAL_SLIDE_HANG_DROP_POSITION);
//        }
//        } else if (joystick.gamepad1GetDLeft() && subState == 0) {
//            robot.setClawHorizontalAnglePosition(RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_LEFT);
//        } else if (joystick.gamepad1GetDRight() && subState == 0) {
//            robot.setClawHorizontalAnglePosition(RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_RIGHT);
//        } else if (joystick.gamepad1GetDDown() && subState == 0) {
//            robot.setClawHorizontalAnglePosition(RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER);
        }


        if (angle_ready)
            robot.autoHorizontalPosHang();


        if(joystick.gamepad1GetLeftBumperRaw()) {
            robot.increaseClawSlideTargetPosition((int) (joystick.gamepad1GetLeftTrigger()*-500));
        } else {
            robot.increaseClawSlideTargetPosition((int) (joystick.gamepad1GetLeftTrigger()*500));
        }

        if(joystick.gamepad1GetRightBumperRaw()) {
            robot.increaseVerticalSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger()*-100));
        } else {
            robot.increaseVerticalSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger()*100));
        }

        executeTasks(telemetry);
    }

    @Override
    public State getState() {
        return State.SPECIMEN_HANG;
    }
}
