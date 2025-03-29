package org.firstinspires.ftc.teamcode.states;

import com.acmerobotics.roadrunner.Pose2d;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.DriveToPointTask;
import org.firstinspires.ftc.teamcode.controllers.ExecuteOnceTask;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskParallel;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class WallPickUpState extends BaseState {


    public static Pose2d _lastPose = null;

    boolean angle_ready = false;
    boolean isDriftModeEnabled = false;
    public WallPickUpState(JoystickWrapper joystick) {
        super(joystick);
    }

    @Override
    public void initialize(Robot robot, IRobot prevState) {

        RobotTaskSeries transferParallel = new RobotTaskSeries();


        transferParallel.add(
                new ExecuteOnceTask(
                        new ExecuteOnceTask.ExecuteListener() {
                            @Override
                            public void execute() {
                                angle_ready = false;
                            }
                        }, "Substate Transition"
                )
        );

        //taskArrayList.add(createClawHorizontalAngleTask(robot, RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER,1,"ClawHorizontalAngle",false));

        if(prevState.getState() == State.INTAKINGCLAW) {
            //use transferParallel.add();
        } else if (prevState.getState() == State.SPECIMEN_HANG) {



            RobotTaskSeries transferMainSeries = new RobotTaskSeries();

            RobotTaskParallel subParallel = new RobotTaskParallel();





            transferMainSeries.add(subParallel);


            if(_lastPose != null){
//                transferMainSeries.add(createWaitTask(robot , 200, "Wait for final move" ));
                transferMainSeries.add(new DriveToPointTask(robot, new Vector3D(_lastPose.position.x, _lastPose.position.y-1.2, _lastPose.heading.toDouble()), 2000, 1, 0));
            }



            transferParallel.add(transferMainSeries);


        } else {
           //use transferParalle.add();
        }


        taskArrayList.add(transferParallel);
    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {

        if(joystick.gamepad1GetDUp()) {


        }



        if(joystick.gamepad1GetRightBumperRaw()) {
            robot.increaseSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger()*-100));
        } else {
            robot.increaseSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger()*100));
        }

//        if (joystick.gamepad1GetDLeft()) {
//            robot.setClawHorizontalAnglePosition(RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_LEFT);
//        } else if (joystick.gamepad1GetDRight()) {
//            robot.setClawHorizontalAnglePosition(RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_RIGHT);
//        } else if (joystick.gamepad1GetDDown()) {
//            robot.setClawHorizontalAnglePosition(RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER);
//        }

        if (angle_ready)
            robot.autoHorizontalPosWall(telemetry);

        telemetry.addData("angle_ready", angle_ready);

        executeTasks(telemetry);
    }

    @Override
    public State getState() {
        return State.WALLPICKUP;
    }
}
