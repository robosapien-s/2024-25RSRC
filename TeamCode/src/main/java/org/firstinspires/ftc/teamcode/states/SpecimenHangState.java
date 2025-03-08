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
            //use taskArrayList.add();

        } else {


            RobotTaskSeries transferMainSeries = new RobotTaskSeries();

            RobotTaskParallel subParallel = new RobotTaskParallel();




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
            robot.switchState(State.INTAKINGCLAW);
        } else if(joystick.gamepad1GetB()) {

            robot.switchState(State.WALLPICKUP);

        } else if(joystick.gamepad1GetY()) {

            robot.switchState(State.DROPPING_L1);
        }


        if (angle_ready)
            robot.autoHorizontalPosHang();


        if(joystick.gamepad1GetRightBumperRaw()) {
            robot.increaseSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger()*-100));
        } else {
            robot.increaseSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger()*100));
        }

        executeTasks(telemetry);
    }

    @Override
    public State getState() {
        return State.SPECIMEN_HANG;
    }
}
