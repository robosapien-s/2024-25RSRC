package org.firstinspires.ftc.teamcode.states;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.AutoCycleTask;
import org.firstinspires.ftc.teamcode.controllers.IRobotTask;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class AutoWallPickUpState extends WallPickUpState{
    public AutoWallPickUpState(JoystickWrapper joystick) {
        super(joystick);
    }

    @Override
    public IRobotTask runTrajectory(Robot robot) {
        robot.setDriveTrainEnabled(false);
        return new AutoCycleTask(
                new AutoCycleTask.AutoCycleListener() {
                    @Override
                    public Follower follower() {
                        return robot.getFollower();
                    }

                    @Override
                    public boolean breakFollowing() {
                        return joystick.gamepad1GetYRaw();
                    }
                }, Robot.pathChains.remove(0)
        );
    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {
        if (joystick.gamepad1GetY()) {
            robot.getFollower().breakFollowing();
            robot.setDriveTrainEnabled(true);
            robot.switchState(State.WALLPICKUP);
        } else if (taskArrayList.isEmpty() /*joystick.gamepad1GetB()*/) {
            if (Robot.pathChains.isEmpty()) {
                robot.setDriveTrainEnabled(true);
                robot.getFollower().breakFollowing();
                robot.switchState(State.WALLPICKUP);
            } else {
                robot.getFollower().breakFollowing();
                robot.switchState(State.AUTO_SPECIMEN_HANG);
            }
        }

        executeTasks(telemetry);
    }

    @Override
    public State getState() {
        return State.AUTO_WALLPICKUP;
    }

//    @Override
//    public void dispose(Robot robot) {
//
//    }

}
