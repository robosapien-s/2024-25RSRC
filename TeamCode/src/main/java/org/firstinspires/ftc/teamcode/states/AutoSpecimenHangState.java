package org.firstinspires.ftc.teamcode.states;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.AutoCycleTask;
import org.firstinspires.ftc.teamcode.controllers.IRobotTask;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class AutoSpecimenHangState extends SpecimenHangState{

    public AutoSpecimenHangState(JoystickWrapper joystick) {
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
                        return joystick.gamepad1GetY();
                    }
                }, Robot.pathChains.remove(0)
        );
    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {
        if (joystick.gamepad1GetY()) {
            robot.setDriveTrainEnabled(true);
            robot.switchState(State.SPECIMEN_HANG);
        }


        executeTasks(telemetry);
        if (taskArrayList.isEmpty() /*joystick.gamepad1GetB()*/) {
            robot.switchState(State.AUTO_WALLPICKUP);
        }
    }

    @Override
    public State getState() {
        return State.AUTO_SPECIMEN_HANG;
    }
}
