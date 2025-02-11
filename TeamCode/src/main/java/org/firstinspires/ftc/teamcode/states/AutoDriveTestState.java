package org.firstinspires.ftc.teamcode.states;

import android.graphics.Point;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class AutoDriveTestState extends BaseState {


    public Vector3D storedDeadWheelLocation = new Vector3D(0,0, 0);
    public AutoDriveTestState(JoystickWrapper joystick) {
        super(joystick);
    }

      @Override
    public void initialize(Robot robot, IRobot prevState) {

    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {

        if(joystick.gamepad1GetX()) {

            storedDeadWheelLocation = robot.getDeadWheelLocation();

        } else if(joystick.gamepad1GetA()) {
            robot.setAutoTarget(storedDeadWheelLocation.getX(), storedDeadWheelLocation.getY(), storedDeadWheelLocation.getZ());
        }

        executeTasks(telemetry);
    }

    @Override
    public State getState() {
        return State.DROPPING_L1;
    }

}
