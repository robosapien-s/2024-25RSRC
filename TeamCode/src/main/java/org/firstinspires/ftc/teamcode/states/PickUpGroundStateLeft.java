package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class PickUpGroundStateLeft extends BaseState {


    public PickUpGroundStateLeft(JoystickWrapper joystick) {
        super(joystick);
    }

    @Override
    public void initialize(Robot robot, IRobot prevState) {
        RobotTaskSeries transferSeries = new RobotTaskSeries();


        taskArrayList.add(transferSeries);




    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {
        executeTasks(telemetry);

    }

    @Override
    public State getState() {
        return State.PICKUP_GROUND;
    }
}
