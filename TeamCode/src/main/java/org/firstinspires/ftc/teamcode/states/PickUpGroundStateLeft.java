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

        taskArrayList.add(createIntakeAngleServoTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_PICKUP, 50, "IntakeAngle", false));

        taskArrayList.add(createClawTask(robot, RoboSapiensTeleOp.Params.CLAW_CLOSE, 300, "IntakeClawClose", false));


        taskArrayList.add(createRotationAndAngleTask(robot, RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PREP, 0, "IntakeAngle", false));

        taskArrayList.add(createIntakeAngleServoTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_READY, 50, "IntakeAngle", false));




    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {
        executeTasks(telemetry);

    }

    @Override
    public State getState() {
        return State.PICKUP_GROUND_LEFT;
    }
}
