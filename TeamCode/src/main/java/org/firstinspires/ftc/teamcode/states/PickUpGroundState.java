package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class PickUpGroundState extends BaseState {


    public PickUpGroundState(JoystickWrapper joystick) {
        super(joystick);
    }

    @Override
    public void initialize(Robot robot, IRobot prevState) {
        RobotTaskSeries transferSeries = new RobotTaskSeries();
        transferSeries.add(BaseState.createIntakeClawAngleTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_PICKUP, 1, "IntakeAngle", false));
        transferSeries.add(BaseState.createIntakeKnuckleTask(robot, RoboSapiensTeleOp.Params.INTAKE_KNUCKLE_PICKUP, 50, "KnucklePickUp", false));
        transferSeries.add(BaseState.createIntakeClawTask(robot, RoboSapiensTeleOp.Params.INTAKE_CLAW_CLOSE, 250, "IntakeClawClose", false));
        //transferSeries.add(createHorizontalSlideTask(robot, DriveTest.Params.HORIZONTAL_SLIDE_TRANSFER_POSITION, 1, "IntakeClawClose", false));
        transferSeries.add(BaseState.createIntakeRotationTask(robot, RoboSapiensTeleOp.Params.INTAKE_ROT_SERVO_DEFAULT, 1, "IntakeClawClose", false));
        transferSeries.add(BaseState.createIntakeClawAngleTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_READY, 1, "CLAW_ANGLE_BACK", false));
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
