package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.ExecuteOnceTask;
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

//        transferSeries.add(createIntakeClawTask(robot, RoboSapiensTeleOp.Params.INTAKE_CLAW_OPEN, 1, "INTAKE_CLAW_OPEN", false));
//        transferSeries.add(createIntakeRotationTask(robot, RoboSapiensTeleOp.Params.INTAKE_ROT_SERVO_DEFAULT, 1, "INTAKE_ROT_SERVO_DEFAULT", false));
        transferSeries.add(createClawHorizontalAngleTask(robot, RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER,1,"CLAW_HORIZONTAL_ANGLE_CENTER", false));
        transferSeries.add(createClawAngleTask(robot, RoboSapiensTeleOp.Params.CLAW_ANGLE_FORWARD, 1, "CLAW_ANGLE_BACK", false));
        transferSeries.add(createClawRotationTask(robot,RoboSapiensTeleOp.Params.ROT_SERVO_DEFAULT, 0, "CLAW_ROT_DEFAULT", false));
        transferSeries.add(createClawSlideTask(robot, RoboSapiensTeleOp.Params.CLAW_SLIDER_TRANSFER, 1, "CLAW_SLIDER_TRANSFER", false));

//        transferSeries.add(createIntakeClawAngleTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_READY, 1, "CLAW_ANGLE_BACK", false));
//        transferSeries.add(createIntakeKnuckleTask(robot, RoboSapiensTeleOp.Params.INTAKE_KNUCKLE_PICKUP, 1, "INTAKE_KNUCKLE_PICKUP", false));
        transferSeries.add(createVerticalSlideTask(robot, RoboSapiensTeleOp.Params.VERTICAL_SLIDE_POSITION, 0, "VerticalSlide", false));
//        taskArrayList.add(transferSeries);


        transferSeries.add(createIntakeClawAngleTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_PICKUP, 1, "IntakeAngle", false));
        transferSeries.add(createIntakeKnuckleTask(robot, RoboSapiensTeleOp.Params.INTAKE_KNUCKLE_PICKUP, 50, "KnucklePickUp", false));
        transferSeries.add(createIntakeClawTask(robot, RoboSapiensTeleOp.Params.INTAKE_CLAW_CLOSE, 250, "IntakeClawClose", false));
        //transferSeries.add(createHorizontalSlideTask(robot, DriveTest.Params.HORIZONTAL_SLIDE_TRANSFER_POSITION, 1, "IntakeClawClose", false));
        transferSeries.add(createIntakeRotationTask(robot, RoboSapiensTeleOp.Params.INTAKE_ROT_SERVO_DEFAULT, 1, "IntakeClawClose", false));
        transferSeries.add(createIntakeClawAngleTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_READY, 100, "CLAW_ANGLE_BACK", false));

        transferSeries.add(createHorizontalSlideTask(robot, RoboSapiensTeleOp.Params.HORIZONTAL_SLIDE_TRANSFER_POSITION, 1, "Claw", false));
        transferSeries.add(createClawTask(robot, RoboSapiensTeleOp.Params.CLAW_OPEN, 1, "Claw", false));
        transferSeries.add(createClawSlideTask( robot, RoboSapiensTeleOp.Params.CLAW_SLIDER_TRANSFER, 1, "ClawSlide", false));

        transferSeries.add(createClawAngleTask(robot, RoboSapiensTeleOp.Params.CLAW_ANGLE_TRANSFER, 1, "IntakeClawOpen", false));
        transferSeries.add(createVerticalSlideTask(robot, RoboSapiensTeleOp.Params.VERTICAL_SLIDE_TRANSFER_POSITION, 1, "IntakeClawClose", false));


        transferSeries.add(createIntakeRotationTask(robot, RoboSapiensTeleOp.Params.INTAKE_ROT_SERVO_DEFAULT, (int) ((250)*(((double) robot.getHorizontalSlidePosition())/ ((double) RoboSapiensTeleOp.Params.HORIZONTAL_SLIDE_MAX_POSITION))), "IntakeClawClose", false));

        transferSeries.add(createIntakeKnuckleTask(robot, RoboSapiensTeleOp.Params.INTAKE_KNUCKLE_TRANSFER, 200, "IntakeClawClose", false));
        transferSeries.add(createIntakeClawAngleTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_TRANSFER, 300, "IntakeClawClose", false));
        transferSeries.add(createIntakeClawTask(robot, RoboSapiensTeleOp.Params.INTAKE_CLAW_LOOSE, 300, "IntakeClawLoose", false));



        //transferSeries.add(createClawSlideTask( robot, DriveTest.Params.CLAW_SLIDER_TRANSFER+800, 200, "ClawSlide", false));
        transferSeries.add(createClawTask(robot, RoboSapiensTeleOp.Params.CLAW_CLOSE, 200, "Claw", false));
        transferSeries.add(createIntakeClawTask(robot, RoboSapiensTeleOp.Params.INTAKE_CLAW_OPEN, 100, "IntakeClawOpen", false));
        transferSeries.add(createIntakeClawAngleTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_READY, 0, "IntakeAngleDown", false));
        transferSeries.add(createIntakeKnuckleTask(robot, RoboSapiensTeleOp.Params.INTAKE_KNUCKLE_PICKUP, 250, "IntakeKnuckleDown", false));

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
