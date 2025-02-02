package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.ExecuteOnceTask;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class DroppingL1State extends BaseState {
    boolean angle_ready = false;


    int substate = 0;

    public DroppingL1State(JoystickWrapper joystick) {
        super(joystick);
    }

    public int getHeight() {
        return RoboSapiensTeleOp.Params.VERTICAL_SLIDE_DROP_L1;
    }
    @Override
    public void initialize(Robot robot, IRobot prevState) {

        RobotTaskSeries transferSeries = new RobotTaskSeries();

        transferSeries.add(new ExecuteOnceTask(
                new ExecuteOnceTask.ExecuteListener() {
                    @Override
                    public void execute() {
                        angle_ready = false;
                    }
                }, "Substate Transition"
        ));


        transferSeries.add(createClawTask(robot, RoboSapiensTeleOp.Params.CLAW_CLOSE, 1, "ClawClose", false));
        transferSeries.add(createHorizontalSlideTask(robot, RoboSapiensTeleOp.Params.HORIZONTAL_SLIDE_TRANSFER_POSITION, 1, "HorizontalSlide", false));
        transferSeries.add(createVerticalSlideTask(robot, getHeight(), 1, "VerticalSlide", false));
        transferSeries.add(createClawSlideTask(robot, RoboSapiensTeleOp.Params.CLAW_SLIDER_BACK, 1000, "CLAW_SLIDER_BACK", false));
        transferSeries.add(createClawAngleTask(robot, RoboSapiensTeleOp.Params.CLAW_ANGLE_BACK, 200, "CLAW_ANGLE_BACK", false));
        transferSeries.add(createClawRotationTask(robot, RoboSapiensTeleOp.Params.ROT_SERVO_BACK, 100, "ROT_SERVO_BACK", false));


        transferSeries.add(new ExecuteOnceTask(
                new ExecuteOnceTask.ExecuteListener() {
                    @Override
                    public void execute() {
                        angle_ready = true;
                    }
                }, "Substate Transition"
        ));

        taskArrayList.add(transferSeries);

    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {

        if(joystick.gamepad1GetX()) {

            if(substate == 0) {
                substate++;
                robot.setClawPosition(RoboSapiensTeleOp.Params.CLAW_OPEN);
            } else {
                angle_ready = false;
                robot.setClawHorizontalAnglePosition(RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER);
                robot.switchState(State.INTAKINGCLAW);
            }
        } else if(joystick.gamepad1GetA()) {
            angle_ready = false;
            robot.setClawHorizontalAnglePosition(RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER);
            robot.switchState(State.INTAKINGCLAW);
        }

        if (angle_ready)
            robot.autoHorizontalPosBucket(telemetry);

        executeTasks(telemetry);
    }

    @Override
    public State getState() {
        return State.DROPPING_L1;
    }

}
