package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.ExecuteOnceTask;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class IntakingStateClaw extends BaseState {

    int clawRotationStateHack = 0;

    int clawStateHack = 0; //0 = ready, 1 = pickup, 2 = transfer

     double[] clawRotationPositions = new double[]{


     };
    public IntakingStateClaw(JoystickWrapper joystick) {
        super(joystick);
    }

    @Override
    public void initialize(Robot robot, IRobot prevState) {
        if(prevState == null) {
            //use robot.set directly, not a task series

        } else if (prevState.getState() == State.SPECIMEN_HANG) {
            RobotTaskSeries transferSeries = new RobotTaskSeries();

            taskArrayList.add(transferSeries);
        }else {
            RobotTaskSeries transferSeries = new RobotTaskSeries();

            taskArrayList.add(transferSeries);
        }
    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {

        if (joystick.gamepad2GetA()) {
            robot.switchState(State.SERVO_TEST);
        }




        if(joystick.gamepad1GetB()) {
            robot.setRobotSpeedNormal();
            robot.switchState(State.WALLPICKUP);
        }

        if(joystick.gamepad1GetX()) {

            robot.setRobotSpeedNormal();
            RobotTaskSeries transferSeries = new RobotTaskSeries();

            transferSeries.add( new ExecuteOnceTask(new ExecuteOnceTask.ExecuteListener() {
                @Override
                public void execute() {
                    robot.switchState(State.DROPPING_L2);
                }
            }, "Set Drop State"));

            taskArrayList.add(transferSeries);



        }
        executeTasks(telemetry);

    }

    @Override
    public State getState() {
        return State.INTAKINGCLAW;
    }
}
