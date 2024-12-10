package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.CallBackTask;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskParallel;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.opmodes.DriveTest;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class WallPickUpState extends BaseState {

    public WallPickUpState(JoystickWrapper joystick) {
        super(joystick);
    }

    @Override
    public void initialize(Robot robot, IRobot prevState) {

        RobotTaskParallel transferParallel = new RobotTaskParallel();

        transferParallel.add(createClawTask(robot, DriveTest.Params.CLAW_OPEN, 1, "Claw", false));
        transferParallel.add(createHorizontalSlideTask(robot, DriveTest.Params.HORIZONTAL_SLIDE_TRANSFER_POSITION, 1000, "Horizontal", false));
        transferParallel.add(createVerticalSlideTask(robot, DriveTest.Params.VERTICAL_SLIDE_WALL_POSITION, 1000, "Vertical", false));
        transferParallel.add(createClawSlideTask( robot, DriveTest.Params.CLAW_SLIDER_BACK, 1000, "ClawSlide", false));
        transferParallel.add(createClawAngleTask( robot, DriveTest.Params.CLAW_ANGLE_BACK, 1000, "ClawAngle", false));
        transferParallel.add(createClawRotationTask( robot, DriveTest.Params.ROT_SERVO_BACK, 1000, "ClawRotation", false));

        taskArrayList.add(transferParallel);
    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {

        if(joystick.gamepad1GetA()) {
            robot.switchState(State.SPECIMEN_HANG);
        } else if(joystick.gamepad1GetB())  {
            robot.switchState(State.INTAKINGCLAW);
        }
        executeTasks(telemetry);
    }

    @Override
    public State getState() {
        return State.WALLPICKUP;
    }
}
