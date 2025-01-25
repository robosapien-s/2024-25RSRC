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

        RobotTaskSeries transferParallel = new RobotTaskSeries();

        transferParallel.add(createClawTask(robot, DriveTest.Params.CLAW_OPEN, 1, "Claw", false));
        transferParallel.add(createHorizontalSlideTask(robot, 0, 500, "Horizontal", false));
        transferParallel.add(createVerticalSlideTask(robot, DriveTest.Params.VERTICAL_SLIDE_WALL_POSITION, 1000, "Vertical", false));
        transferParallel.add(createClawAngleTask( robot, DriveTest.Params.CLAW_ANGLE_PREP_BACK, 400, "ClawAngle", false));
        transferParallel.add(createClawSlideTask( robot, DriveTest.Params.CLAW_SLIDER_BACK, 1000, "ClawSlide", false));
        transferParallel.add(createClawAngleTask( robot, DriveTest.Params.CLAW_ANGLE_BACK, 400, "ClawAngle", false));
        transferParallel.add(createClawRotationTask(robot, DriveTest.Params.ROT_SERVO_BACK,100,"ClawRotation",false));
        transferParallel.add(createVerticalSlideTask(robot, DriveTest.Params.VERTICAL_SLIDE_DOWN_POSITION, 0, "Vertical", false));
        taskArrayList.add(transferParallel);
    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {

        if(joystick.gamepad1GetB()) {
            robot.switchState(State.SPECIMEN_HANG);
        } else if(joystick.gamepad1GetA())  {
            robot.switchState(State.INTAKINGCLAW);
        }

        if(joystick.gamepad1GetLeftBumperRaw()) {
            robot.increseClawSlideTargetPosition((int) (joystick.gamepad1GetLeftTrigger()*-500));
        } else {
            robot.increseClawSlideTargetPosition((int) (joystick.gamepad1GetLeftTrigger()*500));
        }

        if(joystick.gamepad1GetRightBumperRaw()) {
            robot.increseVerticalSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger()*-100));
        } else {
            robot.increseVerticalSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger()*100));
        }
        executeTasks(telemetry);
    }

    @Override
    public State getState() {
        return State.WALLPICKUP;
    }
}
