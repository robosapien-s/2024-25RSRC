package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.ExecuteOnceTask;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.robot.MultiColorSampleDetector;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class AutoPickupState extends BaseState {




    MultiColorSampleDetector colorSampleDetector;

    public AutoPickupState(JoystickWrapper joystick) {
        super(joystick);
    }

    @Override
    public void initialize(Robot robot, IRobot prevState) {

        colorSampleDetector = robot.createColorSampleDetector(MultiColorSampleDetector.ClosestSamplePipeline.SampleColorPriority.all);
        if(prevState == null) {
            //use robot.set directly, not a task series
            robot.setSlideTargetPosition(0);
            robot.setSlideRotationPosition(RoboSapiensTeleOp.Params.SLIDE_ROTATION_CAMERA_POSITION);
            robot.setRotAndAnglePosition(RoboSapiensTeleOp.Params.ROT_AND_ANGLE_CAMERA);
            robot.setClawPosition(RoboSapiensTeleOp.Params.CLAW_OPEN);

        }
    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {

        if (joystick.gamepad1GetX()) {


        }
        else if (joystick.gamepad1GetA()) {
            robot.switchState(State.INTAKINGCLAW);
        }

        executeTasks(telemetry);

    }

    @Override
    public State getState() {
        return State.INTAKINGCLAW;
    }

    @Override
    public void dispose() {
        colorSampleDetector.stopStreaming();
        colorSampleDetector = null;
    }
}
