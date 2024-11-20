package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;

public class WallPickUpState implements IRobot {

    private enum Step {
        CLOSE_CLAW,
        LIFT_CLAW,
        ROTATE_AND_ANGLE,
        LIFT_VERTICAL_SLIDE,
        MOVE_CLAW_SLIDE_FORWARD,
        OPEN_CLAW,
        RETURN_TO_ORIGINAL,
        COMPLETE
    }

    private Step currentStep = Step.CLOSE_CLAW;

    private long stepStartTime = 0;
    private static final long STEP_DELAY_MS = 500;

    @Override
    public void initialize(Robot robot) {
        currentStep = Step.CLOSE_CLAW;
        stepStartTime = System.currentTimeMillis();
    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {
        long elapsedTime = System.currentTimeMillis() - stepStartTime;

        switch (currentStep) {
            case CLOSE_CLAW:
                robot.setClawPosition(Robot.CLAW_CLOSE);
                if (elapsedTime >= STEP_DELAY_MS) {
                    stepStartTime = System.currentTimeMillis();
                    currentStep = Step.LIFT_CLAW;
                }
                break;

            case LIFT_CLAW:
                robot.setVerticalSlideTargetPosition(500);
                if (elapsedTime >= STEP_DELAY_MS) {
                    stepStartTime = System.currentTimeMillis();
                    currentStep = Step.ROTATE_AND_ANGLE;
                }
                break;

            case ROTATE_AND_ANGLE:
                robot.setClawRotationPosition(0.5);
                robot.setClawAnglePosition(Robot.CLAW_ANGLE_FORWARD);
                if (elapsedTime >= STEP_DELAY_MS) {
                    stepStartTime = System.currentTimeMillis();
                    currentStep = Step.LIFT_VERTICAL_SLIDE;
                }
                break;

            case LIFT_VERTICAL_SLIDE:
                robot.setVerticalSlideTargetPosition(2000);
                if (elapsedTime >= STEP_DELAY_MS) {
                    stepStartTime = System.currentTimeMillis();
                    currentStep = Step.MOVE_CLAW_SLIDE_FORWARD;
                }
                break;

            case MOVE_CLAW_SLIDE_FORWARD:
                robot.setClawSlideTargetPosition(Robot.CLAW_SLIDER_FORWARD);
                if (elapsedTime >= STEP_DELAY_MS) {
                    stepStartTime = System.currentTimeMillis();
                    currentStep = Step.OPEN_CLAW;
                }
                break;

            case OPEN_CLAW:
                robot.setClawPosition(Robot.CLAW_OPEN);
                if (elapsedTime >= STEP_DELAY_MS) {
                    stepStartTime = System.currentTimeMillis();
                    currentStep = Step.RETURN_TO_ORIGINAL;
                }
                break;

            case RETURN_TO_ORIGINAL:
                robot.setClawSlideTargetPosition(Robot.CLAW_SLIDER_DOWN);
                robot.setVerticalSlideTargetPosition(0);
                robot.setClawRotationPosition(Robot.ROT_SERVO_DEFAULT);
                robot.setClawAnglePosition(Robot.CLAW_ANGLE_DOWN);
                if (elapsedTime >= STEP_DELAY_MS) {
                    currentStep = Step.COMPLETE;
                }
                break;

            case COMPLETE:
                robot.switchState(State.INITIAL);
                break;
        }
    }

    @Override
    public State getState() {
        return State.WALLPICKUP;
    }
}
