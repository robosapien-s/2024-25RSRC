package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

@Autonomous
public class leftSideAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        JoystickWrapper joystickWrapper = new JoystickWrapper(gamepad1, gamepad2);
        RobotAuto robotAuto = new RobotAuto(hardwareMap, gamepad1, gamepad2, telemetry);
        Pose2d initPose = new Pose2d(-40,-62.5,Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        TrajectoryActionBuilder trajectory1 = drive.actionBuilder(initPose)
                .stopAndAdd(robotAuto.setState(IRobot.State.DROPPING_L2))
                .strafeToLinearHeading(new Vector2d(-60, -55), Math.toRadians(72))
                .stopAndAdd(robotAuto.setHorizontalSlidePosition(RoboSapiensTeleOp.Params.HORIZONTAL_SLIDE_MAX_POSITION))
                .waitSeconds(1.5)
                .stopAndAdd(robotAuto.openTopClawAction())
                .waitSeconds(.2)
                .stopAndAdd(robotAuto.setState(IRobot.State.INTAKINGCLAW))
                .strafeToConstantHeading(new Vector2d(-57.25,-54))
                .waitSeconds(.1)
                .stopAndAdd(robotAuto.setState(IRobot.State.PICKUP_GROUND_LEFT)) //first
                .waitSeconds(3.1)
                .stopAndAdd(robotAuto.setState(IRobot.State.DROPPING_L2))
                .waitSeconds(0.5)
                .stopAndAdd(robotAuto.setHorizontalSlidePosition(RoboSapiensTeleOp.Params.HORIZONTAL_SLIDE_MAX_POSITION))
                .strafeToLinearHeading(new Vector2d(-59.75, -56), Math.toRadians(93.5))
                .waitSeconds(2)
                .stopAndAdd(robotAuto.openTopClawAction())
                .waitSeconds(.2)
//                .stopAndAdd(robotAuto.setState(IRobot.State.INTAKINGCLAW))
//                .waitSeconds(.2)
                .strafeToLinearHeading(new Vector2d(-59.25, -55), Math.toRadians(90))
                .stopAndAdd(robotAuto.setState(IRobot.State.PICKUP_GROUND_LEFT))
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(-59.75, -56), Math.toRadians(93.5))//second
                .waitSeconds(2.6)
                .stopAndAdd(robotAuto.setState(IRobot.State.DROPPING_L2))
                .waitSeconds(.5)
                .stopAndAdd(robotAuto.setHorizontalSlidePosition(1425))
                .waitSeconds(2)
                .stopAndAdd(robotAuto.openTopClawAction())
                .waitSeconds(.2)
                .strafeToLinearHeading(new Vector2d(-52, -45), Math.toRadians(132))
                .stopAndAdd(robotAuto.setIntakeRotationServo(RoboSapiensTeleOp.Params.INTAKE_ROT_SERVO_DEFAULT+.16))
                .waitSeconds(.2)
                .stopAndAdd(robotAuto.setState(IRobot.State.PICKUP_GROUND_LEFT))//third
                .waitSeconds(3.1)
                .strafeToLinearHeading(new Vector2d(-61.5, -57), Math.toRadians(90))
                .waitSeconds(0.5)
                .stopAndAdd(robotAuto.setState(IRobot.State.DROPPING_L2))
                .waitSeconds(2.5)
                .stopAndAdd(robotAuto.openTopClawAction())
                .waitSeconds(0.5)
                .stopAndAdd(robotAuto.setState(IRobot.State.INTAKINGCLAW))
                .strafeToLinearHeading(new Vector2d(-40, -10), 0)
                .stopAndAdd(robotAuto.setVerticalSlidePosition(791))
                .strafeToConstantHeading(new Vector2d(-23,-10));

        TrajectoryActionBuilder smallWait = drive.actionBuilder(initPose)
                .waitSeconds(.1);

        TrajectoryActionBuilder mediumWait = drive.actionBuilder(initPose)
                .waitSeconds(.5);

//        TrajectoryActionBuilder pickUpGround1

        boolean isClawClosed = false;
        while (!isStopRequested() && !opModeIsActive()) {
            if (joystickWrapper.gamepad1GetA()) {
                if (!isClawClosed) {
                    robotAuto.closeTopClaw();
                } else {
                    robotAuto.openTopClaw();
                }
                isClawClosed = !isClawClosed;
            }
        }
        waitForStart();

        if (isStopRequested()) return;

        Action trajectory = trajectory1.build();
        Action smallWaitAction = smallWait.build();
    Action meduimWaitAction = mediumWait.build();

        Actions.runBlocking(
                new ParallelAction(
                        robotAuto.robotExecute(),
                        trajectory
//                        new SequentialAction(
//                                new ParallelAction(
//                                        robotAuto.setState(IRobot.State.SPECIMEN_HANG),

//                                ),
//                                trajectory,
//                                smallWaitAction,
//                                robotAuto.openTopClawAction(),
//                                meduimWaitAction,
//                                robotAuto.specimenHangSubstate1()

//                        )
                )
        );
    }
}
