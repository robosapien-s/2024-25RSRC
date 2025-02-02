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
                .stopAndAdd(robotAuto.setState(IRobot.State.DROPPING_L1))
                .strafeToLinearHeading(new Vector2d(-60, -55), Math.toRadians(72))
                .stopAndAdd(robotAuto.setHorizontalSlidePosition(RoboSapiensTeleOp.Params.HORIZONTAL_SLIDE_MAX_POSITION))
                .waitSeconds(.5)
                .stopAndAdd(robotAuto.openTopClawAction())
                .waitSeconds(.2)
                .stopAndAdd(robotAuto.setState(IRobot.State.INTAKINGCLAW))
                .strafeToConstantHeading(new Vector2d(-57,-52))
                .waitSeconds(.1)
                .stopAndAdd(robotAuto.setState(IRobot.State.PICKUP_GROUND_LEFT))
                .waitSeconds(2.5)
                .stopAndAdd(robotAuto.setState(IRobot.State.DROPPING_L1))
                .waitSeconds(.5)
                .stopAndAdd(robotAuto.setHorizontalSlidePosition(RoboSapiensTeleOp.Params.HORIZONTAL_SLIDE_MAX_POSITION))
                .strafeToLinearHeading(new Vector2d(-61.5, -55), Math.toRadians(90))
                .waitSeconds(.5)
                .stopAndAdd(robotAuto.openTopClawAction())
                .waitSeconds(.2)
//                .stopAndAdd(robotAuto.setState(IRobot.State.INTAKINGCLAW))
//                .waitSeconds(.2)
                .stopAndAdd(robotAuto.setState(IRobot.State.PICKUP_GROUND_LEFT))
                .waitSeconds(2.5)
                .stopAndAdd(robotAuto.setState(IRobot.State.DROPPING_L1))
                .waitSeconds(1.5)
                .stopAndAdd(robotAuto.openTopClawAction())
                .waitSeconds(.2)
                .stopAndAdd(robotAuto.setState(IRobot.State.INTAKINGCLAW));

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
