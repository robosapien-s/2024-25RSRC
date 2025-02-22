package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
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
@Config
@Autonomous
public class rightSideAutoPush extends LinearOpMode {

    public static double pickUpWallX = 33.75;

    @Override
    public void runOpMode() throws InterruptedException {
        JoystickWrapper joystickWrapper = new JoystickWrapper(gamepad1, gamepad2);
        RobotAuto robotAuto = new RobotAuto(hardwareMap, gamepad1, gamepad2, telemetry);
        Pose2d initPose = new Pose2d(-4.4,-63.5,Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        TrajectoryActionBuilder trajectory1 = drive.actionBuilder(initPose)
                .stopAndAdd(robotAuto.setState(IRobot.State.SPECIMEN_HANG))
                .strafeToConstantHeading(new Vector2d(-2, -42))
                .waitSeconds(.05)
//                .stopAndAdd(robotAuto.openTopClawAction())
//                .waitSeconds(.1)
//                .stopAndAdd(robotAuto.specimenHangSubstate1())
                .stopAndAdd(robotAuto.setState(IRobot.State.INTAKINGCLAW))
//                .lineToY(-44)
                .strafeToConstantHeading(new Vector2d(36, -50))

                .strafeToConstantHeading(new Vector2d(36, -15))
                .strafeToConstantHeading(new Vector2d(45, -15))
                .strafeToConstantHeading(new Vector2d(45, -57))//first push back

                .strafeToConstantHeading(new Vector2d(45, -15))
                .strafeToConstantHeading(new Vector2d(57, -15))
                .strafeToConstantHeading(new Vector2d(57, -57))//second push back

//                .strafeToConstantHeading(new Vector2d(60, -12))
//                .strafeToConstantHeading(new Vector2d(66, -12))
//                .strafeToConstantHeading(new Vector2d(48, -55))//third push back

                .stopAndAdd(robotAuto.setState(IRobot.State.WALLPICKUP))
                .strafeToLinearHeading(new Vector2d(32,-56),Math.toRadians(110))
//                .stopAndAdd(robotAuto.setClawHorizontalAnglePosition(RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_LEFT))
                .waitSeconds(.1)
                .strafeToLinearHeading(new Vector2d(34.25,-63),Math.toRadians(110))
                .waitSeconds(.05)
                .stopAndAdd(robotAuto.setState(IRobot.State.SPECIMEN_HANG))
                .waitSeconds(.2)
                .strafeToConstantHeading(new Vector2d(7,-40.25))//1st drop
                .waitSeconds(.2)
                .stopAndAdd(robotAuto.setState(IRobot.State.WALLPICKUP))
                .strafeToConstantHeading(new Vector2d(33.65,-63.25))
                .waitSeconds(.1)
                .stopAndAdd(robotAuto.setState(IRobot.State.SPECIMEN_HANG))
                .waitSeconds(.25)
                .strafeToConstantHeading(new Vector2d(8.6,-40.25))//2nd drop
                .waitSeconds(.2)
                .stopAndAdd(robotAuto.setState(IRobot.State.WALLPICKUP))
                .strafeToConstantHeading(new Vector2d(35,-63.25))
                .waitSeconds(.1)
                .stopAndAdd(robotAuto.setState(IRobot.State.SPECIMEN_HANG))
                .waitSeconds(.25)
                .strafeToConstantHeading(new Vector2d(10.35,-40.25))//3rd drop
                .waitSeconds(.2)
                .stopAndAdd(robotAuto.setState(IRobot.State.WALLPICKUP))
                .strafeToConstantHeading(new Vector2d(34.5,-63));

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
