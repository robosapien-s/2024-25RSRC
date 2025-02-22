package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
public class rightSideAuto extends LinearOpMode {

    public static double firstDropY = -42;

    public static double pickup1x = 25.25;
    public static double pickup1y = -41;
//    public static double pickup1angle;

    public static double pickup2x  = 30.50;
    public static double pickup2y = -36.75;
//    public static double pickup2angle;

    public static double pickup3x = 42;
    public static double pickup3y = -37.25;
//    public static double pickup3angle;

    public static double wallPickup1x = 32.5;
    public static double wallPickup1y = -63;

    public static double wallPickup2x = 33;
    public static double wallPickup2y = -63.25;

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
                .strafeToLinearHeading(new Vector2d(pickup1x,pickup1y), Math.toRadians(30))
                .stopAndAdd(new ParallelAction(
                                        robotAuto.setHorizontalSlidePosition(1650),
                                        robotAuto.setIntakeRotationServo(RoboSapiensTeleOp.Params.INTAKE_ROT_SERVO_DEFAULT-.16)
                                )
                )
                .waitSeconds(.75)
                .stopAndAdd(robotAuto.setState(IRobot.State.PICKUP_GROUND)) //first
                .waitSeconds(.6)
                .strafeToLinearHeading(new Vector2d(26.5, -45), Math.toRadians(-54))
                .stopAndAdd(robotAuto.setState(IRobot.State.INTAKINGCLAW))
                .waitSeconds(.2)
                .stopAndAdd(robotAuto.setIntakeRotationServo(RoboSapiensTeleOp.Params.INTAKE_ROT_SERVO_DEFAULT-.16))
                .strafeToLinearHeading(new Vector2d(pickup2x,pickup2y), Math.toRadians(30))
//                .waitSeconds(.2)
                .stopAndAdd(robotAuto.setState(IRobot.State.PICKUP_GROUND)) //second
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(33, -43.5), Math.toRadians(-60))
                .stopAndAdd(robotAuto.setState(IRobot.State.INTAKINGCLAW))
                .waitSeconds(.2)
                .stopAndAdd(robotAuto.setIntakeRotationServo(RoboSapiensTeleOp.Params.INTAKE_ROT_SERVO_DEFAULT-.16))
                .strafeToLinearHeading(new Vector2d(pickup3x,pickup3y), Math.toRadians(30))
//                .waitSeconds(.2)
                .stopAndAdd(robotAuto.setState(IRobot.State.PICKUP_GROUND)) //third
                .waitSeconds(.5)
                .stopAndAdd(robotAuto.setHorizontalSlidePosition(1150))
                .strafeToLinearHeading(new Vector2d(41,-44), Math.toRadians(-70))
                .stopAndAdd(robotAuto.setState(IRobot.State.INTAKINGCLAW))
                .waitSeconds(.2)
                .stopAndAdd(robotAuto.setState(IRobot.State.WALLPICKUP))
                .strafeToLinearHeading(new Vector2d(wallPickup1x,-56),Math.toRadians(110))
//                .stopAndAdd(robotAuto.setClawHorizontalAnglePosition(RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_LEFT))
                .waitSeconds(.1)
                .strafeToLinearHeading(new Vector2d(wallPickup1x,wallPickup1y), Math.toRadians(110))
                .waitSeconds(.05)
                .stopAndAdd(robotAuto.setState(IRobot.State.SPECIMEN_HANG))
                .waitSeconds(.2)
                .strafeToConstantHeading(new Vector2d(7,-40.25))//1st drop
                .waitSeconds(.15)
                .stopAndAdd(robotAuto.setState(IRobot.State.WALLPICKUP))
                .strafeToConstantHeading(new Vector2d(wallPickup2x,wallPickup2y))
                .waitSeconds(.1)
                .stopAndAdd(robotAuto.setState(IRobot.State.SPECIMEN_HANG))
                .waitSeconds(.25)
                .strafeToConstantHeading(new Vector2d(8.6,-40.25))//2nd drop
                .waitSeconds(.15)
                .stopAndAdd(robotAuto.setState(IRobot.State.WALLPICKUP))
                .strafeToConstantHeading(new Vector2d(wallPickup2x,wallPickup2y))
                .waitSeconds(.1)
                .stopAndAdd(robotAuto.setState(IRobot.State.SPECIMEN_HANG))
                .waitSeconds(.25)
                .strafeToConstantHeading(new Vector2d(10.35,-40.25))//3rd drop
                .waitSeconds(.15)
                .stopAndAdd(robotAuto.setState(IRobot.State.WALLPICKUP))
                .strafeToConstantHeading(new Vector2d(wallPickup2x,wallPickup2y))
                .waitSeconds(.15)
                .stopAndAdd(robotAuto.setState(IRobot.State.SPECIMEN_HANG))
                .waitSeconds(.25)
                .strafeToConstantHeading(new Vector2d(11.85,-40.25))//4th drop
                .waitSeconds(.15)
                .stopAndAdd(robotAuto.setState(IRobot.State.WALLPICKUP))
                .strafeToConstantHeading(new Vector2d(33,-63));

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
