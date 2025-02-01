package org.firstinspires.ftc.teamcode.auto;

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

@Autonomous
public class rightSideAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        JoystickWrapper joystickWrapper = new JoystickWrapper(gamepad1, gamepad2);
        RobotAuto robotAuto = new RobotAuto(hardwareMap, gamepad1, gamepad2, telemetry);
        Pose2d initPose = new Pose2d(8,-63,Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        TrajectoryActionBuilder trajectory1 = drive.actionBuilder(initPose)
                .stopAndAdd(robotAuto.setState(IRobot.State.SPECIMEN_HANG))
                .strafeToConstantHeading(new Vector2d(5.5, -43))
                .waitSeconds(.1)
                .stopAndAdd(robotAuto.openTopClawAction())
                .waitSeconds(.2)
                .stopAndAdd(robotAuto.specimenHangSubstate1())
                .stopAndAdd(robotAuto.setState(IRobot.State.INTAKINGCLAW))
//                .lineToY(-44)
                .strafeToLinearHeading(new Vector2d(26.25,-44), Math.toRadians(30))
                .stopAndAdd(new ParallelAction(
                                        robotAuto.setHorizontalSlidePosition(1650),
                                        robotAuto.setIntakeRotationServo(RoboSapiensTeleOp.Params.INTAKE_ROT_SERVO_DEFAULT-.16)
                                )
                )
                .waitSeconds(.75)
                .stopAndAdd(robotAuto.setState(IRobot.State.PICKUP_GROUND)) //first
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(26.25, -45), Math.toRadians(-60))
                .stopAndAdd(robotAuto.setState(IRobot.State.INTAKINGCLAW))
                .waitSeconds(.2)
                .stopAndAdd(robotAuto.setIntakeRotationServo(RoboSapiensTeleOp.Params.INTAKE_ROT_SERVO_DEFAULT-.16))
                .strafeToLinearHeading(new Vector2d(33,-42.5), Math.toRadians(30))
//                .waitSeconds(.2)
                .stopAndAdd(robotAuto.setState(IRobot.State.PICKUP_GROUND)) //second
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(33, -44), Math.toRadians(-60))
                .stopAndAdd(robotAuto.setState(IRobot.State.INTAKINGCLAW))
                .waitSeconds(.2)
                .stopAndAdd(robotAuto.setIntakeRotationServo(RoboSapiensTeleOp.Params.INTAKE_ROT_SERVO_DEFAULT-.16))
                .strafeToLinearHeading(new Vector2d(41.5,-42.5), Math.toRadians(30))
//                .waitSeconds(.2)
                .stopAndAdd(robotAuto.setState(IRobot.State.PICKUP_GROUND)) //third
                .waitSeconds(.7)
                .stopAndAdd(robotAuto.setHorizontalSlidePosition(1150))
                .strafeToLinearHeading(new Vector2d(41,-44), Math.toRadians(-70))
                .stopAndAdd(robotAuto.setState(IRobot.State.INTAKINGCLAW))
                .waitSeconds(.2)
                .stopAndAdd(robotAuto.setState(IRobot.State.WALLPICKUP))
                .strafeToLinearHeading(new Vector2d(34,-56),Math.toRadians(110))
//                .stopAndAdd(robotAuto.setClawHorizontalAnglePosition(RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_LEFT))
                .waitSeconds(.2)
                .strafeToLinearHeading(new Vector2d(33,-62.5),Math.toRadians(110))
                .waitSeconds(.1)
                .stopAndAdd(robotAuto.setState(IRobot.State.SPECIMEN_HANG))
                .waitSeconds(.3)
                .strafeToConstantHeading(new Vector2d(8.5,-41.5))//1st drop
                .waitSeconds(.5)
                .stopAndAdd(robotAuto.setState(IRobot.State.WALLPICKUP))
                .strafeToConstantHeading(new Vector2d(33,-60.75))
                .waitSeconds(.1)
                .stopAndAdd(robotAuto.setState(IRobot.State.SPECIMEN_HANG))
                .waitSeconds(.3)
                .strafeToConstantHeading(new Vector2d(9,-41.5))//2nd drop
                .waitSeconds(.5)
                .stopAndAdd(robotAuto.setState(IRobot.State.WALLPICKUP))
                .strafeToConstantHeading(new Vector2d(33,-61))
                .waitSeconds(.1)
                .stopAndAdd(robotAuto.setState(IRobot.State.SPECIMEN_HANG))
                .waitSeconds(.3)
                .strafeToConstantHeading(new Vector2d(9.5,-41.5))//3rd drop
                .waitSeconds(.5)
                .stopAndAdd(robotAuto.setState(IRobot.State.WALLPICKUP))
                .strafeToConstantHeading(new Vector2d(33,-61))
                .waitSeconds(.1)
                .stopAndAdd(robotAuto.setState(IRobot.State.SPECIMEN_HANG))
                .waitSeconds(.3)
                .strafeToConstantHeading(new Vector2d(10,-41.5))
                .waitSeconds(.5)
                .stopAndAdd(robotAuto.setState(IRobot.State.WALLPICKUP))
                .strafeToConstantHeading(new Vector2d(33,-61));//4rd drop

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
