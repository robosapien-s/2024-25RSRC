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
public class rightSideAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        JoystickWrapper joystickWrapper = new JoystickWrapper(gamepad1, gamepad2);
        RobotAuto robotAuto = new RobotAuto(hardwareMap, gamepad1, gamepad2, telemetry);
        Pose2d initPose = new Pose2d(8,-63.5,Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        TrajectoryActionBuilder trajectory1 = drive.actionBuilder(initPose)
                .stopAndAdd(robotAuto.setState(IRobot.State.SPECIMEN_HANG))
                .lineToY(-42.5)
                .waitSeconds(.1)
                .stopAndAdd(robotAuto.openTopClawAction())
                .waitSeconds(.5)
                .stopAndAdd(robotAuto.specimenHangSubstate1())
                .stopAndAdd(robotAuto.setState(IRobot.State.INTAKINGCLAW))
                .lineToY(-44)
                .strafeToLinearHeading(new Vector2d(26.25,-44), Math.toRadians(30))
                .stopAndAdd(new ParallelAction(
                                        robotAuto.setHorizontalSlidePosition(1650),
                                        robotAuto.setIntakeRotationServo(RoboSapiensTeleOp.Params.INTAKE_ROT_SERVO_DEFAULT-.16)
                                )
                )
                .waitSeconds(1)
                .stopAndAdd(robotAuto.setState(IRobot.State.PICKUP_GROUND))
                .waitSeconds(.75)
                .strafeToLinearHeading(new Vector2d(26.5, -45), Math.toRadians(-60))
                .stopAndAdd(robotAuto.setState(IRobot.State.INTAKINGCLAW))
                .waitSeconds(.1)
                .stopAndAdd(robotAuto.setIntakeRotationServo(RoboSapiensTeleOp.Params.INTAKE_ROT_SERVO_DEFAULT-.16))
                .strafeToLinearHeading(new Vector2d(34,-43), Math.toRadians(30))
                .waitSeconds(.1)
                .stopAndAdd(robotAuto.setState(IRobot.State.PICKUP_GROUND))
                .waitSeconds(.5)
                .strafeToLinearHeading(new Vector2d(34, -44), Math.toRadians(-60))
                .stopAndAdd(robotAuto.setState(IRobot.State.INTAKINGCLAW))
                .waitSeconds(.1)
                .stopAndAdd(robotAuto.setIntakeRotationServo(RoboSapiensTeleOp.Params.INTAKE_ROT_SERVO_DEFAULT-.16))
                .strafeToLinearHeading(new Vector2d(42,-41.5), Math.toRadians(30))
                .waitSeconds(.1)
                .stopAndAdd(robotAuto.setState(IRobot.State.PICKUP_GROUND))
                .waitSeconds(.5)
                .stopAndAdd(robotAuto.setHorizontalSlidePosition(1150))
                .strafeToLinearHeading(new Vector2d(42,-44), Math.toRadians(-60))
                .stopAndAdd(robotAuto.setState(IRobot.State.INTAKINGCLAW))
                .waitSeconds(.2)
                .stopAndAdd(robotAuto.setHorizontalSlidePosition(0));

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
