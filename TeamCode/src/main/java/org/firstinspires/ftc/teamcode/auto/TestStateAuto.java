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

@Autonomous
public class TestStateAuto extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        RobotAuto robotAuto = new RobotAuto(hardwareMap, gamepad1, gamepad2, telemetry);
        Pose2d initPose = new Pose2d(48,-60,Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        TrajectoryActionBuilder moveForward10 = drive.actionBuilder(initPose)
                .splineTo(new Vector2d(48,0),Math.toRadians(90))
                .waitSeconds(1);

        TrajectoryActionBuilder smallWait = drive.actionBuilder(initPose)
                .waitSeconds(.1);

//        while (!isStopRequested() && !opModeIsActive()) {
//            robotAuto.robotExecute();
//        }

        TrajectoryActionBuilder wait5 = drive.actionBuilder(initPose).waitSeconds(5);

        waitForStart();

        if (isStopRequested()) return;

        Action trajectory = moveForward10.build();

        Action wait5Action = wait5.build();

        Actions.runBlocking(
                new ParallelAction(
                        robotAuto.robotExecute(),
                        new ParallelAction(
                                robotAuto.setState(IRobot.State.WALLPICKUP),
                                trajectory
                        )

                )
        );


    }
}
