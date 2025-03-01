package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.*;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

import kotlin.ParameterName;

@Autonomous
public class rightSideAutoPedro extends LinearOpMode {


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose autoStartPose = new Pose(8.5, 76.5, Math.toRadians(0));

    private final Pose preloadHangPose = new Pose(30.25, 74, Math.toRadians(0));

    private final Pose pickup1Pose = new Pose(31, 47.75, Math.toRadians(-60));

    private final Pose dropoff1Pose = new Pose(29, 45.5, Math.toRadians(-144));

    private final Pose pickup2pose = new Pose(28, 42.5, Math.toRadians(-60));

    private final Pose dropoff2pose = new Pose(29, 39, Math.toRadians(-150));

    private final Pose pickup3pose = new Pose(30, 30, Math.toRadians(-60));

    private final Pose dropoff3pose = new Pose(28, 31, Math.toRadians(-160));

    private boolean clawOpen = false;

    private RobotAutoPedro robotAuto;

    private JoystickWrapper joystickWrapper;

    @Override
    public void runOpMode() throws InterruptedException {

        robotAuto = new RobotAutoPedro(hardwareMap, gamepad1, gamepad2, telemetry);

        joystickWrapper = new JoystickWrapper(gamepad1, gamepad2);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(autoStartPose);

        PathChain hangPreload = lineToConstantHeading(autoStartPose, preloadHangPose);

        PathChain pickup1 = lineToLinearHeading(preloadHangPose, pickup1Pose);

        PathChain dropoff1 = lineToLinearHeading(pickup1Pose, dropoff1Pose);

        PathChain pickup2 = lineToLinearHeading(dropoff1Pose, pickup2pose);

        PathChain dropoff2 = lineToLinearHeading(pickup2pose, dropoff2pose);

        PathChain pickup3 = lineToLinearHeading(dropoff2pose, pickup3pose);

        PathChain dropoff3 = lineToLinearHeading(pickup3pose, dropoff3pose);

        robotAuto.setIntakeClawAnglePos(RoboSapiensTeleOp.Params.INTAKE_ANGLE_TRANSFER);
        while (!opModeIsActive() && !isStopRequested()) {
            if (joystickWrapper.gamepad1GetA()) {
                if (!clawOpen) {
                    robotAuto.openTopClaw();
                } else {
                    robotAuto.closeTopClaw();
                }

                clawOpen = !clawOpen;
            }
        }

        waitForStart();

        opmodeTimer.resetTimer();
        setPathState(0);

        while (!isStopRequested()) {


            follower.update();


            switch(pathState) {
                case 0:

                    robotAuto.setState(IRobot.State.SPECIMEN_HANG);

                    follower.followPath(hangPreload, true);
                    setPathState(pathState+1);
                    break;

                case 1:
                    if (!follower.isBusy()) {
                        robotAuto.setState(IRobot.State.INTAKINGCLAW);
                        robotAuto.startWait(50);
                        setPathState(pathState+1);
                    }
                    break;


                case 2:
                    if (robotAuto.checkWait()) {
                        follower.followPath(pickup1, true);
                        setPathState(pathState+1);
                    }
                    break;

                case 3:
                    if (!follower.isBusy()) {
                        robotAuto.setHorizontalSlidePos(1650);
                        robotAuto.setIntakeRotServo(RoboSapiensTeleOp.Params.INTAKE_ROT_SERVO_DEFAULT-.16);
                        robotAuto.startWait(750);
                        setPathState(pathState+1);
                    }
                    break;

                case 4:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.PICKUP_GROUND);
                        robotAuto.startWait(400);
                        setPathState(pathState+1);
                    }
                    break;

                case 5:
                    if (robotAuto.checkWait()) {
                        follower.followPath(dropoff1, true);
                        setPathState(pathState+1);
                    }
                    break;


                case 6:
                    if (!follower.isBusy()) {
                        robotAuto.setState(IRobot.State.INTAKINGCLAW);
                        robotAuto.startWait(50);
                        setPathState(pathState+1);
                    }
                    break;

                case 7:
                    if (robotAuto.checkWait()) {
                        follower.followPath(pickup2, true);
                        robotAuto.startWait(100);
                        setPathState(pathState+1);
                    }
                    break;

                case 8:
                    if (robotAuto.checkWait()) {
                        robotAuto.setIntakeRotServo(RoboSapiensTeleOp.Params.INTAKE_ROT_SERVO_DEFAULT-.16);
                        setPathState(pathState+1);
                    }
                    break;


                case 9:
                    if (!follower.isBusy()) {
                        robotAuto.startWait(50);
                        setPathState(pathState+1);
                    }
                    break;

                case 10:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.PICKUP_GROUND);
                        robotAuto.startWait(400);
                        setPathState(pathState+1);
                    }
                    break;

                case 11:
                    if (robotAuto.checkWait()) {
                        follower.followPath(dropoff2, true);
                        setPathState(pathState+1);
                    }
                    break;

                case 12:
                    if (!follower.isBusy()) {
                        robotAuto.setState(IRobot.State.INTAKINGCLAW);
                        robotAuto.startWait(50);
                        setPathState(pathState+1);
                    }
                    break;

                case 13:
                    if (robotAuto.checkWait()) {
                        follower.followPath(pickup3, true);
                        robotAuto.startWait(100);
                        setPathState(pathState+1);
                    }
                    break;

                case 14:
                    if (robotAuto.checkWait()) {
                        robotAuto.setIntakeRotServo(RoboSapiensTeleOp.Params.INTAKE_ROT_SERVO_DEFAULT-.16);
                        setPathState(pathState+1);
                    }
                    break;

                case 15:
                    if (!follower.isBusy()) {
                        robotAuto.startWait(50);
                        setPathState(pathState+1);
                    }
                    break;

                case 16:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.PICKUP_GROUND);
                        robotAuto.startWait(400);
                        setPathState(pathState+1);
                    }
                    break;

                case 17:
                    if (robotAuto.checkWait()) {
                        robotAuto.setHorizontalSlidePos(1150);
                        follower.followPath(dropoff3, true);
                        setPathState(-1);
                    }
                    break;
            }


            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());

            robotAuto.execute();
//            telemetry.update();
        }


    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    //prob move this to a separate class later
    public PathChain lineToLinearHeading(Pose startPose, Pose endPose) {
        return follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(endPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
    }

    public PathChain lineToConstantHeading(Pose startPose, Pose endPose) {
        return follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(endPose)))
                .setConstantHeadingInterpolation(endPose.getHeading())
                .build();
    }

}
