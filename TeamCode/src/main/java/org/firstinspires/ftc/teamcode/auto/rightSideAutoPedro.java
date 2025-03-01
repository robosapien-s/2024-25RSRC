package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.*;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

@Autonomous
public class rightSideAutoPedro extends LinearOpMode {


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose autoStartPose = new Pose(8.5, 76.5, Math.toRadians(0));

    private final Pose preloadHangPose = new Pose(30.25, 74, Math.toRadians(0));

    private final Pose pickup1Pose = new Pose(31, 47.75, Math.toRadians(-60));

    private final Pose dropoff1Pose = new Pose(29, 45.5, Math.toRadians(-144));

    private final Pose pickup2pose = new Pose(28, 40.5, Math.toRadians(-60));

    private final Pose dropoff2pose = new Pose(29, 39, Math.toRadians(-150));

    private final Pose pickup3pose = new Pose(30, 30, Math.toRadians(-60));

    private final Pose dropoff3pose = new Pose(28, 31, Math.toRadians(-160));

    private final Pose lineUpWall = new Pose(14, 38.8, Math.toRadians(20));

    private final Pose pickUpWallPose = new Pose(10.24, 38.8, Math.toRadians(20));

    private final Pose hang1Pose = new Pose(31.75, 65, Math.toRadians(20));

    private final Pose hang2Pose = new Pose(30.15, 65, Math.toRadians(20));

    private final Pose hang3Pose = new Pose(28.4, 65, Math.toRadians(20));

    private final Pose hang4Pose = new Pose(26.9, 65, Math.toRadians(20));

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



        PathChain pickUpWall1 = follower.pathBuilder()
            .addPath(new BezierLine(new Point(dropoff3pose), new Point(lineUpWall)))
            .setLinearHeadingInterpolation(dropoff3pose.getHeading(), lineUpWall.getHeading())
            .addPath(new BezierLine(new Point(lineUpWall), new Point(pickUpWallPose)))
            .setConstantHeadingInterpolation(lineUpWall.getHeading())
            .build();

        PathChain hang1 = lineToConstantHeading(pickUpWallPose, hang1Pose);
        PathChain pickUpWall2 = lineToConstantHeading(hang1Pose, pickUpWallPose);

        PathChain hang2 = lineToConstantHeading(pickUpWallPose, hang2Pose);
        PathChain pickUpWall3 = lineToConstantHeading(hang2Pose, pickUpWallPose);

        PathChain hang3 = lineToConstantHeading(pickUpWallPose, hang3Pose);
        PathChain pickUpWall4 = lineToConstantHeading(hang3Pose, pickUpWallPose);

        PathChain hang4 = lineToConstantHeading(pickUpWallPose, hang4Pose);
        PathChain park = lineToConstantHeading(hang4Pose, pickUpWallPose);

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
                        robotAuto.startWait(550);
                        setPathState(pathState+1);
                    }
                    break;


                case 6:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.INTAKINGCLAW);
                        setPathState(pathState+1);
                    }
                    break;

                case 7:
                    if (!follower.isBusy()) {
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
                        robotAuto.startWait(600);
                        setPathState(pathState+1);
                    }
                    break;

                case 12:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.INTAKINGCLAW);
                        setPathState(pathState+1);
                    }
                    break;

                case 13:
                    if (!follower.isBusy()) {
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
                        robotAuto.startWait(500);
                        setPathState(pathState+1);
                    }
                    break;

                case 18:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.INTAKINGCLAW);
                        setPathState(pathState+1);
                    }
                    break;

                case 19:
                    if (!follower.isBusy()) {
                        robotAuto.setState(IRobot.State.WALLPICKUP);
                        follower.followPath(pickUpWall1);
                        setPathState(pathState+1);
                    }
                    break;

                case 20:
                    if (!follower.isBusy()) {
                        robotAuto.setState(IRobot.State.SPECIMEN_HANG);
                        robotAuto.startWait(50);
                        setPathState(pathState+1);
                    }
                    break;

                case 21:
                    if (robotAuto.checkWait()) {
                        follower.followPath(hang1);
                        setPathState(pathState+1);
                    }
                    break;

                case 22:
                    if (!follower.isBusy()) {
                        robotAuto.setState(IRobot.State.WALLPICKUP);
                        robotAuto.startWait(50);
                        setPathState(pathState+1);
                    }
                    break;
                    //first hang done

                case 23:
                    if (robotAuto.checkWait()) {
                        follower.followPath(pickUpWall2);
                        setPathState(pathState+1);
                    }
                    break;

                case 24:
                    if (!follower.isBusy()) {
                        robotAuto.setState(IRobot.State.SPECIMEN_HANG);
                        robotAuto.startWait(50);
                        setPathState(pathState+1);
                    }
                    break;

                case 25:
                    if (robotAuto.checkWait()) {
                        follower.followPath(hang2);
                        setPathState(pathState+1);
                    }
                    break;

                case 26:
                    if (!follower.isBusy()) {
                        robotAuto.setState(IRobot.State.WALLPICKUP);
                        robotAuto.startWait(50);
                        setPathState(pathState+1);
                    }
                    break;

                //hang 2 done

                case 27:
                    if (robotAuto.checkWait()) {
                        follower.followPath(pickUpWall3);
                        setPathState(pathState+1);
                    }
                    break;

                case 28:
                    if (!follower.isBusy()) {
                        robotAuto.setState(IRobot.State.SPECIMEN_HANG);
                        robotAuto.startWait(50);
                        setPathState(pathState+1);
                    }
                    break;

                case 29:
                    if (robotAuto.checkWait()) {
                        follower.followPath(hang3);
                        setPathState(pathState+1);
                    }
                    break;

                case 30:
                    if (!follower.isBusy()) {
                        robotAuto.setState(IRobot.State.WALLPICKUP);
                        robotAuto.startWait(50);
                        setPathState(pathState+1);
                    }
                    break;

                //hang 3 done

                case 31:
                    if (robotAuto.checkWait()) {
                        follower.followPath(pickUpWall4);
                        setPathState(pathState+1);
                    }
                    break;

                case 32:
                    if (!follower.isBusy()) {
                        robotAuto.setState(IRobot.State.SPECIMEN_HANG);
                        robotAuto.startWait(50);
                        setPathState(pathState+1);
                    }
                    break;

                case 33:
                    if (robotAuto.checkWait()) {
                        follower.followPath(hang4);
                        setPathState(pathState+1);
                    }
                    break;

                case 34:
                    if (!follower.isBusy()) {
                        robotAuto.setState(IRobot.State.INTAKINGCLAW);
                        robotAuto.startWait(50);
                        setPathState(pathState+1);
                    }
                    break;

                //hang 4 done

                case 35:
                    if (robotAuto.checkWait()) {
                        follower.followPath(park);
                        setPathState(pathState+1);
                    }
                    break;


            }


            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            int i = 1+1;

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
