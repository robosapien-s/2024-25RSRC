package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

@Autonomous
public class leftSideAuto extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(7.3, 113.7, Math.toRadians(0));
    private final Pose dropPose = new Pose(19.6, 130.8, Math.toRadians(-25.4));
    private final Pose secondPickUpPose = new Pose(19.7, 130.8, Math.toRadians(0));
    private final Pose thirdPickUpPose = new Pose(20.4, 131.35, Math.toRadians(21.74));
    private boolean clawOpen = false;

    private RobotAuto robotAuto;

    private JoystickWrapper joystickWrapper;


    @Override
    public void runOpMode() throws InterruptedException {

        robotAuto = new RobotAuto(hardwareMap, gamepad1, gamepad2, telemetry);

        joystickWrapper = new JoystickWrapper(gamepad1, gamepad2);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        //Path chains here
        PathChain firstDrop = lineToLinearHeading(startPose, dropPose);
        PathChain secondPickUp = lineToLinearHeading(dropPose, secondPickUpPose);
        PathChain thirdDrop = lineToLinearHeading(secondPickUpPose, dropPose);
        PathChain thirdPickUp = lineToLinearHeading(dropPose, thirdPickUpPose);
        PathChain fourthDrop = lineToLinearHeading(thirdPickUpPose, dropPose);


        robotAuto.setIntakeAnglePos(RoboSapiensTeleOp.Params.INTAKE_ANGLE_INIT);
        robotAuto.setRotAndAnglePos(RoboSapiensTeleOp.Params.ROT_AND_ANGLE_BASKET);
        while (!opModeIsActive() && !isStopRequested()) {
            if (joystickWrapper.gamepad1GetA()) {
                if (!clawOpen) {
                    robotAuto.openClaw();
                } else {
                    robotAuto.closeClaw();
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
                    follower.followPath(firstDrop, true);
                    robotAuto.setState(IRobot.State.DROPPING_L2);
                    setPathState(pathState+1);
                    break;
                case 1:
                    if (!follower.isBusy()) {
                        setPathState(pathState+1);
                        robotAuto.startWait(500);
                    }
                    break;
                case 2:
                    if (robotAuto.checkWait()) {
                        robotAuto.openClaw();
                        robotAuto.startWait(200);
                        setPathState(pathState+1);
                    }
                    break;
                case 3:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.INTAKINGCLAW);
                        robotAuto.startWait(1000);
                        setPathState(pathState+1);
                    }
                    break;
                case 4:
                    if (robotAuto.checkWait()) {
                        robotAuto.setSlidePos(880);
                        robotAuto.setRotAndAnglePos(RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PICKUP_LEFT);
                        robotAuto.startWait(1000);
                        setPathState(pathState+1);
                    }
                    break;
                case 5:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.PICKUP_GROUND_LEFT);
                        robotAuto.startWait(400);
                        setPathState(pathState+1);
                    }
                    break;
                case 6:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.INTAKINGCLAW);
                        robotAuto.startWait(500);
                        setPathState(pathState+1);
                    }
                    break;
                case 7:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.DROPPING_L2);
                        robotAuto.startWait(1000);
                        setPathState(pathState+1);
                    }
                    break;
                case 8:
                    if (robotAuto.checkWait()) {
                        robotAuto.openClaw();
                        robotAuto.startWait(200);
                        setPathState(pathState+1);
                    }
                    break;
                case 9:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.INTAKINGCLAW);
                        robotAuto.startWait(1000);
                        follower.followPath(secondPickUp, true);
                        setPathState(pathState+1);
                    }
                    break;
                case 10:
                    if (!follower.isBusy() && robotAuto.checkWait()) {
                        robotAuto.startWait(1000);
                        setPathState(pathState+1);
                    }
                    break;
                case 11:
                    if (robotAuto.checkWait()) {
                        robotAuto.setSlidePos(880);
                        robotAuto.startWait(500);
                        setPathState(pathState+1);
                    }
                    break;
                case 12:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.PICKUP_GROUND_LEFT);
                        robotAuto.startWait(400);
                        setPathState(pathState+1);
                    }
                    break;
                case 13:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.INTAKINGCLAW);
                        follower.followPath(thirdDrop, true);
                        robotAuto.startWait(500);
                        setPathState(pathState+1);
                    }
                    break;
                case 14:
                    if (!follower.isBusy() && robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.DROPPING_L2);
                        robotAuto.startWait(1000);
                        setPathState(pathState+1);
                    }
                    break;
                case 15:
                    if (!robotAuto.checkWait()) {
                        robotAuto.openClaw();
                        robotAuto.startWait(200);
                        setPathState(pathState+1);
                    }
                    break;
                case 16:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.INTAKINGCLAW);
                        follower.followPath(thirdPickUp, true);
                        robotAuto.startWait(1000);
                        setPathState(pathState+1);
                    }
                    break;
                case 17:
                    if (!follower.isBusy() && robotAuto.checkWait()) {
                        robotAuto.setSlidePos(880);
                        robotAuto.setRotAndAnglePos(RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PICKUP_RIGHT);
                        robotAuto.startWait(500);
                        setPathState(pathState+1);
                    }
                    break;
                case 18:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.PICKUP_GROUND_LEFT);
                        robotAuto.startWait(400);
                        setPathState(pathState+1);
                    }
                    break;
                case 19:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.INTAKINGCLAW);
                        robotAuto.startWait(500);
                        follower.followPath(fourthDrop, true);
                        setPathState(pathState+1);
                    }
                    break;
                case 20:
                    if (!follower.isBusy() && robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.DROPPING_L2);
                        robotAuto.startWait(1000);
                        setPathState(pathState+1);
                    }
                    break;
                case 21:
                    if (robotAuto.checkWait()) {
                        robotAuto.openClaw();
                        robotAuto.startWait(200);
                        setPathState(pathState+1);
                    }
                    break;
                case 22:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.INTAKINGCLAW);
                        robotAuto.startWait(1000);
                        setPathState(pathState+1);
                    }
                    break;
            }

            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());

            robotAuto.execute();
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

    public PathChain lineToConstantHeading(Pose startPose, Pose endPose, double zeroPowerAccelerationMultiplier) {
        return follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(endPose)))
                .setConstantHeadingInterpolation(endPose.getHeading())
                .setZeroPowerAccelerationMultiplier(zeroPowerAccelerationMultiplier)
                .build();
    }

}
