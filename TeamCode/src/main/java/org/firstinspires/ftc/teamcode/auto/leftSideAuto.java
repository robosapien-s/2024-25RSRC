package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.robot.AngleDrive.lineToLinearHeading;
import static org.firstinspires.ftc.teamcode.robot.AngleDrive.splineToLinearHeading;
import static org.firstinspires.ftc.teamcode.robot.AngleDrive.splineToTangentialHeading;
import static org.firstinspires.ftc.teamcode.robot.AngleDrive.splineToTangentialHeadingReverse;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

@Autonomous
public class leftSideAuto extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(9.154, 113.7, Math.toRadians(0));
    private final Pose dropPose = new Pose(19.25, 131.15, Math.toRadians(-25.4));
//    private final Pose dropPose = new Pose(20, 130.4, Math.toRadians(-25.4));
    private final Pose firstPickUpPose = new Pose(21.5, 131.8, Math.toRadians(-25.4));
    private final Pose secondPickUpPose = new Pose(19.4, 130.5, Math.toRadians(0));
    private final Pose thirdPickUpPose = new Pose(20.25, 131, Math.toRadians(21.74));
    private final Pose submersibleLineUpPose = new Pose(57, 115, Math.toRadians(-90));
    private final Pose submersiblePickUp1Pose = new Pose(60, 107, Math.toRadians(-90));
    private final Pose submersiblePickUp2Pose = new Pose(62.5, 107, Math.toRadians(-90));

    private final Pose dropLineUpPose = new Pose(55, 110, Math.toRadians(-25.4));

    private boolean clawOpen = false;

    private RobotAuto robotAuto;

    private JoystickWrapper joystickWrapper;


    @Override
    public void runOpMode() throws InterruptedException {

        joystickWrapper = new JoystickWrapper(gamepad1, gamepad2);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);


        robotAuto = new RobotAuto(hardwareMap, gamepad1, gamepad2, telemetry, follower, Robot.rotateAngleOffset);

        follower.setStartingPose(startPose);
        //Path chains here
        PathChain firstDrop = lineToLinearHeading(follower, startPose, dropPose,1.5);
        PathChain firstPickUp = lineToLinearHeading(follower, dropPose, firstPickUpPose,2);
        PathChain secondDrop = lineToLinearHeading(follower, firstPickUpPose, dropPose,2);
        PathChain secondPickUp = lineToLinearHeading(follower, dropPose, secondPickUpPose,2);
        PathChain thirdDrop = lineToLinearHeading(follower, secondPickUpPose, dropPose,2);
        PathChain thirdPickUp = lineToLinearHeading(follower, dropPose, thirdPickUpPose,2);
        PathChain fourthDrop = lineToLinearHeading(follower,thirdPickUpPose, dropPose,2);
//        PathChain submersibleLineUp = lineToLinearHeading(follower, dropPose, submersibleLineUpPose);
        PathChain submersiblePickUp1 = splineToLinearHeading(follower, dropPose,submersibleLineUpPose, submersiblePickUp1Pose);
        PathChain submersiblePickUp2 = splineToLinearHeading(follower, dropPose,submersibleLineUpPose, submersiblePickUp1Pose);

//        PathChain submersibleBackup1 = null;
//
        PathChain finalDrop = null;


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
                        robotAuto.startWait(150);
                        setPathState(pathState+1);
                    }
                    break;
                case 3:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.INTAKINGCLAW);
                        follower.followPath(firstPickUp, true);
                        robotAuto.startWait(1200);
                        setPathState(pathState+1);
                    }
                    break;
                case 4:
                    if (!follower.isBusy() && robotAuto.checkWait()) {
                        robotAuto.setSlidePos(1290);
                        robotAuto.setRotAndAnglePos(RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PICKUP_LEFT);
                        robotAuto.startWait(650);
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
                        follower.followPath(secondDrop);
                        robotAuto.setSlidePos(0);
                        robotAuto.setState(IRobot.State.INTAKINGCLAW);
                        robotAuto.startWait(0);
                        setPathState(pathState + 1);
                    }
                    break;
                case 7:
                    if (robotAuto.checkWait()/* && !follower.isBusy()*/) {
                        robotAuto.setState(IRobot.State.DROPPING_L2);
                        robotAuto.startWait(1900);
                        setPathState(pathState+1);
                    }
                    break;
                case 8:
                    if (robotAuto.checkWait()) {
                        robotAuto.openClaw();
                        robotAuto.startWait(100);
                        setPathState(pathState+1);
                    }
                    break;
                case 9:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.INTAKINGCLAW);
                        robotAuto.startWait(1200);
                        follower.followPath(secondPickUp, true);
                        setPathState(pathState+1);
                    }
                    break;
                case 10:

                    setPathState(pathState+1);

                    break;
                case 11:
                    if (robotAuto.checkWait()) {
                        robotAuto.setSlidePos(1200);
                        robotAuto.startWait(650);
                        setPathState(pathState+1);
                    }
                    break;
                case 12:
                    if (!follower.isBusy() && robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.PICKUP_GROUND_LEFT);
                        robotAuto.startWait(500);
                        setPathState(pathState+1);
                    }
                    break;
                case 13:
                    if (robotAuto.checkWait()) {
                        robotAuto.setSlidePos(0);
                        follower.followPath(thirdDrop, true);
                        robotAuto.startWait(0);
                        setPathState(pathState+1);
                    }
                    break;
                case 14:
                    if (/*!follower.isBusy() &&*/ robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.DROPPING_L2);
                        robotAuto.startWait(1900);
                        setPathState(pathState+1);
                    }
                    break;
                case 15:
                    if (robotAuto.checkWait()) {
                        robotAuto.openClaw();
                        robotAuto.startWait(100);
                        setPathState(pathState+1);
                    }
                    break;
                case 16:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.INTAKINGCLAW);
                        follower.followPath(thirdPickUp, true);
                        robotAuto.startWait(1200);
                        setPathState(pathState+1);
                    }
                    break;
                case 17:
                    if (!follower.isBusy() && robotAuto.checkWait()) {
                        robotAuto.setSlidePos(1290);
                        robotAuto.setRotAndAnglePos(new double[] {.03, 0.8678});
                        robotAuto.startWait(650);
                        setPathState(pathState+1);
                    }
                    break;
                case 18:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.PICKUP_GROUND_LEFT);
                        robotAuto.startWait(600);
                        setPathState(pathState+1);
                    }
                    break;
                case 19:
                    if (robotAuto.checkWait()) {
                        robotAuto.setSlidePos(0);
                        robotAuto.startWait(0);
                        follower.followPath(fourthDrop, true);
                        setPathState(pathState+1);
                    }
                    break;
                case 20:
                    if (/*!follower.isBusy() &&*/ robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.DROPPING_L2);
                        robotAuto.startWait(1900);
                        setPathState(pathState+1);
                    }
                    break;
                case 21:
                    if (robotAuto.checkWait()) {
                        robotAuto.openClaw();
                        robotAuto.startWait(100);
                        setPathState(pathState+1);
                    }
                    break;
                case 22:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.INTAKINGCLAW);
                        robotAuto.startWait(1200);
                        setPathState(pathState+1);
                    }
                    break;
                case 23:
                    if (robotAuto.checkWait()) {
                        follower.followPath(submersiblePickUp1, true);
                        robotAuto.startWait(1200);
                        setPathState(pathState+1);
                    }
                    break;
                case 24:
//                    if (!follower.isBusy()) {
//                        follower.followPath(submersiblePickUp1, true);
//                        setPathState(pathState+1);
//                    }

                    setPathState(pathState+1);
                    break;
                case 25:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.AUTO_PICKUP);
                        setPathState(pathState+1);
                    }
                    break;
                case 26:
                    if (!follower.isBusy())  {
                        follower.breakFollowing();
                        setPathState(pathState+1);
                    }
                    break;
                case 27:
                    if (robotAuto.getState() == IRobot.State.INTAKINGCLAW) {
                        robotAuto.startWait(0);
                        finalDrop = splineToLinearHeading(follower, follower.getPose(), dropLineUpPose, dropPose);
                        setPathState(pathState+1);
                    }
                    break;
                case 28:
                    if (robotAuto.checkWait()) {
                        if (finalDrop == null) {
                            throw (new NullPointerException("final drop path is null"));
                        }
                        follower.followPath(finalDrop, true);
                        setPathState(pathState+1);
                    }
                    break;
                case 29:
                    robotAuto.startWait(600);
                    setPathState(pathState+1);
                    break;
                case 30:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.DROPPING_L2);
                        robotAuto.startWait(1800);
                        setPathState(pathState+1);
                    }
                    break;
                case 31:
                    if (!follower.isBusy() && robotAuto.checkWait()) {
                        robotAuto.openClaw();
                        robotAuto.startWait(100);
                        setPathState(pathState+1);
                    }
                    break;
                case 32:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.INTAKINGCLAW);
                        robotAuto.startWait(300);
                        setPathState(pathState+1);
                    }
                    break;
//                case 33:
//                    if (robotAuto.checkWait()) {
//                        follower.followPath(submersiblePickUp2, true);
//                        robotAuto.startWait(850);
//                        setPathState(pathState+1);
//                    }
//                    break;
//                case 34:
//                    if (robotAuto.checkWait()) {
//                        robotAuto.setState(IRobot.State.AUTO_PICKUP);
//                        setPathState(pathState+1);
//                    }
//                    break;
//                case 35:
//                    if (!follower.isBusy()) {
//                        follower.breakFollowing();
//                        setPathState(pathState+1);
//                    }
//                    break;
//
//                case 36:
//                    if (robotAuto.getState() == IRobot.State.INTAKINGCLAW) {
//                        robotAuto.startWait(0);
//                        finalDrop = splineToLinearHeading(follower, follower.getPose(), dropLineUpPose, dropPose);
//                        setPathState(pathState+1);
//                    }
//                    break;
//                case 37:
//                    follower.followPath(finalDrop, true);
//                    robotAuto.startWait(600);
//                    setPathState(pathState+1);
//                    break;
//                case 38:
//                    if (robotAuto.checkWait()) {
//                        robotAuto.setState(IRobot.State.DROPPING_L2);
//                        robotAuto.startWait(1800);
//                        setPathState(pathState+1);
//                    }
//                    break;
//                case 39:
//                    if (!follower.isBusy() && robotAuto.checkWait()) {
//                        robotAuto.openClaw();
//                        robotAuto.startWait(100);
//                        setPathState(pathState+1);
//                    }
//                    break;
//                case 40:
//                    if (robotAuto.checkWait()) {
//                        robotAuto.setState(IRobot.State.INTAKINGCLAW);
//                        setPathState(pathState+1);
//                    }
//                    break;
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


}
