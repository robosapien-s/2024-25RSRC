package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.robot.AngleDrive.lineToConstantHeading;
import static org.firstinspires.ftc.teamcode.robot.AngleDrive.lineToLinearHeading;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.*;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

@Autonomous
public class rightSideAuto extends LinearOpMode {


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose autoStartPose = new Pose(7.3, 75.6, Math.toRadians(180));

    private final Pose preloadHangPose = new Pose(34, 74, Math.toRadians(180));

    private final Pose pickup1Pose = new Pose(32, 48.25, Math.toRadians(-60));

    private final Pose dropoff1Pose = new Pose(28, 44, Math.toRadians(-138));

    private final Pose pickup2pose = new Pose(32, 37, Math.toRadians(-60));

    private final Pose dropoff2pose = new Pose(29, 32.75, Math.toRadians(-144));

    private final Pose pickup3pose = new Pose(30.5, 28.5, Math.toRadians(-60));

    private final Pose dropoff3pose = new Pose(28, 31, Math.toRadians(-160));

    private final Pose lineUpWallPose = new Pose(19, 35, Math.toRadians(180));

    private final Pose pickUpWallPose = new Pose(14, 37.5, Math.toRadians(180));

//    private final Pose lineUpWall2Pose = new Pose(20, 45.3, Math.toRadians(180));

    private final Pose pickUpWall2Pose = new Pose(14, 37.5, Math.toRadians(180));

    private final Pose hang1Pose = new Pose(34, 71.6, Math.toRadians(180));

    private final Pose hang2Pose = new Pose(34, 69.2, Math.toRadians(180));

    private final Pose hang3Pose = new Pose(34, 66.8, Math.toRadians(180));

    private final Pose hang4Pose = new Pose(34, 64.4, Math.toRadians(180));

    private boolean clawOpen = false;

    private RobotAuto robotAuto;

    private JoystickWrapper joystickWrapper;

    @Override
    public void runOpMode() throws InterruptedException {

        double[] temp1 = RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PREP;
        double temp2 = RoboSapiensTeleOp.Params.INTAKE_ANGLE_READY;

        RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PREP = new double[] {1, 0};
        RoboSapiensTeleOp.Params.INTAKE_ANGLE_READY = RoboSapiensTeleOp.Params.INTAKE_ANGLE_INIT;

        robotAuto = new RobotAuto(hardwareMap, gamepad1, gamepad2, telemetry, follower);

        RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PREP = temp1;
        RoboSapiensTeleOp.Params.INTAKE_ANGLE_READY = temp2;

        joystickWrapper = new JoystickWrapper(gamepad1, gamepad2);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(autoStartPose);

        PathChain hangPreload = lineToConstantHeading(follower, autoStartPose, preloadHangPose,2);

        PathChain pickup1 = lineToLinearHeading(follower, preloadHangPose, pickup1Pose);

        PathChain dropoff1 = lineToLinearHeading(follower, pickup1Pose, dropoff1Pose, 1);

        PathChain pickup2 = lineToLinearHeading(follower, dropoff1Pose, pickup2pose, 1);

        PathChain dropoff2 = lineToLinearHeading(follower, pickup2pose, dropoff2pose, 1);

        PathChain pickup3 = lineToLinearHeading(follower, dropoff2pose, pickup3pose, 1);

        PathChain dropoff3 = lineToLinearHeading(follower, pickup3pose, dropoff3pose, 1);



        PathChain lineUpWall1 = lineToLinearHeading(follower, dropoff3pose, lineUpWallPose);
        PathChain pickUpWall = lineToConstantHeading(follower, lineUpWallPose, pickUpWallPose);

//        PathChain pickUpWall2 = lineToConstantHeading(lineUpWall2Pose, pickUpWall2Pose);

        PathChain hang1 = lineToConstantHeading(follower, pickUpWallPose, hang1Pose, 1.5);
//        PathChain lineUpWall2 = lineToConstantHeading(hang1Pose, lineUpWall2Pose);
        PathChain pickUpWall2 = lineToConstantHeading(follower, hang1Pose, pickUpWall2Pose,1.5);

        PathChain hang2 = lineToConstantHeading(follower, pickUpWall2Pose, hang2Pose, 1.5);
//        PathChain lineUpWall3 = lineToConstantHeading(hang2Pose, lineUpWall2Pose);
        PathChain pickUpWall3 = lineToConstantHeading(follower, hang2Pose, pickUpWall2Pose,1.5);

        PathChain hang3 = lineToConstantHeading(follower, pickUpWall2Pose, hang3Pose, 1.5);
//        PathChain lineUpWall4 = lineToConstantHeading(hang3Pose, lineUpWall2Pose);
        PathChain pickUpWall4 = lineToConstantHeading(follower, hang3Pose, pickUpWall2Pose,1.5);

        PathChain hang4 = lineToConstantHeading(follower, pickUpWall2Pose, hang4Pose, 1.5);
        PathChain park = lineToConstantHeading(follower, hang4Pose, pickUpWallPose);


//        robotAuto.setIntakeAnglePos(RoboSapiensTeleOp.Params.INTAKE_ANGLE_INIT);
//        robotAuto.setRotAndAnglePos(new double[] {.9,.1});
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
                    robotAuto.setState(IRobot.State.SPECIMEN_HANG);

                    follower.followPath(hangPreload, true);

                    robotAuto.startWait(1100);
                    setPathState(pathState+1);
                    break;

                case 1:
                    if (robotAuto.checkWait() || !follower.isBusy()) {
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
                        robotAuto.setSlidePos(1140);
                        robotAuto.setRotAndAnglePos(RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PICKUP_VERTICAL_LEFT);
                        robotAuto.startWait(500);
                        setPathState(pathState+1);
                    }
                    break;

                case 4:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.PICKUP_GROUND);
                        robotAuto.startWait(600);
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
                        robotAuto.setRotAndAnglePos(RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PICKUP_VERTICAL_LEFT);
                        setPathState(pathState+1);
                    }
                    break;


                case 9:
                    if (!follower.isBusy()) {
                        robotAuto.startWait(250);
                        setPathState(pathState+1);
                    }
                    break;

                case 10:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.PICKUP_GROUND); //pickup 2
                        robotAuto.startWait(600);
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
                        robotAuto.startWait(100);
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
                        robotAuto.setRotAndAnglePos(RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PICKUP_VERTICAL_LEFT);
                        setPathState(pathState+1);
                    }
                    break;

                case 15:
                    if (!follower.isBusy()) {
                        robotAuto.startWait(250);
                        setPathState(pathState+1);
                    }
                    break;

                case 16:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.PICKUP_GROUND); //pickup3
                        robotAuto.startWait(600);
                        setPathState(pathState+1);
                    }
                    break;

                case 17:
                    if (robotAuto.checkWait()) {
                        robotAuto.setSlidePos(800);
                        follower.followPath(dropoff3, true);
                        setPathState(pathState+1);
                    }
                    break;

                case 18:
                    if (!follower.isBusy()) {
                        robotAuto.setState(IRobot.State.INTAKINGCLAW);
                        robotAuto.startWait(50);
                        setPathState(pathState+1);
                    }
                    break;

                case 19:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.WALLPICKUP);
                        follower.followPath(lineUpWall1, true);
                        setPathState(pathState+1);
                    }
                    break;
                case 20:
                    if (!follower.isBusy()) {
                        robotAuto.startWait(50);
                        setPathState(pathState+1);
                    }
                    break;
                case 21:
                    if (robotAuto.checkWait()) {
                        follower.followPath(pickUpWall, true);
                        setPathState(pathState+1);
                    }
                    break;
                case 22:
                    if (!follower.isBusy()) {
                        robotAuto.setState(IRobot.State.SPECIMEN_HANG);
                        robotAuto.startWait(50);
                        setPathState(pathState+1);
                    }
                    break;

                case 23:
                    if (robotAuto.checkWait()) {
                        follower.followPath(hang1, true);
                        setPathState(pathState+1);
                    }
                    break;

                case 24:
                    if (!follower.isBusy()) {
                        robotAuto.setState(IRobot.State.WALLPICKUP);
                        robotAuto.startWait(50);
                        setPathState(pathState+1);
                    }
                    break;
                    //first hang done

                case 25:
                    if (robotAuto.checkWait()) {
                        follower.followPath(pickUpWall2, true);
                        setPathState(pathState+1);
                    }
                    break;

                case 26:
                    if (!follower.isBusy()) {
                        robotAuto.startWait(50);
                        setPathState(pathState+1);
                    }
                    break;
//                case 27:
//                    if (robotAuto.checkWait()) {
//                        follower.followPath(pickUpWall2, true);
//                        setPathState(pathState+1);
//                    }
//                    break;
                case 27:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.SPECIMEN_HANG);
                        robotAuto.startWait(50);
                        setPathState(pathState+1);
                    }
                    break;

                case 28:
                    if (robotAuto.checkWait()) {
                        follower.followPath(hang2, true);
                        setPathState(pathState+1);
                    }
                    break;

                case 29:
                    if (!follower.isBusy()) {
                        robotAuto.setState(IRobot.State.WALLPICKUP);
                        robotAuto.startWait(50);
                        setPathState(pathState+1);
                    }
                    break;

                //hang 2 done

                case 30:
                    if (robotAuto.checkWait()) {
                        follower.followPath(pickUpWall3, true);
                        setPathState(pathState+1);
                    }
                    break;

                case 31:
                    if (!follower.isBusy()) {
                        robotAuto.startWait(50);
                        setPathState(pathState+1);
                    }
                    break;
//                case 32:
//                    if (robotAuto.checkWait()) {
//                        follower.followPath(pickUpWall2, true);
//                        setPathState(pathState+1);
//                    }
//                    break;
                case 32:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.SPECIMEN_HANG);
                        robotAuto.startWait(50);
                        setPathState(pathState+1);
                    }
                    break;

                case 33:
                    if (robotAuto.checkWait()) {
                        follower.followPath(hang3, true);
                        setPathState(pathState+1);
                    }
                    break;

                case 34:
                    if (!follower.isBusy()) {
                        robotAuto.setState(IRobot.State.WALLPICKUP);
                        robotAuto.startWait(50);
                        setPathState(pathState+1);
                    }
                    break;

                //hang 3 done

                case 35:
                    if (robotAuto.checkWait()) {
                        follower.followPath(pickUpWall4, true);
                        setPathState(pathState+1);
                    }
                    break;

                case 36:
                    if (!follower.isBusy()) {
                        robotAuto.startWait(50);
                        setPathState(pathState+1);
                    }
                    break;
//                case 39:
//                    if (robotAuto.checkWait()) {
//                        follower.followPath(pickUpWall2, true);
//                        setPathState(pathState+1);
//                    }
//                    break;
                case 37:
                    if (robotAuto.checkWait()) {
                        robotAuto.setState(IRobot.State.SPECIMEN_HANG);
                        robotAuto.startWait(50);
                        setPathState(pathState+1);
                    }
                    break;

                case 38:
                    if (robotAuto.checkWait()) {
                        follower.followPath(hang4, true);
                        setPathState(pathState+1);
                    }
                    break;

                case 39:
                    if (!follower.isBusy()) {
                        robotAuto.setState(IRobot.State.INTAKINGCLAW);
                        robotAuto.startWait(50);
                        setPathState(pathState+1);
                    }
                    break;

                //hang 4 done

                case 40:
                    if (robotAuto.checkWait()) {
                        follower.followPath(park, true);
                        setPathState(pathState+1);
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

}
