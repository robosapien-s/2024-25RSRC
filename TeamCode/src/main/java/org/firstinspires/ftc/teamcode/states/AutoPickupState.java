package org.firstinspires.ftc.teamcode.states;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.robot.MultiColorSampleDetector;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

@Config
public class AutoPickupState extends BaseState {

    public static double xKp = 0.025;
    public static double xKd = 0.01;
    public static double xKi = 0.0;



    public static double yKp = 0.025;
    public static double yKd = 0.0;
    public static double yKi = 0.0;

    public static double capX = .4;
    public static double capY = .3;

    PIDCoefficientsEx pidXCoefficients = new PIDCoefficientsEx(
            xKp, xKi, xKd, 0, 0, 0
    );

    PIDEx xPid = new PIDEx(pidXCoefficients);


    PIDCoefficientsEx pidYCoefficients = new PIDCoefficientsEx(
            yKp, yKi, yKd, 0, 0, 0
    );

    PIDEx yPid = new PIDEx(pidYCoefficients);


    MultiColorSampleDetector colorSampleDetector;

    public AutoPickupState(JoystickWrapper joystick) {
        super(joystick);
    }

    @Override
    public void initialize(Robot robot, IRobot prevState) {



        robot.setDriveTrainEnabled(false);
        colorSampleDetector = robot.createColorSampleDetector(MultiColorSampleDetector.ClosestSamplePipeline.SampleColorPriority.all);

        robot.setSlideRotationPosition(RoboSapiensTeleOp.Params.SLIDE_ROTATION_CAMERA_POSITION);
        robot.setRotAndAnglePosition(RoboSapiensTeleOp.Params.ROT_AND_ANGLE_CAMERA);
        robot.setClawPosition(RoboSapiensTeleOp.Params.CLAW_OPEN);

        if(prevState == null) {
            //use robot.set directly, not a task series
           // robot.setSlideTargetPosition(0);
        }
    }

    RotatedRect prevRect = new RotatedRect();
    long lastAquireTime = 0;


    boolean didGrab = false;
    long delayTimeHack = 0;


    public double calculateDistance(Point p1, Point p2) {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    public void updateFromCamera(Robot robot, Telemetry telemetry) {
        Point centerTarget = colorSampleDetector.getCenterOfScreen();
        centerTarget.y += 100;
        RotatedRect cloestRect = colorSampleDetector.getClosestSample();

        if(delayTimeHack == 0) {
            delayTimeHack = System.currentTimeMillis();
        }

        long timeDelay = System.currentTimeMillis() - delayTimeHack;

        if( timeDelay > 5000 && cloestRect.size.width != 0 && !didGrab) {

            double xPower = 0;
            double yPower = 0;

            xPower = xPid.calculate(centerTarget.x, cloestRect.center.x);
            yPower = yPid.calculate(centerTarget.y, cloestRect.center.y);

            if (xPower < -capX) {
                xPower = -capX;
            } else if (xPower > capX) {
                xPower = capX;
            }

            if (yPower < -capY) {
                yPower = -capY;
            } else if (yPower > capY) {
                yPower = capY;
            }

            if (xPower * xPower + yPower * yPower > 1) {
                double mag = Math.sqrt(xPower * xPower + yPower * yPower);
                xPower = xPower / mag;
                yPower = yPower / mag;
            }

            telemetry.addData("X Power", xPower);
            telemetry.addData("Y Power", yPower);
            telemetry.addData("Current Pos", cloestRect.center);
            telemetry.addData("Target", centerTarget);

            robot.updateDriveTrainsRaw(telemetry, false, -xPower, -yPower, 0, 0, 1, 1);

            double distance = calculateDistance(centerTarget, cloestRect.center);
            telemetry.addData("Distance", distance);
            if(distance < 10 && !didGrab) {
                telemetry.addData("Grab", "dograb");


                double clawPosition = robot.getClawPosition();

                taskArrayList.add(createSlideRotationTask(robot, 0, 200, "Arm Angle", false));

                if(Math.abs(  clawPosition - RoboSapiensTeleOp.Params.CLAW_OPEN ) > .02) {
                    taskArrayList.add(createClawTask(robot, RoboSapiensTeleOp.Params.CLAW_OPEN, 250, "IntakeClawOpen", false));
                }


                taskArrayList.add(createRotationAndAngleTask(robot, RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PICKUP_HORIZONTAL, 50, "IntakeAngle", false));

                taskArrayList.add(createClawTask(robot, RoboSapiensTeleOp.Params.CLAW_CLOSE, 300, "IntakeClawClose", false));


                taskArrayList.add(createRotationAndAngleTask(robot, RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PREP, 50, "IntakeAngle", false));


//                taskArrayList.add(
//                        new ExecuteOnceTask(
//                                new ExecuteOnceTask.ExecuteListener() {
//                                    @Override
//                                    public void execute() {
//                                        didGrab
//                                                = true;
//                                    }
//                                }, "Substate Transition"
//                        )
//                );

                didGrab = true;

            }
        }

        /*

        long timeFromLastAquire = System.currentTimeMillis() - lastAquireTime;


        if(cloestRect.size.width != 0 ||  timeFromLastAquire  < 200) {

            if(cloestRect.size.width == 0) {
                cloestRect = prevRect;
            } else {
                lastAquireTime = System.currentTimeMillis();
                prevRect = cloestRect;
            }

            double xPower = 0;
            double yPower = 0;

            xPower = xPid.calculate(centerTarget.x, cloestRect.center.x);
            yPower = yPid.calculate(centerTarget.y, cloestRect.center.y);

            if(xPower < -.5) {
                xPower = -.5;
            } else if(xPower > .5) {
                xPower = .5;
            }

            if(yPower < -.5) {
                yPower = -.5;
            } else if(yPower > .5) {
                yPower = .5;
            }

            if (xPower * xPower + yPower * yPower > 1) {
                double mag = Math.sqrt(xPower * xPower + yPower * yPower);
                xPower = xPower / mag;
                yPower = yPower / mag;
            }

            telemetry.addData("X Power", xPower);
            telemetry.addData("Y Power", yPower);
            telemetry.addData("Current Pos", cloestRect.center);
            telemetry.addData("Target", centerTarget);

            robot.updateDriveTrainsRaw(telemetry, false, -xPower, -yPower, 0, 0, 1, 1);
        }

         */

    }
    @Override
    public void execute(Robot robot, Telemetry telemetry) {


        updateFromCamera(robot, telemetry);

        if (joystick.gamepad1GetY()) {
            delayTimeHack = 0;
            didGrab = false;
            robot.setSlideRotationPosition(RoboSapiensTeleOp.Params.SLIDE_ROTATION_CAMERA_POSITION);
            robot.setRotAndAnglePosition(RoboSapiensTeleOp.Params.ROT_AND_ANGLE_CAMERA);
            robot.setClawPosition(RoboSapiensTeleOp.Params.CLAW_OPEN);

        }
        else if (joystick.gamepad1GetA()) {
            robot.switchState(State.INTAKINGCLAW);
        }

        executeTasks(telemetry);

    }

    @Override
    public State getState() {
        return State.AUTO_PICKUP;
    }

    @Override
    public void dispose(Robot robot) {
        robot.setDriveTrainEnabled(true);
        colorSampleDetector.stopStreaming();
        colorSampleDetector = null;
    }
}
