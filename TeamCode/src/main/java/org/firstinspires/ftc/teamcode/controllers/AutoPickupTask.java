package org.firstinspires.ftc.teamcode.controllers;

import com.pedropathing.localization.Pose;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.robot.MultiColorSampleDetector;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.states.BaseState;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

public class AutoPickupTask extends  RobotTaskImpl {

    AutoPickupTask.AutoPickupListener _listener;
    boolean _started = false;
    boolean _isComplete = false;

    boolean is_pickupStarted = false;

    MultiColorSampleDetector _detector;

    long delayTimeHack = 0;


    boolean _didPickup = false;
    boolean _didMoveToLocation = false;
    boolean _didFindSample = false;

    RotatedRect _closestRec = null;
    Point _targetLocation = null;

    SquidToPointTask _squidToPointTask = null;

    TaskExecuter _taskExecuter = new TaskExecuter();

    public AutoPickupTask(AutoPickupListener listener) {
        _listener = listener;
    }

    public interface AutoPickupListener {
//        void onUpdate(double x, double y, double heading);
//
//        Pose getCurrentValues();
//
//        void onStarted();
//        void onCompleted();

        Robot getRobot();

       // MultiColorSampleDetector createMultiColorSampleDetector();

    }

    @Override
    public void stopTask() {
        _isComplete = true;
    }

    public Point getTargetPose(Point center) {
        //return new double[] {(240-center.y)*(6.25/480), (320-center.x)*(8.5/640)};
        return new Point((240-center.y)*(6.25/480), (320-center.x)*(8.5/640));
        //return new Point((320-center.x)*(8.5/640), (240-center.y)*(6.25/480));
    }

    public double[] mapAngleToClawPosition() {

        if(_closestRec.size.width>=_closestRec.size.height) {

            if(_closestRec.angle>70) {
                return RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PICKUP_HORIZONTAL;
            } else if(_closestRec.angle<20) {
                return RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PICKUP_VERTICAL;
            } else {
                return RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PICKUP_LEFT;
            }
        } else {
            if(_closestRec.angle<20) {
                return RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PICKUP_HORIZONTAL;
            } else if(_closestRec.angle>70) {
                return RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PICKUP_VERTICAL;
            } else {
                return RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PICKUP_RIGHT;
            }
        }
    }
    public void calculateDistanceToSample() {

        Point centerTarget = _detector.getCenterOfScreen();
        centerTarget.y += 50;
        RotatedRect cloestRect = _detector.getClosestSample();

        if(delayTimeHack == 0) {
            delayTimeHack = System.currentTimeMillis();
        }

        long timeDelay = System.currentTimeMillis() - delayTimeHack;

        if( timeDelay > 2000 && cloestRect.size.width != 0) {

            _closestRec = cloestRect;
            _didFindSample = true;
        }
    }

    @Override
    public void execute(Telemetry telemetry) {

        telemetry.addData("AutoPickup: Started", hasStarted());
        telemetry.addData("AutoPickup: Found Sample", String.valueOf(_didFindSample));
        telemetry.addData("AutoPickup: Moved to Location", _didMoveToLocation);
        telemetry.addData("AutoPickup: Did Pickup", _didPickup);



        Robot robot = _listener.getRobot();

        if(_didPickup) {

            telemetry.addData("AutoPickup", "didPickup");

            telemetry.addData("AutoPickup: Rec", _closestRec);
            telemetry.addData("AutoPickup: Width", _closestRec.size.width);
            telemetry.addData("AutoPickup: Height", _closestRec.size.height);
            telemetry.addData("AutoPickup: Angle", _closestRec.angle);

        } else if(_didMoveToLocation) {
            if (!is_pickupStarted) {
                is_pickupStarted = true;
                double clawPosition = robot.getClawPosition();



                _taskExecuter.add(BaseState.createRotationAndAngleTask(robot, mapAngleToClawPosition(), 50, "IntakeAngle", false));


                _taskExecuter.add(BaseState.createSlideRotationTask(robot, 80, 500, "Arm Angle", false));

                if (Math.abs(clawPosition - RoboSapiensTeleOp.Params.CLAW_OPEN) > .02) {
                    _taskExecuter.add(BaseState.createClawTask(robot, RoboSapiensTeleOp.Params.CLAW_OPEN, 250, "IntakeClawOpen", false));
                }


                _taskExecuter.add(BaseState.createIntakeAngleServoTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_PICKUP-.01, 150, "IntakeAngle", false));

                _taskExecuter.add(BaseState.createClawTask(robot, RoboSapiensTeleOp.Params.CLAW_CLOSE, 300, "IntakeClawClose", false));


                _taskExecuter.add(BaseState.createRotationAndAngleTask(robot, RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PREP, 0, "IntakeAngle", false));

                _taskExecuter.add(BaseState.createIntakeAngleServoTask(robot, RoboSapiensTeleOp.Params.INTAKE_ANGLE_READY, 50, "IntakeAngle", false));


            }
            if(_taskExecuter.isComplete()) {
                //TODO:  check to see if it has it and maybe try a second time???
                _didPickup = true;
            }

        } else if(_didFindSample) {
            _targetLocation = getTargetPose(_closestRec.center);
            robot.setDriveTrainEnabled(false);

            telemetry.addData("AutoPickup: Rect", _closestRec);
            telemetry.addData("AutoPickup: Angle", _closestRec.angle);

            double angle = angleWrap(robot.getPose().getHeading());

            double rotX = _targetLocation.x*Math.cos(angle)-_targetLocation.y*Math.sin(angle);
            double rotY = _targetLocation.y*Math.cos(angle)+_targetLocation.x*Math.sin(angle);


            if(_squidToPointTask == null) {
                _squidToPointTask = new SquidToPointTask(
                        new Pose(rotX, rotY, angle),
                        new SquidToPointTask.SquidToPointListener() {
                            @Override
                            public void onUpdate(double x, double y, double heading) {


                                telemetry.addData("Auto Target: ", new Pose(rotX, rotY, angle));
                                telemetry.addData("Auto Current: ", robot.getPose());
                                telemetry.addData("Auto Power: ", new Vector3D(x,y,heading));


                                robot.moveMecanum(x,y,-heading);


//                               dont use this robot.updateDriveTrainsRaw(telemetry, false, x, y, 0, 0, 1, 1);
                            }

                            @Override
                            public Pose getCurrentValues() {
                                return robot.getPose();
                            }
                        }
                );

                _taskExecuter.add(_squidToPointTask);
            }

            if(_taskExecuter.isComplete()) {
                robot.setDriveTrainEnabled(true);
                _didMoveToLocation = true;
            }

        } else if(hasStarted()) {
            calculateDistanceToSample();
        } else if(!hasStarted()) {

            robot.setDriveTrainEnabled(false);
            _detector = robot.createColorSampleDetector(MultiColorSampleDetector.ClosestSamplePipeline.SampleColorPriority.all);

            robot.setSlideTargetPosition(700);
            robot.setSlideRotationPosition(RoboSapiensTeleOp.Params.SLIDE_ROTATION_CAMERA_POSITION);
            robot.setRotAndAnglePosition(RoboSapiensTeleOp.Params.ROT_AND_ANGLE_CAMERA);
            robot.setIntakeAnglePosition(RoboSapiensTeleOp.Params.INTAKE_ANGLE_CAMERA);
            robot.setClawPosition(RoboSapiensTeleOp.Params.CLAW_OPEN);

            _started = true;
        }

        telemetry.update();
        _taskExecuter.executeTasks(telemetry);

    }

    @Override
    public boolean isBlocking() {
        return false;
    }

    @Override
    public boolean hasStarted() {
        return _started;
    }

    @Override
    public boolean isRunning() {
        return false;
    }

    @Override
    public boolean isComplete() {
        return false;
    }

    @Override
    public void dispose() {
        _listener.getRobot().setDriveTrainEnabled(true); //??
        _detector.stopStreaming();
        _detector = null;
    }

    public double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return radians;
    }
}
