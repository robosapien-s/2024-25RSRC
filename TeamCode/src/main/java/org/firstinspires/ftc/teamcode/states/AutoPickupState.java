package org.firstinspires.ftc.teamcode.states;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.DriveToPointTask;
import org.firstinspires.ftc.teamcode.controllers.ExecuteOnceTask;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.robot.MultiColorSampleDetector;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;
import org.opencv.core.Rect;

public class AutoPickupState extends BaseState {

    private MultiColorSampleDetector sampleDetector;
    private boolean targetAcquired = false;
    private String targetColor = "None";
    private double targetX = 0;
    private double targetY = 0;

    // Constants for PID motion control
    private static final double CAMERA_CENTER_X = 320.0; // Camera center X coordinate
    private static final double CAMERA_CENTER_Y = 240.0; // Camera center Y coordinate

    // Robot movement parameters
    private static final double APPROACH_DISTANCE = 15.0; // Distance to move when approaching sample
    private static final double PICKUP_DISTANCE = 5.0;    // Final distance for pickup

    // State machine states
    private enum PickupState {
        SCANNING,      // Looking for sample
        APPROACHING,   // Moving to sample
        ALIGNING,      // Aligning with sample
        PICKING_UP,    // Picking up sample
        COMPLETE       // Pickup complete
    }

    private PickupState currentPickupState = PickupState.SCANNING;

    public AutoPickupState(JoystickWrapper joystick) {
        super(joystick);
    }

    @Override
    public void initialize(Robot robot, IRobot prevState) {
        // Create a detector instance if not already created
        if (sampleDetector == null) {
            sampleDetector = new MultiColorSampleDetector(robot.getHardwareMap(), telemetry);
        }

        currentPickupState = PickupState.SCANNING;
        targetAcquired = false;

        // Setup initial scanning task
        RobotTaskSeries scanningTask = new RobotTaskSeries();
        scanningTask.add(new ExecuteOnceTask(new ExecuteOnceTask.ExecuteListener() {
            @Override
            public void execute() {
                // Prepare robot for scanning
                robot.setClawPosition(RoboSapiensTeleOp.Params.CLAW_OPEN);
                robot.setRotAndAnglePosition(RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PICKUP_HORIZONTAL);
            }
        }, "Initial Scanning Setup"));

        taskArrayList.add(scanningTask);
    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {
        // Update sample detection
        Rect closestSample = null;
        if (sampleDetector != null) {
            closestSample = sampleDetector.pipeline.getClosestSample();
        }

        if (closestSample != null) {
            targetColor = sampleDetector.getLastDetectedColor();
            targetX = closestSample.x + closestSample.width / 2.0;
            targetY = closestSample.y + closestSample.height / 2.0;
            targetAcquired = true;
        }

        telemetry.addData("State", currentPickupState.toString());
        telemetry.addData("Target Acquired", targetAcquired);
        telemetry.addData("Target Color", targetColor);
        telemetry.addData("Target Position", String.format("X: %.2f, Y: %.2f", targetX, targetY));

        // State machine for pickup sequence
        switch (currentPickupState) {
            case SCANNING:
                if (targetAcquired) {
                    // Move to approaching state
                    currentPickupState = PickupState.APPROACHING;

                    // Calculate target position for approach
                    // Convert camera coordinates to robot movement
                    double xOffset = (targetX - CAMERA_CENTER_X) / CAMERA_CENTER_X; // Normalized -1 to 1
                    double yOffset = (targetY - CAMERA_CENTER_Y) / CAMERA_CENTER_Y; // Normalized -1 to 1

                    // Convert to robot coordinates and apply approach distance
                    Vector3D currentPose = robot.getDeadWheelLocation();
                    double targetRobotX = currentPose.getX() + (xOffset * APPROACH_DISTANCE);
                    double targetRobotY = currentPose.getY() + (yOffset * APPROACH_DISTANCE);

                    // Create drive task to approach target
                    DriveToPointTask approachTask = new DriveToPointTask(
                            robot,
                            new Vector3D(targetRobotX, targetRobotY, currentPose.getZ()),
                            2000, // timeout in ms
                            1L,  // max speed
                            0L   // initial timeout
                    );

                    taskArrayList.add(approachTask);
                } else {
                    // Continue scanning
                    telemetry.addLine("Scanning for samples...");
                }
                break;

            case APPROACHING:
                if (taskArrayList.isEmpty()) {
                    // We've reached approach position, now align
                    currentPickupState = PickupState.ALIGNING;

                    // Prepare alignment task
                    RobotTaskSeries alignTask = new RobotTaskSeries();
                    alignTask.add(createClawHorizontalAngleTask(robot,
                            RoboSapiensTeleOp.Params.CLAW_HORIZONTAL_ANGLE_CENTER,
                            500, "Center Claw", false));

                    alignTask.add(createRotationAndAngleTask(robot,
                            RoboSapiensTeleOp.Params.ROT_AND_ANGLE_PICKUP_HORIZONTAL,
                            500, "Position Claw", false));

                    taskArrayList.add(alignTask);
                }
                break;

            case ALIGNING:
                if (taskArrayList.isEmpty()) {
                    // Move to pickup position
                    currentPickupState = PickupState.PICKING_UP;

                    // Get final position for pickup
                    Vector3D currentPose = robot.getDeadWheelLocation();
                    double pickupX = currentPose.getX() + PICKUP_DISTANCE;

                    // Create drive task to pickup position
                    DriveToPointTask pickupTask = new DriveToPointTask(
                            robot,
                            new Vector3D(pickupX, currentPose.getY(), currentPose.getZ()),
                            1500, // timeout in ms
                    1L,  // slower speed for precision
                            0L   // initial timeout
                    );

                    taskArrayList.add(pickupTask);

                    // Prepare grab sequence
                    RobotTaskSeries grabSequence = new RobotTaskSeries();
                    grabSequence.add(createWaitTask(robot, 500, "Wait before grab"));
                    grabSequence.add(createClawTask(robot,
                            RoboSapiensTeleOp.Params.CLAW_CLOSE,
                            300, "Close Claw", false));
                    grabSequence.add(createWaitTask(robot, 500, "Wait after grab"));

                    // Add lift sequence
                    grabSequence.add(createVerticalSlideTask(robot,
                            RoboSapiensTeleOp.Params.SLIDE_POSITION,
                            800, "Lift Sample", false));

                    taskArrayList.add(grabSequence);
                }
                break;

            case PICKING_UP:
                if (taskArrayList.isEmpty()) {
                    // Pickup complete
                    currentPickupState = PickupState.COMPLETE;

                    // Add completion task - reset to intaking state
                    RobotTaskSeries completeTask = new RobotTaskSeries();
                    completeTask.add(new ExecuteOnceTask(new ExecuteOnceTask.ExecuteListener() {
                        @Override
                        public void execute() {
                            robot.switchState(State.INTAKINGCLAW);
                        }
                    }, "Complete Pickup"));

                    taskArrayList.add(completeTask);
                }
                break;

            case COMPLETE:
                // Wait for final tasks to complete
                break;
        }

        executeTasks(telemetry);

        if (joystick.gamepad1GetB()) {
            robot.switchState(State.INTAKINGCLAW);
        }
    }

    @Override
    public State getState() {
        return State.AUTO_PICKUP;
    }
}