package org.firstinspires.ftc.teamcode.opmodes;


import android.transition.Slide;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.CallBackTask;
import org.firstinspires.ftc.teamcode.controllers.IRobotTask;
import org.firstinspires.ftc.teamcode.interfaces.IRobotPidMechanism;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SlideController;
import org.firstinspires.ftc.teamcode.robot.SlideRotationController;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

import java.util.ArrayList;

@TeleOp(name = "Worlds Bot Test", group = "Test")
public class WorldsBotTestOp extends LinearOpMode {



    private ArrayList<IRobotTask> taskArrayList = new ArrayList<IRobotTask>();

    private SlideRotationController horizontalSlideController;
    private double lastHkP = 0.0, lastHkI = 0.0, lastHkD = 0.0;


    private SlideController verticalSlideController;
    private double lastVkP = 0.0, lastVkI = 0.0, lastVkD = 0.0;



    Servo clawServo;
    Servo rightServo;
    Servo leftServo;



    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;



    public static IRobotTask createSlideTask(IRobotPidMechanism slide, int position, int duration, String name, boolean steps) {

        return new CallBackTask(new CallBackTask.CallBackListener() {
            @Override
            public void setPosition(double value) {
                slide.setTargetPosition((int) value);
            }

            @Override
            public double getPosition() {

                return slide.getCurrentPosition();
            }
        }, position, duration, name, steps);
    }


    public static IRobotTask createServoTask(Servo servo, double position, int duration, String name, boolean steps) {

        return new CallBackTask(new CallBackTask.CallBackListener() {
            @Override
            public void setPosition(double value) {
                servo.setPosition(value);
            }

            @Override
            public double getPosition() {

                return servo.getPosition();
            }
        }, position, duration, name, steps);
    }


    public void goUp() {
        taskArrayList.add(   createSlideTask(verticalSlideController, 500, 600, "retract", false));
        taskArrayList.add( createSlideTask(horizontalSlideController, -1400, 500, "up", false));
        taskArrayList.add(   createSlideTask(verticalSlideController, 2500, 500, "extend", false));
    }

    public void goDown() {
        taskArrayList.add(createServoTask(clawServo, .4, 200, "close", false ));
        taskArrayList.add(   createSlideTask(verticalSlideController, 500, 600, "retract", false));
        taskArrayList.add( createSlideTask(horizontalSlideController, 0, 500, "up", false));
        taskArrayList.add(   createSlideTask(verticalSlideController, 1500, 500, "extend", false));
    }

    public void reset() {
        taskArrayList.add(   createSlideTask(verticalSlideController, 0, 500, "retract", false));
        taskArrayList.add( createSlideTask(horizontalSlideController, 0, 500, "up", false));
    }

    public void grab() {
        taskArrayList.add( createSlideTask(horizontalSlideController, 140, 500, "up", false));
        taskArrayList.add(createServoTask(clawServo, .4, 200, "close", false ));
        taskArrayList.add( createSlideTask(horizontalSlideController, 0, 0, "up", false));
    }




    public void runPIDTuneStuff(FtcDashboard dashboard, JoystickWrapper joystick ) {

        if (coefficientsChanged()) {
            horizontalSlideController = new SlideRotationController(
                    hardwareMap,
                    "horizontalSlide1",
                    RoboSapiensTeleOp.Params.SLIDE_ROTATION_MAX_POSITION, // Example max position
                    RoboSapiensTeleOp.Params.SLIDE_ROTATION_MIN_POSITION,
                    false// Example min position,
            );

            verticalSlideController = new SlideController(hardwareMap, "verticalSlide1", "verticalSlide2", "clawSliderEncoder",
                    true,
                    RoboSapiensTeleOp.Params.SLIDE_MAX_POSITION, // Example max position
                    0,
                    false);
        }

        int targetHPosition = SlideRotationController.targetPosition;
        int currentHPosition = horizontalSlideController.getCurrentPosition();


        if(joystick.gamepad1GetB()) {
            horizontalSlideController.setTargetPosition(targetHPosition);
        }
/*

            if(joystick.gamepad1GetLeftBumperRaw()) {
                horizontalSlideController.increaseTargetPosition((int) (joystick.gamepad1GetLeftTrigger()*10));
            } else {
                horizontalSlideController.increaseTargetPosition((int) (joystick.gamepad1GetLeftTrigger()-10));
            }
*/




        // Use the controller to calculate motor power
        horizontalSlideController.update(telemetry);

        // Create a telemetry packet for graphing
        TelemetryPacket packet = new TelemetryPacket();

        // Add PID coefficients
        packet.put("Horiz kP", SlideRotationController.kP);
        packet.put("Horiz kI", SlideRotationController.kI);
        packet.put("Horiz kD", SlideRotationController.kD);

        // Add position and power data
        packet.put("Target Horiz Position", targetHPosition);
        packet.put("Current Horiz Position", currentHPosition);
        // Send the packet to the dashboard
        dashboard.sendTelemetryPacket(packet);

        // Update telemetry (optional, for Driver Station)
        telemetry.addData("Target Horiz Position", targetHPosition);
        telemetry.addData("Current Horiz Position", currentHPosition);
        telemetry.addData("Horiz kP", SlideRotationController.kP);
        telemetry.addData("Horiz kI", SlideRotationController.kI);
        telemetry.addData("Horiz kD", SlideRotationController.kD);



        //Vertical


        int currentVPosition = verticalSlideController.getCurrentPosition();
        int targetVPosition = SlideController.targetPosition;
        double currentPower = verticalSlideController.getCurrentPower();
        if(joystick.gamepad1GetA()) {
            verticalSlideController.setTargetPosition(targetVPosition);
        }




        //slideController.setTargetPosition(targetPosition);
        verticalSlideController.update(telemetry);

        packet.put("Vert kP", SlideController.kP);
        packet.put("Vert kI", SlideController.kI);
        packet.put("Vert kD", SlideController.kD);

        packet.put("Target Vert  Position", targetVPosition);
        packet.put("Current Vert  Position", currentVPosition);
        packet.put("Current Vert  Power", currentPower);

        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("Target Vert  Position", targetVPosition);
        telemetry.addData("Current Vert  Position", currentVPosition);
        telemetry.addData("Current Vert  Power", currentPower);
//            telemetry.addData("kP", VerticalSlideController.kP);
//            telemetry.addData("kI", VerticalSlideController.kI);
//            telemetry.addData("kD", VerticalSlideController.kD);
        telemetry.update();
    }


    @Override
    public void runOpMode() throws InterruptedException {


        frontLeftMotor = hardwareMap.dcMotor.get("fL");
        backLeftMotor = hardwareMap.dcMotor.get("bL");
        frontRightMotor = hardwareMap.dcMotor.get("fR");
        backRightMotor = hardwareMap.dcMotor.get("bR");

        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        clawServo = hardwareMap.get(Servo.class, "clawServo");
        rightServo = hardwareMap.get(Servo.class, "clawAngleServo");
        leftServo = hardwareMap.get(Servo.class, "clawRotationServo");

        rightServo.setPosition(0);
        leftServo.setPosition(1);

        //VerticalSlideController.kP =
        SlideRotationController.kP =  0.035;

        horizontalSlideController = new SlideRotationController(
                hardwareMap,
                "horizontalSlide1",
                RoboSapiensTeleOp.Params.SLIDE_ROTATION_MAX_POSITION, // Example max position
                RoboSapiensTeleOp.Params.SLIDE_ROTATION_MIN_POSITION,
                false// Example min position
        );

        verticalSlideController = new SlideController(hardwareMap, "verticalSlide1", "verticalSlide2", "clawSliderEncoder", true,
                RoboSapiensTeleOp.Params.SLIDE_MAX_POSITION, // Example max position
                0,
                false);

        // Initialize the FTC Dashboard for real-time monitoring
        FtcDashboard dashboard = FtcDashboard.getInstance();
        JoystickWrapper joystick = new JoystickWrapper(gamepad1, gamepad2);


        waitForStart();

        while (opModeIsActive()) {
            // Check for changes in PID coefficients

            double offset = .025;

            if(joystick.gamepad1GetY()) {
                goUp();
            } else if(joystick.gamepad1GetA()) {
                goDown();
            } else if(joystick.gamepad1GetB()) {
                reset();
            } else if(joystick.gamepad1GetX()) {
                grab();
            } else if(joystick.gamepad1GetDUp()) {

                double rightServoPosition = rightServo.getPosition()+offset;
                if(rightServoPosition>1) {
                    rightServoPosition = 1;
                }
                double leftServoPosition = leftServo.getPosition()-offset;
                if(leftServoPosition < 0) {
                    leftServoPosition = 0;
                }

                rightServo.setPosition(rightServoPosition);
                leftServo.setPosition(leftServoPosition);

            } else if(joystick.gamepad1GetDDown()) {
                double leftServoPosition = leftServo.getPosition()+offset;
                if(leftServoPosition>1) {
                    leftServoPosition = 1;
                }
                double rightServoPosition = rightServo.getPosition()-offset;
                if(rightServoPosition < 0) {
                    rightServoPosition = 0;
                }

                rightServo.setPosition(rightServoPosition);
                leftServo.setPosition(leftServoPosition);
            } else if(joystick.gamepad1GetDRight()) {

                double rightServoPosition = rightServo.getPosition()+offset;
                if(rightServoPosition>1) {
                    rightServoPosition = 1;
                }
                double leftServoPosition = leftServo.getPosition()+offset;
                if(leftServoPosition > 1) {
                    leftServoPosition = 1;
                }

                rightServo.setPosition(rightServoPosition);
                leftServo.setPosition(leftServoPosition);

            } else if(joystick.gamepad1GetDLeft()) {
                double leftServoPosition = leftServo.getPosition()-offset;
                if(leftServoPosition<0) {
                    leftServoPosition = 0;
                }
                double rightServoPosition = rightServo.getPosition()-offset;
                if(rightServoPosition < 0) {
                    rightServoPosition = 0;
                }

                rightServo.setPosition(rightServoPosition);
                leftServo.setPosition(leftServoPosition);
            }

            //rightServo.setPosition(.5);
            //leftServo.setPosition(.5);

            if(joystick.gamepad1GetRightBumperRaw()) {
                verticalSlideController.increaseTargetPosition((int) (joystick.gamepad1GetRightTrigger()*50));
                verticalSlideController.increaseTargetPosition((int) (joystick.gamepad1GetLeftTrigger()*-50));
            }

            if(joystick.gamepad1GetLeftBumperRaw()) {
                horizontalSlideController.increaseTargetPosition((int) (joystick.gamepad1GetRightTrigger()*-30));
                horizontalSlideController.increaseTargetPosition((int) (joystick.gamepad1GetLeftTrigger()*30));
            }

            mecDrive();
            executeTasks(telemetry);
            horizontalSlideController.update(telemetry);
            verticalSlideController.update(telemetry);

        }


    }


    private void mecDrive() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    private boolean coefficientsChanged() {
        boolean changed = SlideRotationController.kP != lastHkP ||
                SlideRotationController.kI != lastHkI ||
                SlideRotationController.kD != lastHkD ||

                SlideController.kP != lastVkP ||
                SlideController.kI != lastVkI ||
                SlideController.kD != lastVkD;

        // Update the last known values
        lastHkP = SlideRotationController.kP;
        lastHkI = SlideRotationController.kI;
        lastHkD = SlideRotationController.kD;


        lastVkP = SlideController.kP;
        lastVkI = SlideController.kI;
        lastVkD = SlideController.kD;

        return changed;
    }

    public void executeTasks(Telemetry telemetry) {

        if(!taskArrayList.isEmpty()) {

            boolean isStarted = taskArrayList.get(0).hasStarted();
            boolean isRunning = taskArrayList.get(0).isRunning();
            boolean isComplete = taskArrayList.get(0).isComplete();

            taskArrayList.get(0).execute(telemetry);


            if(isComplete){
                taskArrayList.remove(0);
            }
        }
    }
}
