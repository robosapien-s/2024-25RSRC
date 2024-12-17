package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.robot.VerticalSlideController;

@TeleOp(name = "Vertical PID Test", group = "Test")
public class VerticalPidTest extends LinearOpMode {

    private VerticalSlideController slideController;
    private double lastkP = 0.0, lastkI = 0.0, lastkD = 0.0;

    @Override
    public void runOpMode() {
        // Initialize the slide controller
        slideController = new VerticalSlideController(
                hardwareMap,
                "verticalSlide1",
                "verticalSlide2",
                true,
                2000,
                0
        );

        // Initialize the FTC Dashboard for real-time monitoring
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Wait for the game to start
        waitForStart();

        while (opModeIsActive()) {
            // Check for changes in PID coefficients and reinitialize if needed
            if (coefficientsChanged()) {
                slideController = new VerticalSlideController(
                        hardwareMap,
                        "verticalSlide1",
                        "verticalSlide2",
                        true,
                        2000, // Maximum position
                        0     // Minimum position
                );
            }

            // Retrieve the target and current positions
            int targetPosition = VerticalSlideController.targetPosition;
            int currentPosition = slideController.getCurrentPosition();

            slideController.setTargetPosition(targetPosition);
            slideController.update(telemetry);
            TelemetryPacket packet = new TelemetryPacket();

            packet.put("kP", VerticalSlideController.kP);
            packet.put("kI", VerticalSlideController.kI);
            packet.put("kD", VerticalSlideController.kD);

            packet.put("Target Position", targetPosition);
            packet.put("Current Position", currentPosition);

            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("kP", VerticalSlideController.kP);
            telemetry.addData("kI", VerticalSlideController.kI);
            telemetry.addData("kD", VerticalSlideController.kD);
            telemetry.update();
        }
    }

    private boolean coefficientsChanged() {
        boolean changed = VerticalSlideController.kP != lastkP ||
                VerticalSlideController.kI != lastkI ||
                VerticalSlideController.kD != lastkD;

        lastkP = VerticalSlideController.kP;
        lastkI = VerticalSlideController.kI;
        lastkD = VerticalSlideController.kD;

        return changed;
    }
}
