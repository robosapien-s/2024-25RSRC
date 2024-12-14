//P: 0.0224
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.robot.HorizontalSlideController;

@TeleOp(name = "PID Test", group = "Test")
public class PidTest extends LinearOpMode {

    private HorizontalSlideController slideController;
    private double lastkP = 0.0, lastkI = 0.0, lastkD = 0.0;

    @Override
    public void runOpMode() {
        // Initialize the slide controller
        slideController = new HorizontalSlideController(
                hardwareMap,
                "horizontalSlide1",
                2000, // Example max position
                0     // Example min position
        );

        // Initialize FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        while (opModeIsActive()) {
            // Check for changes in PID coefficients
            if (coefficientsChanged()) {
                slideController = new HorizontalSlideController(
                        hardwareMap,
                        "horizontalSlide1",
                        2000, // Example max position
                        0     // Example min position
                );
            }

            int targetPosition = HorizontalSlideController.targetPosition;
            int currentPosition = slideController.getCurrentPosition();

            slideController.setTargetPosition(targetPosition);
            // Use the controller to calculate motor power
            slideController.update(telemetry);

            // Create a telemetry packet for graphing
            TelemetryPacket packet = new TelemetryPacket();

            // Add PID coefficients
            packet.put("kP", HorizontalSlideController.kP);
            packet.put("kI", HorizontalSlideController.kI);
            packet.put("kD", HorizontalSlideController.kD);

            // Add position and power data
            packet.put("Target Position", targetPosition);
            packet.put("Current Position", currentPosition);
            // Send the packet to the dashboard
            dashboard.sendTelemetryPacket(packet);

            // Update telemetry (optional, for Driver Station)
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("kP", HorizontalSlideController.kP);
            telemetry.addData("kI", HorizontalSlideController.kI);
            telemetry.addData("kD", HorizontalSlideController.kD);
            telemetry.update();
        }
    }

    private boolean coefficientsChanged() {
        boolean changed = HorizontalSlideController.kP != lastkP ||
                HorizontalSlideController.kI != lastkI ||
                HorizontalSlideController.kD != lastkD;

        // Update the last known values
        lastkP = HorizontalSlideController.kP;
        lastkI = HorizontalSlideController.kI;
        lastkD = HorizontalSlideController.kD;

        return changed;
    }
}
