package org.firstinspires.ftc.teamcode.robot;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.IRobotPidMechanism;

abstract class RobotPidMechanism implements IRobotPidMechanism {

    private final PIDEx pidController;
    private int targetPosition;

    public RobotPidMechanism(double kp, double ki, double kd, double minL, double maxL, double rampRate) {

        PIDCoefficientsEx pidCoefficients = new PIDCoefficientsEx(
                kp,     // Proportional gain
                ki,    // Integral gain
                kd,    // Derivative gain
                minL,      // Minimum output limit
                maxL,       // Maximum output limit
                rampRate       // Output ramp rate (optional)
                // Integral wind-up limit (optional)
        );

        pidController = new PIDEx(pidCoefficients);

        targetPosition = 0;
    }

    public RobotPidMechanism() {

        PIDCoefficientsEx pidCoefficients = new PIDCoefficientsEx(
                0.005,     // Proportional gain
                0.0001,    // Integral gain
                0.0003,    // Derivative gain
                -1.0,      // Minimum output limit
                1.0,       // Maximum output limit
                0.1       // Output ramp rate (optional)
                // Integral wind-up limit (optional)
        );

        pidController = new PIDEx(pidCoefficients);

        targetPosition = 0;
    }

    public void update(Telemetry telemetry) {
        int currentPosition = getCurrentPosition();
        double power = pidController.calculate(currentPosition, targetPosition);

        telemetry.addData(getName()+ ": currentPosition", currentPosition);
        telemetry.addData(getName()+ ": Slide Power", power);
        onSetPower(power);
    }

    public void setTargetPosition(int position) {
        targetPosition = position;
    }

    public double testCapPower(double power, double cap) {

        if(cap != -1) {
            if (power < 0) {
                if (power < -cap) {
                    power = -cap;
                }
                ;
            } else if (power > 0) {
                if (power > cap) {
                    power = cap;
                }
            }
        }

        return power;
    }

    public String getName() {
        return "Unknown";
    }

}
