package org.firstinspires.ftc.teamcode.robot;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.IRobotPidMechanism;

abstract class RobotPidMechanism implements IRobotPidMechanism {

    private final PIDEx pidController;
    private int targetPosition;
    private int maxPostition;
    private int minPosition;

    public boolean showTelemetry = false;

    public RobotPidMechanism(double kp, double ki, double kd, double minL, double maxL, double rampRate, int inMaxPostition, int inMinPosition) {

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
        maxPostition = inMaxPostition;
        minPosition = inMinPosition;
    }

    public RobotPidMechanism( int inMaxPostition, int inMinPosition) {

        maxPostition = inMaxPostition;
        minPosition = inMinPosition;

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
        double power = pidController.calculate(currentPosition, targetPosition); //TODO: check if this is correct, I think it's backwards

        if(showTelemetry) {
            //power = testCapPower(power, .6);
            telemetry.addData(getName() + ": currentPosition", currentPosition);
            telemetry.addData(getName() + ": targetPosition", targetPosition);
            telemetry.addData(getName() + ": Slide Power", power);
        }
        onSetPower(power);
    }

    public int evaluateConstraints(int position) {
        if(position < minPosition) {
            position = minPosition;
        } else if(position > maxPostition) {
            position = maxPostition;
        }

        return position;
    }

    public void setTargetPosition(int position) {
        targetPosition = evaluateConstraints(position);
    }

    public void increaseTargetPosition(int offset) {
        int newTargetPosition = targetPosition+offset;

        if(newTargetPosition > maxPostition) {
            newTargetPosition = maxPostition;
        } else if(newTargetPosition < minPosition) {
            newTargetPosition = minPosition;
        }

        //TODO need limit
        targetPosition = evaluateConstraints(newTargetPosition);
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
