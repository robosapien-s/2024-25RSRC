package org.firstinspires.ftc.teamcode.robot;

public class SquidController {
    private double kP;

    public SquidController(double kP) {
        this.kP = kP;
    }


    public double calculate(double target, double state) {
        double error = target - state;
        return kP*signedSqrt(error);
    }




    public double signedSqrt(double val) {
        return Math.signum(val) * Math.sqrt(Math.abs(val));
    }
}
