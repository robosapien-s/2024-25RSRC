package org.firstinspires.ftc.teamcode.robot;

public class SquidController {
    private double kP;
    private double kD;

    private double lastTime = 0;
    private double currentTime = 0;
    private double lastError = 0;
    private double currentError = 0;


    public SquidController(double kP, double kD) {
        this.kP = kP;
        this.kD = kD;
    }




    public double calculate(double target, double state) {
        lastTime = currentTime;
        currentTime = System.currentTimeMillis();

        double dt = currentTime-lastTime;

        lastError = currentError;

        currentError = target - state;

        double dError = currentError-lastError;

        return kP*signedSqrt(currentError)+(kD*dError/dt);
    }




    public double signedSqrt(double val) {
        return Math.signum(val) * Math.sqrt(Math.abs(val));
    }
}
