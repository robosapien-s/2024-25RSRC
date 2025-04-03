package org.firstinspires.ftc.teamcode.robot;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.localization.Pose;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class DriveToPointController {

//    public static double xKp = .45;
//    public static double xKd = 2; //PID values
//    public static double xKi = 0;

    public static double xKp = 0.3;
    public static double xKd = 8; //SQUID VALUES

    //public static double xKp = .22;
    //public static double xKd = 18;
//    public static double xKi = 0;

//    public static double yKp = .11;
//    public static double yKd = 10;
    //public static double yKp = .22;
    //public static double yKd = 18;
//    public static double yKi = 0;

    public static double angleKp = 0.7;
    public static double angleKd = 0.4; //SQUID values
//    public static double angleKi = 0;
//    public static double angleKp = .75;
//    public static double angleKd = 40; //PID values
//    public static double angleKi = 0;
//


//    final BasicPID xPid;
//    final BasicPID yPid;
//    final BasicPID anglePidInner;
//    final AngleController anglePid;


    final SquidController xSquid;
    final SquidController ySquid;
    final SquidController angleSquid;

    public DriveToPointController() {

//        xPid = new BasicPID(new PIDCoefficients(xKp,xKi,xKd));
//        yPid = new BasicPID(new PIDCoefficients(xKp,angleKi,xKd));
//        anglePidInner = new BasicPID(new PIDCoefficients(angleKp,0,angleKd));
//        anglePid = new AngleController(anglePidInner);

        xSquid = new SquidController(xKp, xKd);
        ySquid = new SquidController(xKp, xKd);
        angleSquid = new SquidController(angleKp, angleKd);


    }

    public Vector3D calculate(double xTarget, double yTarget, double angleTarget, Pose current, Telemetry telemetry) {
        double angle = angleWrap(current.getHeading());

        double xPower = xSquid.calculate(xTarget, current.getX());
        double yPower = -ySquid.calculate(yTarget, current.getY());


        double anglePower = -angleSquid.calculate(angleTarget, angleWrap(angle));


//        double xPower = xPid.calculate(xTarget, current.getX());
//        double yPower = -yPid.calculate(yTarget, current.getY());
//        double anglePower = -anglePid.calculate(angleTarget, angleWrap(current.getHeading()));

        double xRotated = xPower*Math.cos(angle) - yPower*Math.sin(angle);
        double yRotated = xPower*Math.sin(angle) + yPower*Math.cos(angle);
//
        return new Vector3D(xRotated, yRotated, anglePower);


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
