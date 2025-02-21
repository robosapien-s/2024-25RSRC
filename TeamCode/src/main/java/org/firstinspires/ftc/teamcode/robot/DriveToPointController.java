package org.firstinspires.ftc.teamcode.robot;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class DriveToPointController {

    public static double xKp = .11;
    public static double xKd = 6.5;
    public static double xKi = 0;

    public static double yKp = .11;
    public static double yKd = 6.5;
    public static double yKi = 0;

    public static double angleKp = 2.1;
    public static double angleKd = 0.04;
    public static double angleKi = 0;


    final BasicPID xPid;
    final BasicPID yPid;
    final BasicPID anglePidInner;
    final AngleController anglePid;

    final SquidController xSquid;
    final SquidController ySquid;
    final SquidController angleSquid;


    public DriveToPointController() {

        xPid = new BasicPID(new PIDCoefficients(xKp,xKi,xKd));
        yPid = new BasicPID(new PIDCoefficients(yKp,yKi,yKd));
        anglePidInner = new BasicPID(new PIDCoefficients(angleKp,angleKi,angleKd));
        anglePid = new AngleController(anglePidInner);

        xSquid = new SquidController(xKp);
        ySquid = new SquidController(yKp);
        angleSquid = new SquidController(angleKp);
    }

    public Vector3D calculate(double xTarget, double yTarget, double angleTarget, Pose2d current, Telemetry telemetry) {



        double angle = angleWrap(current.heading.toDouble());

        double xPower = xSquid.calculate(xTarget, current.position.x);
        double yPower = -ySquid.calculate(yTarget, current.position.y);
        double anglePower = -angleSquid.calculate(angleTarget, angle);

        double xRotated = xPower*Math.cos(angle) - yPower*Math.sin(angle);
        double yRotated = xPower*Math.sin(angle) + yPower*Math.cos(angle);

        return new Vector3D(xRotated, yRotated, anglePower);


        /*

         double angle = angleWrap(localizer.getPose().heading.toDouble());

        xPower = xPid.calculate(xTarget, localizer.getPose().position.x);
        yPower = -yPid.calculate(yTarget, localizer.getPose().position.y);
        anglePower = -anglePid.calculate(Math.toRadians(angleTarget), angle);






        xRotated = xPower*Math.cos(angle) - yPower*Math.sin(angle);
        yRotated = xPower*Math.sin(angle) + yPower*Math.cos(angle);

        frontLeftMotor.setPower(xRotated + yRotated + anglePower);
        backLeftMotor.setPower(xRotated - yRotated + anglePower);
        frontRightMotor.setPower(xRotated - yRotated - anglePower);
        backRightMotor.setPower(xRotated + yRotated - anglePower);
        *
         */

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
