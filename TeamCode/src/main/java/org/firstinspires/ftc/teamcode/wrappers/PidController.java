/* NOT USED
package org.firstinspires.ftc.teamcode.wrappers;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Utils.Timer;

public class PidController {
    double Pos;
    Timer timer;
    private double Kp;
    private double Ki;
    private double Kd;
    private double upperLimit;
    private double lowerLimit;
    PIDCoefficients coefficients = new PIDCoefficients(Kp,Ki,Kd);

    private IGetPos getPos;
    private ISetPos setPos;

    public PidController(double Ki,double Kd,double Kp,double lower,double upper, IGetPos posfunc, ISetPos powerFunc){

        getPos = posfunc;
        setPos = powerFunc;
    }

    public void update(){


        double currentPos = getPos.getCurrentPos();


        //cal some power
        setPos.setPow(0.0);

    }
}
*/