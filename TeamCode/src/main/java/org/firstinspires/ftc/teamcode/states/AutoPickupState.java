package org.firstinspires.ftc.teamcode.states;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.AutoPickupTask;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.robot.MultiColorSampleDetector;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SquidController;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

@Config
public class AutoPickupState extends BaseState {

    public AutoPickupState(JoystickWrapper joystick) {
        super(joystick);
    }

    @Override
    public void initialize(Robot robot, IRobot prevState) {


        AutoPickupTask pickupTask = new AutoPickupTask(new AutoPickupTask.AutoPickupListener() {
            @Override
            public Robot getRobot() {
                return robot;
            }
        });

        taskArrayList.add(pickupTask);
    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {


        executeTasks(telemetry);

    }

    @Override
    public State getState() {
        return State.AUTO_PICKUP;
    }

    @Override
    public void dispose(Robot robot) {
      //  robot.setDriveTrainEnabled(true);
    }

    public double[] getTargetPose(Point center) {
        return new double[] {(240-center.y)*(6.25/480), (320-center.x)*(8.5/640)};
    }
}
