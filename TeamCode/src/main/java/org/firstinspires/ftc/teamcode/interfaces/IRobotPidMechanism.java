package org.firstinspires.ftc.teamcode.interfaces;

public interface IRobotPidMechanism {

    public int getCurrentPosition();
    public void onSetPower(double power);

    public void setTargetPosition(int position);

    public void increaseTargetPosition(int offset);

    public String getName();
}
