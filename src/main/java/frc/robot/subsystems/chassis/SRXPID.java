package frc.robot.subsystems.chassis;

public class SRXPID
{
    //Set up PID values for HawkTalons(Drivetrain Master Motors)
    public double p, i, d, f, iZone, peakOutput;

    public SRXPID(double f, double p, double i, double d, double iZ, double pO)
    {
        this.f = f;
        this.p = p;
        this.i = i;
        this.d = d;
        this.iZone = iZ;
        this.peakOutput = pO;
    }

    public SRXPID(double f, double p, double i, double d) {
        this.f = f;
        this.p = p;
        this.i = i;
        this.d = d;
    }
}