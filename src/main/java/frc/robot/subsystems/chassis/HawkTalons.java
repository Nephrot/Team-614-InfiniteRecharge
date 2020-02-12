package frc.robot.subsystems.chassis;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.subsystems.chassis.SRXPID;
import frc.robot.motionprofile.*;

public class HawkTalons extends WPI_TalonSRX {
    // Set up motion profiling on TalonSRX, called HawkTalons, exactly the same as
    // regular talons but with Motion Magic & Motion Profiling functions
    public Profile profile;
    private int pointsCounter;
    private int talonTimeout = 10;
    public HawkTalons(int port) {
        super(port);
        reset();
        setConfig(new SRXPID(0, 0, 0, 0), 0);
        pointsCounter = 0;
        talonTimeout = 10;
    }
    public void resetEncoder() {
        // Reset encoder position, for reference encoder is CTRE_MagEncoder
        setSelectedSensorPosition(0, 0, talonTimeout);
    }

    public void setConfig(SRXPID pid, int pidChannel) {
        // Set individual talon pid values through seperate class
        // No need to write 4 lines of code anymore
        config_kP(pidChannel, pid.p, talonTimeout);
        config_kI(pidChannel, pid.i, talonTimeout);
        config_kD(pidChannel, pid.d, talonTimeout);
        config_kF(pidChannel, pid.f, talonTimeout);
    }

    public boolean sendNextPoint()
    {
        // Send points from the uploaded profile to the talon's notifier
        System.out.println("Sending Points");
        if (profile == null)
        {
            System.err.println("Error | Attempted to load null profile to talon.");
            // Error 201: This means your profile doesn't exist usually this problem originates
            // from not refrencing the point correctly in OI or your use of motion magic failed 
            // to be implemented.
            // Check:
            // OI
            // Follow Profile
            return false;
        }
        if (pointsCounter >= profile.size())
        {
            System.err.println("Error | Attempted to send more points than were in the profile.");
            // Error 202: This means your point counter is more than the profile itself if this 
            // happens when you dont reference the correct profile size, should never happen with
            // motion magic
            // Check:
            // HawkTalons
            // CreateNewPath.path(path);
            return false;
        }

        if (pushMotionProfileTrajectory(profile.getPoint(pointsCounter)) == ErrorCode.OK)
        {
            pointsCounter++;
            return true;
        }
        return false;
    }

    public MotionProfileStatus motionProfileStatus() {
        // Get motion profile status efficiently without all the hassle
        // just call the method
        MotionProfileStatus status = new MotionProfileStatus();
        getMotionProfileStatus(status);
        return status;
    }

    // The rest is regular drivetrain stuff
    public void init() {
        set(ControlMode.PercentOutput, 0);
        pointsCounter = 0;
        resetEncoder();
        clearMotionProfileTrajectories();
    }

    public void reset() {
        pointsCounter = 0;
        resetEncoder();
        clearMotionProfileTrajectories();
    }

    public void setSpeedZero() {
       set(ControlMode.PercentOutput, 0);
    }

    public void setSpeed(double speed) {
        set(ControlMode.PercentOutput, speed);
    }
    public void setVelocity(double velocity) {
        set(ControlMode.Velocity, velocity);
    }

    public void getPostion() {
        getSelectedSensorPosition(0);
    }

    public void getVelocity() {
        getSelectedSensorVelocity(0);
    }
}