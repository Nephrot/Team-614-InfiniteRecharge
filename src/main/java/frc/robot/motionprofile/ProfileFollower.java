package frc.robot.motionprofile;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.subsystems.chassis.HawkTalons;

public class ProfileFollower {

    private HawkTalons leftMotor;
    private HawkTalons rightMotor;

    public Notifier processBuffer;

    public ProfileFollower(HawkTalons leftMotor, HawkTalons rightMotor)
    {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        processBuffer = new Notifier(() -> {
            System.out.println("processBuffer notifier");
            processMotionProfileBufferPeriodic();
            followProfilePeriodic();
        });
    }

    public void setProfiles(Profile[] profiles)
    {
        if (profiles[0].size() != profiles[1].size())
            System.err.println("Error | Profile size mismatch, things will break, fix it please");
        leftMotor.profile = profiles[0]; //Left Profile
        rightMotor.profile = profiles[1]; //Right Profile
    }

    public void initFollowProfile()
    {
        for (HawkTalons v : new HawkTalons[] {leftMotor, rightMotor})
        {
            v.init();
            for (int i = 0; i < 64; i++)
                v.sendNextPoint(); // Get some initial points
        }
        System.out.println("Initializing");
    }

    public void processMotionProfileBufferPeriodic()
    {
        for (HawkTalons v : new HawkTalons[] {leftMotor, rightMotor})
        {
            v.sendNextPoint(); // It's ok if this fails, we won't lose any points
            v.processMotionProfileBuffer(); // Move points from the top buffer to the bottom buffer
        }
        System.out.println("Processing Buffer");
    }

    public void followProfilePeriodic()
    {
        MotionProfileStatus statusL = leftMotor.motionProfileStatus();
        MotionProfileStatus statusR = rightMotor.motionProfileStatus();
        leftMotor.set(ControlMode.MotionProfile, statusL.isLast ? 2 : 1);
        rightMotor.set(ControlMode.MotionProfile, statusR.isLast ? 2 : 1);
        System.out.println("Following Profile");
    }

    public boolean doneWithProfile()
    {
        MotionProfileStatus statusL = leftMotor.motionProfileStatus();
        MotionProfileStatus statusR = rightMotor.motionProfileStatus();
        return statusL.isLast && statusR.isLast;
    }
    public void startFollowing()
    {
        System.out.println("Starting to Follow");
        if (leftMotor.profile == null || rightMotor.profile == null) {
            System.err.println("Error | Call setProfiles before attempting to follow a profile");
            return;
        }
        initFollowProfile();
        System.out.println("Starting notifiers");
        processBuffer.startPeriodic(50/2000);
        System.out.println("Notifiers started");
    }

    public void stopFollowing()
    {
        System.out.println("stopFollowing");
        leftMotor.reset();
        rightMotor.reset();
        processBuffer.stop();
    }
}