package frc.robot.motionprofile;

import com.ctre.phoenix.motion.TrajectoryPoint;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;
import frc.lib.*;

public class Profile {
    private double[][] pointData;
    private TrajPointReading profile;
    private double driveTicksPerInch = 0.0001;
    public Profile(double[][] pointData) {
        this.pointData = pointData;
    }
    
    public TrajectoryPoint getPoint(int index) {
         TrajectoryPoint point = new TrajectoryPoint();
         if(index >= pointData.length || index < 0) {
             System.err.println("Error | Attempted to get a point that shouldn't exist");
             return null;
         }
         point.position = (driveTicksPerInch * 12) * pointData[index][0];
         point.velocity = ((driveTicksPerInch * 12) * pointData[index][1]);
        //  point.auxiliaryVel = pointData[index][3] * 0.25;
         point.profileSlotSelect0 = 0;
         point.profileSlotSelect1 = 0;
         profile = new TrajPointReading(pointData);
         point.timeDur = (int)(profile.getDuration() * 1000);
         point.zeroPos = index == 0;
         point.isLastPoint = index == (pointData.length - 1);
         System.out.println("Success | Point was created successfully");
         return point;
    }

    public int size()
    {
        return pointData.length;
    }
}