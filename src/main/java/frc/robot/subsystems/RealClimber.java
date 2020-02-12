package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;



public class RealClimber extends Subsystem {

    public CANSparkMax one = new CANSparkMax(RobotMap.sparkMax1, MotorType.kBrushed);
    public CANSparkMax two = new CANSparkMax(RobotMap.sparkMax2, MotorType.kBrushed);

@Override

    public void initDefaultCommand() {

    }

    public void setSpeed (double speed){
    one.set(speed);
    two.set(speed);;



    }















}