package frc.robot.subsystems.climber;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

public class Climber extends Subsystem {
    public CANSparkMax climberMotorA;
    public CANSparkMax climberMotorB;
    public CANPIDController pidController;
    public DigitalInput limitSwitch;
    //public TrapezoidProfile elevatorTrapezoidalProfile;


    //smart motion = trapezoidal motion profile
    
    public Climber() {
        climberMotorA = new CANSparkMax(RobotMap.climberMotorPort, MotorType.kBrushless);
        climberMotorB = new CANSparkMax(RobotMap.climberMotorPortTwo, MotorType.kBrushless);
        pidController = climberMotorA.getPIDController();
        //elevatorTrapezoidalProfile = new TrapezoidProfile.Constraints(5, 5);
    }


    @Override
    public void initDefaultCommand() {

    }

    // public void setSpeed (double speed){
    // // check this if motors are fighting each other (if not moving but making
    // sounds)||(if motor and battery dies) <---- dumb rafeal made this
    // climberMotorA.set(speed);
    // climberMotorB.set(speed);

    // }
    
    public void setPowerOutputA(double power) {
        climberMotorA.set(power);
    }

    public void setPowerOutputB(double power) {
        climberMotorB.set(power);
    }

    

}