package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;



public class Climber extends Subsystem {
    public CANSparkMax climberMotorA;
    public CANSparkMax climberMotorB;
    public Climber(){
    climberMotorA = new CANSparkMax(RobotMap.climberMotorPort, MotorType.kBrushless);
    climberMotorB = new CANSparkMax(RobotMap.climberMotorPortTwo, MotorType.kBrushless);
    }
@Override

    public void initDefaultCommand() {

    }

    public void setSpeed (double speed){
// check this if motors are fighting each other (if not moving but making sounds)||(if motor and battery dies) <---- dumb rafeal made this
    climberMotorA.set(speed);
    climberMotorB.set(speed);
   



    }















}