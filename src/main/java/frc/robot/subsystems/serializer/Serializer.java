package frc.robot.subsystems.serializer;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Serializer extends Subsystem {
  public CANSparkMax serializerMotorA;
  public CANSparkMax serializerMotorB;

  public Serializer() {
    serializerMotorA = new CANSparkMax(RobotMap.serializerMotorPortA, MotorType.kBrushless);
    serializerMotorB = new CANSparkMax(RobotMap.serializerMotorPortB, MotorType.kBrushless);
  }

  public void initDefaultCommand() {
    // setDefaultCommand(new VisionProcessing());
  }

  public void setSerializerSpeedA(double speed) {
    serializerMotorA.set(speed);
  }

  public void setSerializerSpeedB(double speed) {
    serializerMotorB.set(speed);
  }

  public void togetherSerializerSpeeds(double speed) {
    setSerializerSpeedA(speed);
    setSerializerSpeedB(speed < 0 ? -Math.abs(speed - .2) : Math.abs(speed - .2));
  }

  public double getCurrentA() {
    return serializerMotorA.getOutputCurrent();
  }

  public double getcurrentB() {
    return serializerMotorB.getOutputCurrent();
  }

  public boolean checkCurrentA(double current) {
    return serializerMotorA.getOutputCurrent() > current ? true : false;
  }

  public boolean checkCurrentB(double current) {
    return serializerMotorB.getOutputCurrent() > current ? true : false;
  }
  // if current above certain amount, do equations instead

  public void runMotorFunction(double speed) {
       if(!checkCurrentA(RobotMap.setCurrent) && !checkCurrentB(RobotMap.setCurrent))
       {
        setSerializerSpeedA(speed);
        setSerializerSpeedB(speed < 0 ? -Math.abs(speed - .2) : Math.abs(speed - .2));
       }
       else if(checkCurrentA(RobotMap.setCurrent))
       {
         setSerializerSpeedA(-speed);
         setSerializerSpeedB(speed < 0 ? -Math.abs(speed - .2) : Math.abs(speed - .2));
       }
       else
       {
         setSerializerSpeedB(speed);
         setSerializerSpeedA(speed < 0 ? Math.abs(speed - .2) : -Math.abs(speed - .2));
       }
  }
}
