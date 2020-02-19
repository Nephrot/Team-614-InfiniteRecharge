package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

import frc.robot.subsystems.vision.Vision;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANEncoder;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.playingwithfusion.TimeOfFlight;

public class Shooter extends Subsystem {
  public CANSparkMax shooterMotor;
  public TimeOfFlight tofSensorShooter;

  public CANSparkMax acceleratorMotor;
  private CANPIDController shooterPIDController;
  private CANEncoder encoder;

  public Shooter() {
    shooterMotor = new CANSparkMax(RobotMap.shooterMotorPort, MotorType.kBrushless);
    acceleratorMotor = new CANSparkMax(RobotMap.acceleratorMotorPort, MotorType.kBrushless);
    followShooter();

    tofSensorShooter = new TimeOfFlight(1);
    shooterPIDController = shooterMotor.getPIDController();
    encoder = shooterMotor.getEncoder();

    shooterPIDController.setP(RobotMap.shooterPValue);
    shooterPIDController.setI(RobotMap.shooterIValue);
    shooterPIDController.setD(RobotMap.shooterDValue);
    shooterPIDController.setIZone(RobotMap.shooterIZValue);
    shooterPIDController.setFF(RobotMap.shooterFFValue);
    shooterPIDController.setOutputRange(RobotMap.minOutput, RobotMap.maxOutput);
  }

  public void initDefaultCommand() {
    // setDefaultCommand(new VisionProcessing());
  }

  public void setShooterSpeed(double speed) {
    shooterMotor.set(speed);
  }

  public void setAcceleratorSpeed(double speed) {
    acceleratorMotor.set(speed);
  }

  public void createTelemetry() {
    SmartDashboard.putNumber("Shooter: P Value", RobotMap.shooterPValue);
    SmartDashboard.putNumber("Shooter: I Value", RobotMap.shooterIValue);
    SmartDashboard.putNumber("Shooter: D Value", RobotMap.shooterDValue);
    SmartDashboard.putNumber("Shooter: FF Values", RobotMap.shooterFFValue);
    SmartDashboard.putNumber("Shooter: IZ Value", RobotMap.shooterIZValue);
  }

  public void updateTelemetry() {
    if(shooterPIDController.getP() != SmartDashboard.getNumber("Shooter: P Value", RobotMap.shooterPValue)) {
      shooterPIDController.setP(SmartDashboard.getNumber("Shooter: P Value", RobotMap.shooterPValue));
    } else if (shooterPIDController.getI() != SmartDashboard.getNumber("Shooter: I Value", RobotMap.shooterIValue)) {
      shooterPIDController.setI(SmartDashboard.getNumber("Shooter: I Value", RobotMap.shooterIValue));
    } else if (shooterPIDController.getD() != SmartDashboard.getNumber("Shooter: D Value", RobotMap.shooterDValue)) {
      shooterPIDController.setD(SmartDashboard.getNumber("Shooter: D Value", RobotMap.shooterDValue));
    } else if (shooterPIDController.getFF() != SmartDashboard.getNumber("Shooter: FF Value", RobotMap.shooterFFValue)) {
      shooterPIDController.setFF(SmartDashboard.getNumber("Shooter: FF Value", RobotMap.shooterFFValue));
    } else if (shooterPIDController.getIZone() != SmartDashboard.getNumber("Shooter: IZ Value", RobotMap.shooterIZValue)) {
      shooterPIDController.setIZone(SmartDashboard.getNumber("Shooter: IZ Value", RobotMap.shooterIZValue));
    }
  }

  public void setShooterReference(double setPoint) {
    shooterPIDController.setReference(setPoint, ControlType.kVelocity);
    SmartDashboard.putNumber("Shooter: Process Variable", encoder.getVelocity());
  }

  public void followShooter() {
    acceleratorMotor.follow(shooterMotor);
  }

  // public void setIntakeSpeed(double speed) {
  // intakeMotor.set(speed);
  // }

}