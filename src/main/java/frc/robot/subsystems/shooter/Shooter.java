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
  public TimeOfFlight tofsensor;

  public CANSparkMax acceleratorMotor;
  private CANPIDController shooterPIDController;
  private CANEncoder encoder;

  public Shooter() {
    shooterMotor = new CANSparkMax(RobotMap.shooterMotorPort, MotorType.kBrushless);
    acceleratorMotor = new CANSparkMax(RobotMap.acceleratorMotorPort, MotorType.kBrushless);
    followShooter();

    tofsensor = new TimeOfFlight(1);
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
    if(shooterPIDController.getP() != SmartDashboard.getString("Shooter: P Value", RobotMap.shooterPValue)) {
      shoooterPIDController.setP(SmartDashboard.getString("Shooter: P Value", RobotMap.shooterPValue));
    } else if (shooterPIDController.getI() != SmartDashboard.getString("Shooter: I Value", RobotMap.shooterIValue)) {
      shoooterPIDController.setI(SmartDashboard.getString("Shooter: I Value", RobotMap.shooterIValue));
    } else if (shooterPIDController.getD() != SmartDashboard.getString("Shooter: D Value", RobotMap.shooterDValue)) {
      shoooterPIDController.setD(SmartDashboard.getString("Shooter: D Value", RobotMap.shooterDValue));
    } else if (shooterPIDController.getFF() != SmartDashboard.getString("Shooter: FF Value", RobotMap.shooterFFValue)) {
      shoooterPIDController.setFF(SmartDashboard.getString("Shooter: FF Value", RobotMap.shooterFFValue));
    } else if (shooterPIDController.getIZ() != SmartDashboard.getString("Shooter: IZ Value", RobotMap.shooterIZValue)) {
      shoooterPIDController.setIZ(SmartDashboard.getString("Shooter: IZ Value", RobotMap.shooterIZValue));
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