            /*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;

  //Controller
  //public static final XboxController intakeController;

  //Chassis
  public static final int leftMotorAPort = 1;
  public static final int leftMotorBPort = 2;
  public static final int rightMotorAPort = 3;
  public static final int rightMotorBPort = 4;
  public static final MotorType brushless = MotorType.kBrushless;
  public static final MotorType brushed = MotorType.kBrushed;
  public static final double ticksInARevolution = 10.659;
  public static final double wheelDiameter = 6;
  public static final double pValue = 0.028;
  public static final double iValue = 0;
  public static final double dValue = 0;
  public static final double chassisVelocityPValue = 0;
  public static final double chassisVelocityIValue = 1;
  public static final double chassisVelocityDValue = 2;
  public static final double chassisVelocityFFValue = 2;
  public static final double chassisVelocityIZValue = 2;
  public static final double chassisVelocityMinOutput = 2;
  public static final double chassisVelocityMaxOutput = 2;
  public static double vValue = 0.08;
  public static final double hValue = -0.01;

  //Shooter
  public static final double shooterPValue = 5e-5;
  public static final double shooterIValue = 0;
  public static final double shooterDValue = 0;
  public static final double shooterIZValue = 0;
  public static final double shooterFFValue = 0;
  public static final double maxOutput = 0.8;
  public static final double minOutput = -0.8;
  public static final double maxRPM = 5700;
  public static final int shooterMotorPort = 6;
  public static final int acceleratorMotorPort = 5;
  public static double lowGoalSpeed = 1300;
  public static double highGoalSpeed = 2000;

  //Climber
  public static final int limitSwitchPortA = 0;
  public static final int limitSwitchPortB = 1;
  public static final int limitSwitchPortC = 2;
  public static final int limitSwitchPortD = 3;
  public static final int climberMotorPort = 7;
  public static final double climberPValue = 5e-5;
  public static final double climberIValue = 0;
  public static final double climberDValue = 0;
  public static final double climberIZValue = 0;
  public static final double climberFFValue = 0;
  public static final double climberMinOutput = 0;
  public static final double climberMaxOutput = 0;
  public static final double climberMaxVel = 0;
  public static final double climberMaxAcc = 0;
  public static final double climberMinVel = 0;
  public static final double climberAllowedErr = 0;
  public static final int climberSmartMotionPort = 0;
  public static final int climberPistonPortA = 0;
  public static final int climberPistonPortB = 0;
  
  //Intake
  public static final int intakeMotorPort = 8;
  public static final int doubleSolenoidAPort1 = 1;
  public static final int doubleSolenoidAPort2 = 2;
  public static final int doubleSolenoidBPort1 = 3;
  public static final int doubleSolenoidBPort2 = 4;

  //Vision 
  public static final double limelightToTarget = 25.7;
  public static final double vPValue = 0.1;
  public static final double vPAltValue = 0.023;
  public static final double vFFValue = 0.35;
  public static final double vMaxOutput = 0.5;
  public static final double highGoalDistance = 74.4;
  public static final DoubleSolenoid.Value PistonOut = DoubleSolenoid.Value.kForward;
	public static final DoubleSolenoid.Value PistonIn = DoubleSolenoid.Value.kReverse;
  
  //pneumatics
	public static final int compressor = 0;

  //Misc
 

  //Controller.
  public static final int AButton = 1;
	public static final int BButton = 2;
	public static final int XButton = 3;
	public static final int YButton = 4;
	public static final int LeftBumper = 5;
	public static final int RightBumper = 6;
	public static final int BackButton = 7;
	public static final int StartButton = 8;
	public static final int LeftStick = 9;
  public static final int RightStick = 10;
  public static final int driverPort = 0;
  public static final int operatorPort = 2;

  //Serializer
  public static final int serializerMotorPortA = 9;
  public static final int serializerMotorPortB = 10;
  public static final int setCurrent = 30;

  //Feeder
  public static final int feederMotorPort = 1;
  public static final double feederPValue = 5e-5;
  public static final double feederIValue = 0;
  public static final double feederDValue = 0;
  public static final double feederIZValue = 0;
  public static final double feederFFValue = 0;
}
