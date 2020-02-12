/*----------------------------------------------------------------------------*/

/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */

/* Open Source Software - may be modified and shared by FRC teams. The code   */

/* must be accompanied by the FIRST BSD license file in the root directory of */

/* the project.                                                               */

/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.subsystems.chassis.SRXPID;

/**
 * 
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * 
 * to a variable name. This provides flexibility changing wiring, makes checking
 * 
 * the wiring easier and significantly reduces the number of magic numbers
 * 
 * floating around.
 * 
 */

public class RobotMap {

	// For example to map the left and right motors, you could define the

	// following variables to use with your drivetrain subsystem.

	// public static int leftMotor = 1;

	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port

	// number and the module. For example you with a rangefinder:

	// public static int rangefinderPort = 1;

	// public static int rangefinderModule = 1;

	// MOTOR CONTROLLERS

	public static final int sparkMax2 = 1; // LDrive
	public static final int sparkMax1 =2;

	public static final int rightMotorA = 3; // RDrive
	public static final int rightMotorB = 0;

	public static final int sparkMaxA = 5; // Arm
	public static final int sparkMaxB = 6;

	public static final int sparkMaxC = 7; // Intake
	public static final int sparkMaxD = 8;

	public static final int sparkMaxE = 9; // Climber
	public static final int sparkMaxF = 10;

	// DRIVETRAIN CONSTANTS

	public static final double DRIVETRAIN_WHEEL_DIAMETER = 6; // Fix
	public static final double DRIVETRAIN_ENCODER_PULSES_PER_REV = 245; // Fix

	public static final double inchesToTicksEquation = (1400) / (2 * Math.PI * (DRIVETRAIN_WHEEL_DIAMETER / 2)); // Fix
	public static final double degreesToTicksConstant = 42 / 180;

	public static final double DRIVETRAIN_DISTANCE_PER_PULSE = 1.258
			* ((RobotMap.DRIVETRAIN_WHEEL_DIAMETER * Math.PI) / RobotMap.DRIVETRAIN_ENCODER_PULSES_PER_REV);

	public static final double maxVelocity = 0;

	public static final double maxAcceleration = 0;

	public static final double kCollisionThreshold_DeltaG = 0.6f;

	// ENCODERS

	// public static final int leftMotorEncoderA = 0;
	// public static final int leftMotorEncoderB = 1;
	// NOT USED

	// MOTION MAGIC VALUES


	public static final double drivetrainRotationP = 4;
	public static final double drivetrainRotationI = 0;
	public static final double drivetrainRotationD = 6;
	public static final double drivetrainRotationF = 0;

	public static final double drivetrainDistanceP = 0.014;
	public static final double drivetrainDistanceI = 0.001;
	public static final double drivetrainDistanceD = 6;
	public static final double drivetrainDistanceF = 0.08;

	public static final double p = 1;
	public static final double f = 0.01;

	// PNEUMATICS
	public static final int solenoidBeakA = 0;
	public static final int solenoidBeakB = 1;
	public static final int solenoidBrakeA = 2;
	public static final int solenoidBrakeB = 3;
	public static final int deployClampA = 4;
	public static final int deployClampB = 5;
	public static final int climberPistonA = 6;
	public static final int climberPistonB = 7;
	public static final int compressor = 0;

	public static final double nativeUpdates = 100;

	public static final int dioServoPort1 = 0;
	public static final int dioServoPort2 = 0;


	public static final DoubleSolenoid.Value PistonOut = DoubleSolenoid.Value.kForward;
	public static final DoubleSolenoid.Value PistonIn = DoubleSolenoid.Value.kReverse;

	public static final int PIDLoopIdx = 0;
	public static final int timeoutMs = 0;
	public static final int slotIdx = 0;

	public static final int encTicksPerDeg = 1920 / 90;
	public static final int degPerEncTicks = 90 / 1920;

	// Distance PID
	public static final double distP = 0.1;
	public static final double distI = 0;
	public static final double distD = 0;
	public static final double distF = 0;
	public static final int distIZone = 100;
	public static final double distPeakOutput = 1;
	
	// Turn PID
	public static final double turnP = 2;
	public static final double turnI = 0;
	public static final double turnD = 4;
	public static final double turnF = 0;
	public static final int turnIZone = 200;
	public static final double turnPeakOutput = 1;

	public static SRXPID turnGains = new SRXPID(turnF, turnP, turnI, turnD, turnIZone, turnPeakOutput);

	// Velocity PID
	// public static final double velP = 0.1;
	// public static final double velI = 0;
	// public static final double velD = 20;
	// public static final double velF = (1023/6800);
	// public static final int velIZone = 300;
	// public static final double velPeakOutput = 0.5;

	// Motion Profile PID
	// public static final double mpP = 1;
	// public static final double mpI = 0;
	// public static final double mpD = 0;
	// public static final double mpF = (1023/6800);
	// public static final int mpIZone = 400;
	// public static final double mpPeakOutput = 1;
}