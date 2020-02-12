package frc.robot.motionprofile;



import com.ctre.phoenix.motorcontrol.ControlMode;

import com.ctre.phoenix.motorcontrol.DemandType;

import com.ctre.phoenix.motorcontrol.FollowerType;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.chassis.Drivetrain;

public class AbsoluteMotionMagic extends Command {
	private double encoderTicks;
	private double targetHeading;
	private boolean turning;
	private int timeoutCtr;
	private Timer time;

	private final static double DISTANCE_FINISH_THRESHOLD = 4000; 
	private final static double TURNING_FINISH_THRESHOLD = 1.5;
	private final static double DISTANCE_TIMER_THRESHOLD = 10000; 
	private final static double TURNING_TIMER_THRESHOLD = 8.0;
	private final static double TIMER_THRESHOLD = 0.5; 

	public static double limelightRawDistance;

	public static double limelightAngle;
	public static double limelightY;
	public static double limelightX = limelightY / (Math.toDegrees(Math.tan(limelightAngle)));
	public static double limelightH = limelightY / (Math.toDegrees(Math.sin(limelightAngle)));
	public static double limelightDistance = limelightH * 1.54;

	public double beforeAngle;
	public double multiplier = 10;
	public boolean limelightCompleted = false;

    public AbsoluteMotionMagic(double inches, double absoluteDegrees, boolean isTurnMotion) {
		this.encoderTicks =  (limelightDistance * RobotMap.DRIVETRAIN_ENCODER_PULSES_PER_REV) * multiplier;
		this.targetHeading = absoluteDegrees;
		this.turning = isTurnMotion;
		requires(Robot.drivetrain);
		//System.out.println("DriveByMotionMagicAbsolute: constructed");
	}
	
    // Called just before this Command runs the first time
    protected void initialize() {
		time.reset();
		time.start(); //Fix
		Robot.drivetrain.getRightTalon().configMotionCruiseVelocity(20000); // Sensor Units per 100 ms 1/10 of a second
		Robot.drivetrain.getRightTalon().configMotionAcceleration(10000); // Sensor Units per 100 ms 1/10 of a second
    	System.out.println("Log: Motion Magic Configured");
		Robot.drivetrain.zeroSensors();
		targetHeading = limelightAngle;  
		Robot.navX.getYaw(); // Angle
        encoderTicks = (limelightDistance * RobotMap.DRIVETRAIN_ENCODER_PULSES_PER_REV) * multiplier; // Distance
    	double inches = (encoderTicks * (6 * Math.PI))/RobotMap.DRIVETRAIN_ENCODER_PULSES_PER_REV;
    	System.out.println("Log: Inches, Heading, IsTurnMotion: " + inches + " " + targetHeading + " " + turning);
		Robot.drivetrain.getRightTalon().set(ControlMode.MotionMagic, 2 * encoderTicks, DemandType.AuxPID, 10 * targetHeading);
		Robot.drivetrain.getLeftTalon().follow(Robot.drivetrain.getRightTalon(), FollowerType.AuxOutput1);
		System.out.println("Log: running profile...");
		timeoutCtr = 0;
	}
	
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
		targetHeading = limelightAngle + (Robot.navX.getYaw() - beforeAngle);
	    beforeAngle = Robot.navX.getYaw();
	    encoderTicks = limelightDistance;
		Robot.drivetrain.getRightTalon().set(ControlMode.MotionMagic, 2 * encoderTicks, DemandType.AuxPID, 10 * targetHeading);
		Robot.drivetrain.getLeftTalon().follow(Robot.drivetrain.getRightTalon(), FollowerType.AuxOutput1);	
    	// if (!turning) //Better Checkers
    	// {
    	// 	double currentTicks = Robot.drivetrain.rightMasterMotor.getSelectedSensorPosition();
		// 	double error = Math.abs(encoderTicks - currentTicks);
		
		// 	if (error < DISTANCE_TIMER_THRESHOLD)
		// 	     timeoutCtr++;
		// }
		
    	// else //Better Checkers
    	// {
    	// 	double currentHeading = Robot.navX.getYaw();
    	// 	double error = Math.abs(targetHeading - currentHeading);
    	// 	//System.out.println("DriveByMotionMagicAbsolute: turning error = " + error);
    	// 	if (error < TURNING_TIMER_THRESHOLD) timeoutCtr++;
		// }
		// Disregard this
	}
	
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	// if (timeoutCtr > (TIMER_THRESHOLD * 50))
		// {
    	// 	System.out.println("DriveByMotionMagicAbsolute: timeout reached");
    	// 	return true;
		// } //It messed up?
    	// else//if trying to drive straight
    	// {
    	// 	double currentTicks = Robot.drivetrain.rightMasterMotor.getSelectedSensorPosition();
    	// 	double error = Math.abs(encoderTicks - currentTicks);
    	// 	//System.out.println("DriveByMotionMagicAbsolute: distance error = " + error);
    	// 	if (error < DISTANCE_FINISH_THRESHOLD)
    	// 	{
    	// 		System.out.println("DriveByMotionMagicAbsolute: encoder ticks reached");
        // 		return true;
    	// 	}
		// 	else 
		// 	    return false;

		// }
		// Disregard this
		return limelightCompleted;
    }



    // Called once after isFinished returns true

    protected void end() {
    	double currentTicks = Robot.drivetrain.getRightPos();
		double ticksError = Math.abs(encoderTicks - currentTicks);
		double inches = (encoderTicks * (6 * Math.PI)) /RobotMap.DRIVETRAIN_ENCODER_PULSES_PER_REV;
		double currentHeading = Robot.navX.getYaw();
		double degreesError = Math.abs(targetHeading - currentHeading);

    	System.out.println("Completed: ended. Error = " + inches/2 + " inches, " + degreesError + " degrees, " + time + " seconds");
    	Robot.drivetrain.zeroSensors();
    }



    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	System.out.println("DriveByMotionMagicAbsolute: interrupted");
    	end();
    }
}