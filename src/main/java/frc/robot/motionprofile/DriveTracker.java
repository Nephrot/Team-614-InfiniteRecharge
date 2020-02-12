package frc.robot.motionprofile;

import frc.robot.motionprofile.ProfileFollower;
import frc.robot.Robot;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTracker extends Command {
	ProfileCommand rightHandler;
	ProfileCommand leftHandler;

	MotionProfileStatus rightStatus;
	MotionProfileStatus leftStatus;

    public DriveTracker(double[][] pathLeft, double[][] pathRight) {
    	requires(Robot.drivetrain);
    	this.setTimeout(15); 
    	rightHandler = new ProfileCommand(Robot.drivetrain.getRightTalon(), pathRight);
    	leftHandler = new ProfileCommand(Robot.drivetrain.getLeftTalon(), pathLeft);
    	Robot.drivetrain.getRightTalon().selectProfileSlot(0, 0);
    	Robot.drivetrain.getLeftTalon().selectProfileSlot(0, 0);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
	    leftHandler.startMotionProfile();
	    rightHandler.startMotionProfile();
    	// Robot.drivetrain.getRightTalon().setInverted(false);
    	// Robot.drivetrain.getRightSlaveVictor().setInverted(false);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	rightStatus = rightHandler.control();
    	leftStatus  = leftHandler.control();

		SmartDashboard.putNumber("Right Set Value", rightHandler.getSetValue().value);
		Robot.drivetrain.getRightTalon().set(ControlMode.MotionProfile, rightHandler.getSetValue().value);
		Robot.drivetrain.getRightSlaveVictor().follow(Robot.drivetrain.getRightTalon());

		SmartDashboard.putNumber("Left Set Value", rightHandler.getSetValue().value);
		Robot.drivetrain.getLeftTalon().set(ControlMode.MotionProfile, leftHandler.getSetValue().value);
		Robot.drivetrain.getLeftSlaveVictor().follow(Robot.drivetrain.getLeftTalon());

		
    	SmartDashboard.putNumber("Right Error: ", Robot.drivetrain.getRightTalon().getClosedLoopError(0));
    	SmartDashboard.putNumber("Left  Error: ", Robot.drivetrain.getRightTalon().getClosedLoopError(0));
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return rightStatus.isLast || leftStatus.isLast || this.isTimedOut();

	}

    // Called once after isFinished returns true
    protected void end() {
    	rightHandler.reset();
    	leftHandler.reset();

    	// Robot.drivetrain.getRightTalon().setInverted(true);
    	// Robot.drivetrain.getRightSlaveVictor().setInverted(true);
    	Robot.drivetrain.getRightTalon().set(ControlMode.Velocity, 0);
    	Robot.drivetrain.getLeftTalon().set(ControlMode.Velocity, 0);
    }

}