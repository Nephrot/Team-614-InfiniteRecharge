package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

/**
 * Makes the drivetrain drive straight for a distance in inches
 */
public class DriveForADistance extends Command {
    private double distance, speed;
    private double setpointAngle;

	Timer timer; 
	public DriveForADistance() {
		// Use requires() here to declare subsystem dependencies
        requires(Robot.drivetrain);
        
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.drivetrain.rightMotor1.setSelectedSensorPosition(0);
		// PID Controller Setting the setpoint which
		// just returns the setpoint
        // timer.start();
        Robot.drivetrain.rightMotor1.selectProfileSlot(0, 0);
        Robot.drivetrain.rightMotor1.selectProfileSlot(1, 1);
        Robot.navX.reset();
        Robot.drivetrain.rightMotor1.set(0);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		// Robot.drivetrain.
		// Sets the setpoint at which how far you want the robot to go, it will
		// manipulate the output whenever it needs to
          Robot.drivetrain.straightDrive(400, -Robot.navX.getYaw());
	}

	protected boolean isFinished() {
		return Robot.drivetrain.rightMotor1.getSelectedSensorPosition() > 4000;
	}

	// Called once after isFinished returns true
	protected void end() {
        Robot.drivetrain.rightMotor1.set(0);
        Robot.drivetrain.leftMotor1.set(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
        Robot.drivetrain.rightMotor1.set(0);
        Robot.drivetrain.leftMotor1.set(0);
	}
}