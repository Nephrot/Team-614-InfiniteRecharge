package frc.robot.commands.chassis;

import frc.robot.OI;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ModifiedArcadeDrive extends Command {
	public ModifiedArcadeDrive() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.m_drivetrain);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.m_drivetrain.arcadeDrive(OI.driverController.getY(Hand.kLeft) < 0 ? .8 * -Math.sqrt(OI.driverController.getY(Hand.kLeft)) : .8 * Math.sqrt(OI.driverController.getY(Hand.kLeft)), (OI.driverController.getX(Hand.kRight) < 0 ? .8 * -Math.sqrt(OI.driverController.getX(Hand.kRight)) : .8 * Math.sqrt(OI.driverController.getX(Hand.kRight))));
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
