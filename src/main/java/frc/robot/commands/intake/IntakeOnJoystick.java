package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 *
 */
public class IntakeOnJoystick extends Command {
	public IntakeOnJoystick() {
		requires(Robot.m_intake);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.m_intake.intakeMotor.set(0);		
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	 Robot.m_intake.intakeMotor.set(OI.intakeController.getY(Hand.kLeft));
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.m_intake.intakeMotor.set(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		Robot.m_intake.intakeMotor.set(0);
	}

}