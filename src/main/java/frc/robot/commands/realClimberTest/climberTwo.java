package frc.robot.commands.realClimberTest;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.subsystems.RealClimber;

/**
 *
 */
public class climberTwo extends Command {

	public climberTwo() {
		requires(Robot.RealClimber);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		// Robot.arm.sparkMaxB.setInverted(true);
		Robot.RealClimber.one.set(0);
        Robot.RealClimber.two.set(0);
        
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	//  SmartDashboard.putNumber("Left Trigger",OI.operatorController.getTriggerAxis(Hand.kLeft));
	//  SmartDashboard.putNumber("Right Trigger",OI.operatorController.getTriggerAxis(Hand.kRight));
	//	Controls speed of motor
	Robot.RealClimber.one.set(0.5);
    Robot.RealClimber.two.set(-0.5);
}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.RealClimber.one.set(0);
        Robot.RealClimber.two.set(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		Robot.RealClimber.one.set(0);
        Robot.RealClimber.two.set(0);
	}
}