package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.GenericHID.Hand;


/**
 *
 */
public class ReverseClimber extends Command {

	public ReverseClimber() {
		requires(Robot.m_climber);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		// Robot.arm.sparkMaxB.setInverted(true);
        Robot.m_climber.climberMotorA.set(0);
        Robot.m_climber.climberMotorB.set(0);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	//  SmartDashboard.putNumber("Left Trigger",OI.operatorController.getTriggerAxis(Hand.kLeft));
    //  SmartDashboard.putNumber("Right Trigger",OI.operatorController.getTriggerAxis(Hand.kRight));
    // If you need to change speed it here
	//Robot.m_climber.climberMotorA.set(SmartDashboard.getNumber("Intake Speed", 0));
    //Robot.m_climber.climberMotorB.set(SmartDashboard.getNumber("Intake Speed", 0));
    Robot.m_climber.climberMotorA.set(-.5);
    Robot.m_climber.climberMotorB.set(.5);
}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
        Robot.m_climber.climberMotorA.set(0);
        Robot.m_climber.climberMotorB.set(0);
		//Robot.arm.hawkTalonA.setSelectedSensorPosition(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
        Robot.m_climber.climberMotorA.set(0);
        Robot.m_climber.climberMotorB.set(0);
		//Robot.arm.hawkTalonA.setSelectedSensorPosition(0);
	}
}