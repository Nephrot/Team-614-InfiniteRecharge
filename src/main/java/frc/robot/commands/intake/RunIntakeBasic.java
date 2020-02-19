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
public class RunIntakeBasic extends Command {
    double speed;
	public RunIntakeBasic(double speed) {
        requires(Robot.m_intake);
        this.speed = speed;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
        Robot.m_intake.intakeMotor.set(0);
        Robot.m_intake.setDoubleSolenoidA(Robot.m_intake.pistonOut);
        Robot.m_intake.setDoubleSolenoidB(Robot.m_intake.pistonOut);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.m_intake.intakeMotor.set(speed);
	}

	// Make this return true when this Command no lo
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.m_intake.intakeMotor.set(0);
		Robot.m_intake.setDoubleSolenoidA(Robot.m_intake.pistonIn);
        Robot.m_intake.setDoubleSolenoidB(Robot.m_intake.pistonIn);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		Robot.m_intake.intakeMotor.set(0);
		Robot.m_intake.setDoubleSolenoidA(Robot.m_intake.pistonIn);
        Robot.m_intake.setDoubleSolenoidB(Robot.m_intake.pistonIn);
    }
    
    // protected void pistonToggle() {
    //     Robot.m_intake.toggleDoubleSolenoidA();
    //     Robot.m_intake.toggleDoubleSolenoidB();
    // }
}