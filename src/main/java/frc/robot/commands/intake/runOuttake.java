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
public class runOuttake extends Command {
	public runOuttake() {
		requires(Robot.m_intake);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		// Robot.arm.sparkMaxB.setInverted(true);
		Robot.m_intake.intakeMotor.set(0);
		Robot.m_intake.toggleDoubleSolenoidA();
		Robot.m_intake.toggleDoubleSolenoidB();
		Robot.m_serializer.serializerMotorA.set(0);
		Robot.m_serializer.serializerMotorB.set(0);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute(double speed) {
	//  SmartDashboard.putNumber("Left Trigger",OI.operatorController.getTriggerAxis(Hand.kLeft));
	//  SmartDashboard.putNumber("Right Trigger",OI.operatorController.getTriggerAxis(Hand.kRight));
     //Robot.m_intake.intakeMotor.set(-1.0 * SmartDashboard.getNumber("Intake Speed", 0));
	 Robot.m_intake.intakeMotor.set(-.5);
	 Robot.m_serializer.serializerMotorA.set(-speed);
	 Robot.m_serializer.serializerMotorB.set(-speed);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.m_intake.intakeMotor.set(0);		
		Robot.m_intake.untoggleDoubleSolenoidA();
		Robot.m_intake.untoggleDoubleSolenoidB();
		Robot.m_serializer.serializerMotorA.set(0);
		Robot.m_serializer.serializerMotorB.set(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		Robot.m_intake.intakeMotor.set(0);		
		Robot.m_intake.untoggleDoubleSolenoidA();
		Robot.m_intake.untoggleDoubleSolenoidB();
		Robot.m_serializer.serializerMotorA.set(0);
		Robot.m_serializer.serializerMotorB.set(0);
	}
}