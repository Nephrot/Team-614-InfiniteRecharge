package frc.robot.commands.limelight;

import frc.robot.OI;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import java.lang.module.ModuleDescriptor.Requires;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.controller.PIDController;
import com.revrobotics.CANSparkMax;
import frc.robot.subsystems.vision.Vision;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.chassis.OutputCalculator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class turnToTarget extends Command {
    
   
    private PIDController pid;
    private double angle = 0;
  
    
      
    
    
    public turnToTarget() {
        requires(Robot.m_limelight);
        requires(Robot.m_drivetrain);
        pid = new PIDController(5e-5,1e-6, 0);
        pid.setSetpoint(angle);
        pid.setTolerance(0.1f);
	}

	// Called just before this Command runs the first time
	protected void initialize() {

		//Robot.m_feeder.feederMotor.set(0);

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
        double val = pid.calculate(Robot.m_limelight.getX(), angle);
        Robot.m_drivetrain.arcadeDrive(0.0, val);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		//Robot.m_feeder.feederMotor.set(0);

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		//Robot.m_feeder.feederMotor.set(0);
	}
}
