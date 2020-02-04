package frc.robot.commands.limelight;

import frc.robot.OI;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import java.lang.module.ModuleDescriptor.Requires;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import edu.wpi.first.wpilibj.Timer;
public class turnToTarget extends Command {
    
   
    private PIDController pid;
    private double angle = 0;
	private Timer timer = new Timer();
	private double val;
    
      
    
    
    public turnToTarget() {
        requires(Robot.m_limelight);
        requires(Robot.m_drivetrain);
        pid = new PIDController(5e-2,1e-6, 0);
        pid.setSetpoint(angle);
        pid.setTolerance(-1f);
        
	}

	// Called just before this Command runs the first time
	protected void initialize() {

		//Robot.m_feeder.feederMotor.set(0);
		 Robot.m_limelight.printTelemetry();
		 timer.reset();
		 timer.start();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
        //this code puts the value of getX() in smartdashboard under "Limelight" when connected to robot
        SmartDashboard.putNumber("limelight", Robot.m_limelight.getX());
        
        //sets value of pid.calculate so that we use this later for drive arcade (feeds it value)
         val = pid.calculate(Robot.m_limelight.getX(), 0);
        //checks this number in smartdashboard
        SmartDashboard.putNumber("val", pid.calculate(Robot.m_limelight.getX(), 0));
        //makes it turn towards the target
        Robot.m_drivetrain.arcadeDrive(0.0, -pid.calculate(Robot.m_limelight.getX(), 0));
		Robot.m_limelight.printTelemetry();
		if (Math.abs(Robot.m_drivetrain.leftMotorA.getEncoder().getVelocity())> 30 && Math.abs(Robot.m_drivetrain.rightMotorA.getEncoder().getVelocity()) > 30) 
			timer.reset();
		
		
		//SmartDashboard.putBoolean()
    }

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		
		return (timer.get() > .7);
	}

	// Called once after isFinished returns true
	protected void end() {
		//Robot.m_feeder.feederMotor.set(0);
		Robot.m_drivetrain.resetDrivetrain();
	}


	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		//Robot.m_feeder.feederMotor.set(0);
				Robot.m_drivetrain.resetDrivetrain();

	}
}
