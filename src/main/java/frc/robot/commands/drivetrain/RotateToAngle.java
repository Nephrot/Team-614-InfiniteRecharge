// package frc.robot.commands.drivetrain;

// import edu.wpi.first.wpilibj.command.Command;
// import frc.robot.Robot;

// // 	NAVX DEGREE ORIENTATION:
// // 			 0
// // 	    -45 \|/ +45
// // 	  -90  --X--  +90
// //	   -135 /|\ +135
// //		  +/-180

// public class RotateToAngle extends Command {

// 	double angle = 0;
// 	boolean useAbsoluteAngle = false;

// 	public RotateToAngle(double angle, boolean useAbsoluteAngle) {
// 		// Use requires() here to declare subsystem dependencies
// 		// eg. requires(chassis);
// 		requires(Robot.drivetrain);
// 		this.angle = angle;
// 		this.useAbsoluteAngle = useAbsoluteAngle;
// 	}

// 	// Called just before this Command runs the first time
// 	protected void initialize() {
// 		// Robot.navX.reset();
// 		// Robot.navX.zeroYaw();

// 		Robot.drivetrain.setUsingTurnPID(true);

// 		if (useAbsoluteAngle) {
// 			Robot.drivetrain.getTurnController().setSetpoint(angle % 360);
// 		} else { // relative angle
// 			Robot.drivetrain.getTurnController().setSetpoint((Robot.navX.getYaw() + angle));// % 360);
// 		}
// 	}

// 	// Called repeatedly when this Command is scheduled to run
// 	protected void execute() {
// 		Robot.drivetrain.arcadeDrive(0.0, .8 * Robot.drivetrain.getPIDRotateRate());
// 		// Robot.drivetrain.arcadeDrive(0.0, Robot.drivetrain.getPIDRotateRate());
// 	}

// 	// Make this return true when this Command no longer needs to run execute()
// 	protected boolean isFinished() {
// 		// if (timeSinceInitialized() > 1.5) {
// 		if (Robot.drivetrain.leftEncoder.getRate() < 2.5 && Robot.drivetrain.leftEncoder.getRate() > -2.5) {
// 			return true;
// 		}
// 		// }
// 		return false;
// 	}

// 	// Called once after isFinished returns true
// 	protected void end() {
// 		Robot.drivetrain.setUsingTurnPID(false);
// 		Robot.drivetrain.stop();
// 	}

// 	// Called when another command which requires one or more of the same
// 	// subsystems is scheduled to run
// 	protected void interrupted() {
// 		Robot.drivetrain.setUsingTurnPID(false);
// 		Robot.drivetrain.stop();
// 	}
// }