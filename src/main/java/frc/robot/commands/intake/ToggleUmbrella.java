// package frc.robot.commands.intake;

// import edu.wpi.first.wpilibj.command.Command;

// import frc.robot.Robot;
// import frc.robot.RobotMap;

// /**
//  *
//  */
// public class ToggleUmbrella extends Command {
// 	public ToggleUmbrella() {
// 		requires(Robot.pneumatics);
// 	}

// 	// Called just before this Command runs the first time
// 	protected void initialize() {
// 		if (Robot.pneumatics.getUmbrellaState().equals(RobotMap.PistonIn)) {
// 			Robot.pneumatics.setUmbrellaState(RobotMap.PistonOut);
// 		} else {
// 			Robot.pneumatics.setUmbrellaState(RobotMap.PistonIn);
// 		}
// 	}

// 	// Called repeatedly when this Command is scheduled to run
// 	protected void execute() {
// 	}

// 	// Make this return true when this Command no longer needs to run execute()
// 	protected boolean isFinished() {
// 		return true;
// 	}

// 	// Called once after isFinished returns true
// 	protected void end() {
// 	}

// 	// Called when another command which requires one or more of the same
// 	// subsystems is scheduled to run
// 	protected void interrupted() {
// 	}
// }