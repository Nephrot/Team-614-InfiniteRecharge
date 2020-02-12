// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.robot.commands.climber;

// import edu.wpi.first.wpilibj.GenericHID.Hand;
// import edu.wpi.first.wpilibj.command.Command;

// import frc.robot.OI;
// import frc.robot.Robot;
// import frc.robot.RobotMap;

// /**
//  *
//  */
// public class BackPistons extends Command {
// 	public BackPistons() {
// 		// Use requires() here to declare subsystem dependencies
// 		// eg. requires(chassis);
// 		requires(Robot.pneumatics);
// 	}

// 	// Called just before this Command runs the first time
// 	protected void initialize() {
// 		if (Robot.pneumatics.getClamperAAState().equals(RobotMap.PistonIn)) {
// 			Robot.pneumatics.setClamperAAState(RobotMap.PistonOut);
// 			Robot.pneumatics.setClamperBBState(true);
// 		} else {
// 			Robot.pneumatics.setClamperAAState(RobotMap.PistonIn);
// 			Robot.pneumatics.setClamperBBState(false);

// 		}
// 	}

// 	// Called repeatedly when this Command is scheduled to run
// 	protected void execute() {
// 		// Robot.climber.setSpeedBoth(OI.operatorController.getY(Hand.kLeft));
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