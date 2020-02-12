package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.drivetrain.DriveToTarget;

public class DriveToCargo extends CommandGroup {
    public DriveToCargo() {
		addSequential(new DriveToTarget(1, 0));
	}
}