package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.drivetrain.DriveToTarget;

public class DriveToHatchPanel extends CommandGroup {
    public DriveToHatchPanel() {
		addSequential(new DriveToTarget(0, 0));
	}
}