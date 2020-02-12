/*----------------------------------------------------------------------------*/

/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */

/* Open Source Software - may be modified and shared by FRC teams. The code   */

/* must be accompanied by the FIRST BSD license file in the root directory of */

/* the project.                                                               */

/*----------------------------------------------------------------------------*/

package frc.robot;

//
import frc.robot.commands.realClimberTest.*;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.buttons.Button;

import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.lib.CreateNewPath;
// import frc.robot.commands.drivetrain.DriveForADistance;
// import frc.robot.commands.drivetrain.RotateToAngle;
// import frc.robot.commands.umbrella.ToggleUmbrella;
// import frc.robot.commands.SetSpeed;
// import frc.robot.commands.Unspool;
import frc.robot.commands.climber.*;
import frc.robot.commands.arm.*;
import frc.robot.commands.intake.*;
import frc.robot.motionprofile.DriveTracker;
import frc.robot.commands.drivetrain.DriveToHatchPanel;
import frc.robot.commands.drivetrain.DriveForADistance;
import frc.robot.commands.drivetrain.DriveToCargo;
import frc.robot.commands.drivetrain.DriveToTarget;
/**
 * 
 * This class is the glue that binds the controls on the physical operator
 * 
 * interface to the commands and command groups that allow control of the robot.
 * 
 */

public class OI {

	//// CREATING BUTTONS

	// One type of button is a joystick button which is any button on a

	//// joystick.

	// You create one by telling it which joystick it's on and which button

	// number it is.

	// Joystick stick = new Joystick(port);

	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,

	// by subclassing Button you can create custom triggers and bind those to

	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS

	// Once you have a button, it's trivial to bind it to a button in one of

	// three ways:

	// Start the command when the button is pressed and let it run the command

	// until it is finished as determined by it's isFinished method.

	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once

	// the button is released.

	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command

	// until it is finished as determined by it's isFinished method.

	// button.whenReleased(new ExampleCommand());

	public static final int AButton = 1;

	public static final int BButton = 2;

	public static final int XButton = 3;

	public static final int YButton = 4;

	public static final int LeftBumper = 5;

	public static final int RightBumper = 6;

	public static final int BackButton = 7;

	public static final int StartButton = 8;

	public static final int LeftStick = 9;
	public static final int RightStick = 10;

	public static final XboxController driverController = new XboxController(0);
	public static final XboxController operatorController = new XboxController(2);

	public static final Button clamper = new JoystickButton(operatorController, YButton);
	public static final Button setSpeedSpooler = new JoystickButton(operatorController, BButton);
	
	public static final Button armBikeBrake = new JoystickButton(driverController, XButton);
	public static final Button setPosition = new JoystickButton(driverController, AButton);


	public static final Button climberTwo = new JoystickButton(driverController, BButton);
	public static final Button RevOuttake = new JoystickButton(driverController, RightBumper);
	public static final Button toggleUmbrella = new JoystickButton(driverController, LeftBumper);

	public static final Button driveToHatchPanel = new JoystickButton(driverController, YButton);

	public static final Button unlockBikebrake = new JoystickButton(operatorController, BButton);
	public static final Button lockBikebrake = new JoystickButton(operatorController, AButton);
	public static final Button frontPistons = new JoystickButton(operatorController, XButton);
	public static final Button backPistons = new JoystickButton(operatorController, YButton);



	public OI() {
		// Unspool.whileHeld(new Unspool());
		// Unspool.whileHeld(new SetSpeed());
		// clamper.whenPressed(new Clamper());
		// setSpeedSpooler.whileHeld(new SetSpeedSpooler());
		
		// armBikeBrake.whenPressed(new Bikebrake());

		// Driver Controller
		climberTwo.whileHeld(new climberTwo()); // B
		// RevOuttake.whileHeld(); // Right Bumper
		// toggleUmbrella.whenPressed(new ToggleUmbrella()); // Left Bumper
		driveToHatchPanel.whileHeld(new DriveToHatchPanel()); // Y

		// Operator Controller
		// unlockBikebrake.whenPressed(new UnlockBikebrake()); // B
		// lockBikebrake.whenPressed(new LockBikebrake()); // A	
		// frontPistons.whenPressed(new FrontPistons()); // X
		backPistons.whenPressed(new DriveTracker(CreateNewPath.pathLeft, CreateNewPath.pathRight)); // Y
	}
}