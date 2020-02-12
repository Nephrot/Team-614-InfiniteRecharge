/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import com.fasterxml.jackson.databind.deser.impl.SetterlessProperty;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.autonomous.LeftPath;
import frc.robot.commands.chassis.FollowPath;
import frc.robot.commands.shooter.DeliverGoalHigh;
import frc.robot.commands.shooter.DeliverGoalLow;
import frc.robot.commands.shooter.ShootForATime;
import frc.robot.commands.misc.BlinkForATime;
import frc.robot.commands.shooter.SetSpeed;
import frc.robot.commands.shooter.SetSpeedButton;
import frc.robot.commands.chassis.RotateToAngle;
import frc.robot.commands.climber.RunClimber;
import frc.robot.commands.feeder.RunFeeder;
import frc.robot.commands.intake.runIntake;
import frc.robot.commands.limelight.*;
import frc.robot.commands.climber.ReverseClimber;
import frc.robot.commands.intake.runOuttake;
/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
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


  // y button
  public static final Button clamper = new JoystickButton(operatorController, YButton);
  
  //b button
	public static final Button setSpeedSpooler = new JoystickButton(operatorController, BButton);
  
  //x button
  public static final Button runOuttake = new JoystickButton(driverController, XButton);
  
  //A button
	public static final Button setPosition = new JoystickButton(driverController, AButton);

  //b button
  public static final Button RunClimber = new JoystickButton(driverController, BButton);

  //a button
  public static final Button ReverseClimber = new JoystickButton(driverController, AButton);
  

  //y button
  public static final Button runIntake = new JoystickButton(driverController, YButton);
  
  //Left bumper
  public static final Button toggleUmbrella = new JoystickButton(driverController, LeftBumper);
  
  public static final Button turnToTarget = new JoystickButton(driverController, RightBumper);

	


  // public static final Button followPath = new JoystickButton(driverController, RobotMap.YButton);
  //public static final Button coastMode = new JoystickButton(driverController, RobotMap.RightBumper);

  public OI() {
    // followPath.whenPressed(new LeftPath());
    //coastMode.whileHeld(new SetSpeed(0.5));
    RunClimber.whileHeld(new RunFeeder());
  };
}
