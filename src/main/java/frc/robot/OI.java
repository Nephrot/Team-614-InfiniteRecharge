/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//poopy butt face
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
import frc.robot.commands.chassis.RotateToAngle;
import frc.robot.commands.intake.RunIntakeBasic;
import frc.robot.commands.intake.SetPistonIn;
import frc.robot.commands.intake.SetPistonOut;
import frc.robot.commands.limelight.*;
import frc.robot.commands.intake.IntakeToggle;
import frc.robot.commands.intake.IntakeSetValue;
import frc.robot.commands.intake.IntakeOnSmartDashboard;
import frc.robot.commands.serializer.RunSerializerA;
import frc.robot.commands.serializer.RunSerializerB;
import frc.robot.commands.serializer.RunSerializerAdvanced;
import frc.robot.commands.serializer.RunSerializerBasic;
import frc.robot.commands.serializer.SerializerAutomated;
import frc.robot.commands.serializer.SerializerOnSmartDashboard;
import frc.robot.commands.climber.BrakePistonIn;
import frc.robot.commands.climber.BrakePistonOut;
import frc.robot.commands.feeder.*;
//import frc.robot.commands.intake.runOuttake;
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
  
  //Intake controller
  public static final XboxController intakeController = new XboxController(1);

  //Serializer controller
  public static final XboxController serializerController = new XboxController(2);

  //Climber controller
  public static final XboxController climberController = new XboxController(3);
  
  //Feeder controller
  public static final XboxController feederController = new XboxController(4);

  //Driver Controller 
  public static final Button RunSerializerBasic = new JoystickButton(driverController, RobotMap.LeftBumper);
  public static final Button RunFeederBasic = new JoystickButton(driverController, RobotMap.BButton);
  public static final Button RunFeederBasicBackwards = new JoystickButton(driverController, RobotMap.XButton);
  public static final Button RunShooterBasic = new JoystickButton(driverController, RobotMap.AButton);
  public static final Button RunShooterBasicBackwards = new JoystickButton(driverController, RobotMap.YButton);
  public static final Button RunIntakeBasic = new JoystickButton(driverController, RobotMap.RightBumper);
  
  //intake button 
  public static final Button setPistonInButton = new JoystickButton(intakeController, RobotMap.AButton);
  public static final Button setPistonOutButton = new JoystickButton(intakeController, RobotMap.BButton);
  public static final Button runIntakeBasicButton = new JoystickButton(intakeController, RobotMap.XButton);
  public static final Button intakeToggleButton = new JoystickButton(intakeController, RobotMap.YButton);
  public static final Button intakeSetValueButton = new JoystickButton(intakeController, RobotMap.LeftBumper);
  public static final Button intakeOnSmartDashboardButton = new JoystickButton(intakeController, RobotMap.RightBumper);

  //Serializer Buttons
  public static final Button RunSerializerAButton  = new JoystickButton(serializerController, RobotMap.AButton);
  public static final Button RunSerializerBButton = new JoystickButton(serializerController, RobotMap.BButton);
  public static final Button RunSerializerAdvancedButton = new JoystickButton(serializerController, RobotMap.XButton);
  public static final Button RunSerializerBasicButton = new JoystickButton(serializerController, RobotMap.YButton);
  public static final Button SerializerAutomatedButton = new JoystickButton(serializerController, RobotMap.LeftBumper);
  public static final Button SerializerOnSmartDashboardButton = new JoystickButton(serializerController, RobotMap.RightBumper);

  //Climber Buttons 
  public static final Button BrakeInButton = new JoystickButton(climberController, RobotMap.AButton);
  public static final Button BrakeOutButton = new JoystickButton(climberController, RobotMap.BButton);

  //Feeder Buttons
  public static final Button FeederAutomatedButton = new JoystickButton(feederController, RobotMap.AButton);
  public static final Button FeederOnJoystickButton = new JoystickButton(feederController, RobotMap.BButton);
  public static final Button FeederOnSmartDashboardButton = new JoystickButton(feederController, RobotMap.XButton);
  public static final Button FeederSetValueButton = new JoystickButton(feederController, RobotMap.YButton);
  public static final Button RunFeederButton = new JoystickButton(feederController, RobotMap.LeftBumper);
  public static final Button RunFeederJoystickButton = new JoystickButton(feederController, RobotMap.RightBumper);

  public OI() {
    
    //DriverController 
    RunFeederBasic.whileHeld(new FeederSetValue(0.8));
    RunFeederBasicBackwards.whileHeld(new FeederSetValue(-0.8));
    RunShooterBasic.whileHeld(new FeederSetValue(0.8));
    RunShooterBasicBackwards.whileHeld(new FeederSetValue(-0.8));
    RunIntakeBasic.whileHeld(new RunIntakeBasic(0.8));
    RunSerializerBasic.whileHeld(new RunSerializerBasic(0.8));
    
    //Intake Commands
    // followPath.whenPressed(new LeftPath());
    setPistonInButton.whenPressed(new SetPistonIn());
    setPistonOutButton.whenPressed(new SetPistonOut());
    runIntakeBasicButton.whileHeld(new RunIntakeBasic(0.5)); //speed motors are running at
    //when true, whenpressed, when false, whileheld
    intakeToggleButton.whenPressed(new IntakeToggle());
    intakeSetValueButton.whileHeld(new IntakeSetValue(0.5));
    intakeOnSmartDashboardButton.whileHeld(new IntakeOnSmartDashboard());

    //serializer
    RunSerializerAButton.whileHeld(new RunSerializerA(0.7));
    RunSerializerBButton.whileHeld(new RunSerializerB(-0.5));
    RunSerializerAdvancedButton.whileHeld(new RunSerializerAdvanced(0.7));
    RunSerializerBasicButton.whileHeld(new RunSerializerBasic(0.7));
    SerializerAutomatedButton.whileHeld(new SerializerAutomated(0.7));
    SerializerOnSmartDashboardButton.whileHeld(new SerializerOnSmartDashboard());

    //climber
    BrakeInButton.whenPressed(new BrakePistonIn());
    BrakeOutButton.whenPressed(new BrakePistonOut());

    //Feeder
    FeederAutomatedButton.whileHeld(new FeederAutomated());
    FeederOnJoystickButton.whileHeld(new FeederAutomated());
    FeederOnSmartDashboardButton.whileHeld(new FeederOnSmartDashboard());
    FeederSetValueButton.whileHeld(new FeederSetValue(0.5));
    RunFeederButton.whileHeld(new RunFeeder());
    RunFeederJoystickButton.whileHeld(new RunFeederJoystick());
    
    
  }
}
