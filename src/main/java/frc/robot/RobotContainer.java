// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/* You will need to import all commands and subsystems here. Use the below method to import all with only 2 lines of code */
import frc.robot.commands.*;
import frc.robot.commands.autoCommands.noteAuto;
import frc.robot.commands.autoCommands.sub;
import frc.robot.subsystems.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import com.pathplanner.lib.commands.PathPlannerAuto;
//import com.pathplanner.lib.path.PathPlannerPath;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  private final Lets_freakin_DRIVEEEE s_Lets_freakin_DRIVEEEE = new Lets_freakin_DRIVEEEE();
  private final ShooterMan s_ShooterMan = new ShooterMan();
  private final Kicker s_Kicker = new Kicker();
  private final woowee s_woowee = new woowee();
  private final Intake s_Intake = new Intake();
  private final lightTime s_LightTime = new lightTime();
  private final climb s_climb = new climb();
  private final lazerbeam s_lazerbeam = new lazerbeam();
  public final static Joystick Drew = new Joystick(0);
  public final static Joystick bazinga = new Joystick(1);


  private final JoystickButton BIGEAT = new JoystickButton(Drew, XboxController.Button.kRightBumper.value);
  private final JoystickButton spit = new JoystickButton(Drew, XboxController.Button.kLeftBumper.value);
  private final JoystickButton yButtonD = new JoystickButton(Drew, XboxController.Button.kY.value);
  private final JoystickButton aButtonD = new JoystickButton(Drew, XboxController.Button.kA.value);
  private final JoystickButton ResetEncoders = new JoystickButton(Drew, XboxController.Button.kB.value);
  private final JoystickButton mid = new JoystickButton(bazinga, XboxController.Button.kB.value);
  private final JoystickButton close = new JoystickButton(bazinga, XboxController.Button.kA.value);
  private final JoystickButton centerstage = new JoystickButton(bazinga, XboxController.Button.kY.value);
  private final JoystickButton buttonX = new JoystickButton(bazinga, XboxController.Button.kX.value);
  private final JoystickButton spinAim = new JoystickButton(bazinga, XboxController.Button.kLeftBumper.value);
  private final JoystickButton KICK = new JoystickButton(bazinga, XboxController.Button.kRightBumper.value);
  
  
  private final SendableChooser<Command> autoChooser;  
  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    s_Lets_freakin_DRIVEEEE.setDefaultCommand(new DrivinCommand(s_Lets_freakin_DRIVEEEE, Drew));
    s_woowee.setDefaultCommand(new UpeeDownCommand(s_woowee, bazinga));



    NamedCommands.registerCommand("aimBOY", new aimBOY(s_woowee, s_ShooterMan));
    NamedCommands.registerCommand("Intake", new Intakicker(s_Intake, s_Kicker, s_lazerbeam, s_woowee));
    NamedCommands.registerCommand("mid", new mid(s_Lets_freakin_DRIVEEEE, s_ShooterMan, s_woowee));
    NamedCommands.registerCommand("shoot", new ShootCommand(s_ShooterMan, 1));
    NamedCommands.registerCommand("close", new close(s_Lets_freakin_DRIVEEEE, s_ShooterMan, s_woowee));
    NamedCommands.registerCommand("kicker", new KickerCommand(s_Kicker, 1));
    NamedCommands.registerCommand("sub", new sub(s_Kicker, s_woowee, s_ShooterMan, s_lazerbeam));
    NamedCommands.registerCommand("note", new noteAuto(s_Kicker, s_woowee, s_ShooterMan, s_lazerbeam));

    configureBindings();

         autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@litnk edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release. 
       BIGEAT.whileTrue(new Radintake(s_Intake, s_Kicker, s_woowee, s_lazerbeam));
       spit.whileTrue(new unEat(s_Intake, s_Kicker, s_woowee, s_lazerbeam)); // also need to add set positions with woowee but just for
       close.whileTrue(new close(s_Lets_freakin_DRIVEEEE, s_ShooterMan, s_woowee));
       mid.whileTrue(new mid(s_Lets_freakin_DRIVEEEE, s_ShooterMan, s_woowee));
       centerstage.whileTrue(new centerstage(s_Lets_freakin_DRIVEEEE, s_ShooterMan, s_woowee));
       buttonX.whileTrue(new ampShoot(s_woowee, s_ShooterMan));
       spinAim.whileTrue(new ShootCommand(s_ShooterMan, 1)); // this will aim and spinup shooter
       KICK.whileTrue(new KickerCommand(s_Kicker, 0.5));
       ResetEncoders.whileTrue(new RESETSTUFF(s_Lets_freakin_DRIVEEEE, s_climb, s_woowee));
       yButtonD.whileTrue(new acendingCommand(s_climb, 1));
       aButtonD.whileTrue(new acendingCommand(s_climb, -1));
      // BIGEAT.whileFalse(new WooweePosition(s_woowee, 90));
      // close.whileFalse(new WooweePosition(s_woowee, 90));
      // mid.whileFalse(new WooweePosition(s_woowee, 90));
      // centerstage.whileFalse(new WooweePosition(s_woowee, 90));
      // buttonX.whileFalse(new WooweePosition(s_woowee, 90));


       
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   // An example command will be run in autonomous
   return autoChooser.getSelected();
  }
}
