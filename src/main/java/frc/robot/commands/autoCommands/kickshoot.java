// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.ShooterMan;
import frc.robot.subsystems.lazerbeam;
import frc.robot.subsystems.woowee;

public class kickshoot extends Command {
  /** Creates a new kickshoot. */
  Kicker s_Kicker;
  ShooterMan s_ShooterMan;
  lazerbeam s_Lazerbeam;
  woowee s_woowee;
  double shootspeed;
  double kickspeed;
  double sped;
  double position;
  //PIDController AutoPauseController2;

  
  public kickshoot(Kicker s_Kicker, ShooterMan s_ShooterMan, woowee s_woowee, double shootspeed, double kickspeed, lazerbeam s_Lazerbeam, double position) {
    // Use addRequirements() here to declare subsystem dependencies.

   // AutoPauseController2 = new  PIDController(0.005, 0.0001, 0);


    this.s_Kicker = s_Kicker;
    this.s_woowee = s_woowee;
    this.s_ShooterMan = s_ShooterMan;
    this.shootspeed = shootspeed;
    this.kickspeed = kickspeed;
    this.s_Lazerbeam = s_Lazerbeam;
    this.position = position;
    

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  //  AutoPauseController2.reset();
  s_woowee.resetPID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sped = s_woowee.calc(s_woowee.getEnc(), position);

    s_ShooterMan.shootThing(shootspeed);
    s_Kicker.twoAndFour(-kickspeed);
    s_woowee.woo(-sped);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_ShooterMan.shootThing(0);
    s_Kicker.twoAndFour(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (s_Lazerbeam.EpicLaz()) {
      return true;
    } else {
      return false;
    }
  }
}
