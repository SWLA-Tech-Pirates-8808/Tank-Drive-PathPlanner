// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.lazerbeam;
import frc.robot.subsystems.woowee;


public class Intakicker extends Command {

  Intake s_Intake;
  Kicker s_Kicker;
  lazerbeam s_lazerbeam;
  woowee s_woowee;
  //PIDController InPauseController;
  double sped;
  /** Creates a new Intakicker. */
  public Intakicker(Intake s_Intake, Kicker s_Kicker, lazerbeam s_lazerbeam, woowee s_woowee) {
    // Use addRequirements() here to declare subsystem dependencies.

    //InPauseController = new  PIDController(0.003, 0.0001, 0);

    this.s_Intake = s_Intake;
    this.s_Kicker = s_Kicker;
    this.s_lazerbeam = s_lazerbeam;
    this.s_woowee = s_woowee;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //InPauseController.reset();
    s_woowee.resetPID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    sped = s_woowee.calc(s_woowee.getEnc(), 318);

    if(s_lazerbeam.EpicLaz() == true){
      s_Intake.nom(.3);
      s_Kicker.twoAndFour(-.3);
      s_woowee.woo(-sped);
      

    } else if(s_lazerbeam.EpicLaz() == false){
      end(true);

    
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  s_Intake.nom(0);
  s_Kicker.twoAndFour(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return !s_lazerbeam.EpicLaz();
    }
  }

