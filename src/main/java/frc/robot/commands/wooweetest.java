// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.woowee;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.ShooterMan;



public class wooweetest extends Command {

  woowee s_woowee;
  //PIDController AutoPauseController;
  double sped;
  double position;


  /** Creates a new PositionIntake. */
  public wooweetest(woowee s_woowee, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    //AutoPauseController = new  PIDController(0.005, 0.0001, 0);


    this.s_woowee = s_woowee;
    this.position = position;
  
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //AutoPauseController.reset();

    s_woowee.resetPID();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sped = s_woowee.calc(s_woowee.getEnc(), position);


    s_woowee.woo(-sped);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_woowee.woo(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(s_woowee.getEnc() > (position - 5) && s_woowee.getEnc() < (position + 5)){
      return true;
    } else {
      return false;
    }
  }
}
