// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterMan;



public class ShootSeperateWheels extends Command {
  /** Creates a new ShootCommand. */
ShooterMan s_ShooterMan;
double shootTop;
double shootBottom;

  public ShootSeperateWheels(ShooterMan s_ShooterMan, double shootTop, double shootBottom) {
    // Use addRequirements() here to declare subsystem dependencies.
  this.s_ShooterMan = s_ShooterMan;
  this.shootTop = shootTop;
  this.shootBottom = shootBottom;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    s_ShooterMan.ShootTop(shootTop);
    
    s_ShooterMan.ShootBottom(shootBottom);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    s_ShooterMan.ShootTop(0);
    s_ShooterMan.ShootBottom(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
