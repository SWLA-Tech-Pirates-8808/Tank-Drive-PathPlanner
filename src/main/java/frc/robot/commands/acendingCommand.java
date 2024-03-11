// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb;

public class acendingCommand extends Command {

  climb s_climb;
  double speed;
  /** Creates a new acendingCommand. */
  public acendingCommand(climb s_climb, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.s_climb = s_climb;
    this.speed = speed;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_climb.acending(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      s_climb.acending(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
