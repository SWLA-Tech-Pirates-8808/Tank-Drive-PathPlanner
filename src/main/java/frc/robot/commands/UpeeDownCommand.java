// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.woowee;

public class UpeeDownCommand extends Command {
  woowee s_woowee;
  Joystick bam;

  /** Creates a new UpeeDownCommand. */
  public UpeeDownCommand(woowee s_woowee,Joystick bam) {

    addRequirements(s_woowee);
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_woowee = s_woowee;
    this.bam = bam;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    s_woowee.woo(bam.getRawAxis(1)*0.2);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
