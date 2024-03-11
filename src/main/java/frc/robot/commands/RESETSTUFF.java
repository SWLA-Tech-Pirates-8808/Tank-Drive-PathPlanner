// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lets_freakin_DRIVEEEE;
import frc.robot.subsystems.climb;
import frc.robot.subsystems.woowee;

public class RESETSTUFF extends Command {

  Lets_freakin_DRIVEEEE s_Lets_freakin_DRIVEEEE;
  climb s_Climb;
  woowee s_woowee;
  /** Creates a new RESETSTUFF. */
  public RESETSTUFF(Lets_freakin_DRIVEEEE s_Lets_freakin_DRIVEEEE, climb s_Climb, woowee s_woowee) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.s_Lets_freakin_DRIVEEEE = s_Lets_freakin_DRIVEEEE;
    this.s_Climb = s_Climb;
    this.s_woowee = s_woowee;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Lets_freakin_DRIVEEEE.resetEnc();
    s_Climb.resetClimbEnc();
    s_woowee.resetShootEnc();


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
