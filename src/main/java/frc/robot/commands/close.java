// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.autoMove;
import frc.robot.subsystems.Lets_freakin_DRIVEEEE;
import frc.robot.subsystems.ShooterMan;
import frc.robot.subsystems.woowee;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class close extends SequentialCommandGroup {
  /** Creates a new close. */
  public close(Lets_freakin_DRIVEEEE s_Lets_freakin_DRIVEEEE, ShooterMan s_ShooterMan, woowee s_woowee) {

    addCommands(new autoMove(s_Lets_freakin_DRIVEEEE, 82).alongWith(new ShootCommand(s_ShooterMan, 1))
    .alongWith(new WooweePosition(s_woowee, 280)));
  }
}
