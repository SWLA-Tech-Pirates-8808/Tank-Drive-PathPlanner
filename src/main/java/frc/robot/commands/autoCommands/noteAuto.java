// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.woowee;
import frc.robot.subsystems.lazerbeam;
import frc.robot.subsystems.ShooterMan;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class noteAuto extends SequentialCommandGroup {
  /** Creates a new sub. */
  public noteAuto(Kicker s_Kicker, woowee s_Woowee, ShooterMan s_ShooterMan, lazerbeam s_Lazerbeam) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new wooweeAuto(s_Woowee, s_ShooterMan, 217).
      andThen(new kickshoot(s_Kicker, s_ShooterMan, s_Woowee, 1, 0.35, s_Lazerbeam, 217))
    );
  }
}
