// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.woowee;
import frc.robot.subsystems.lazerbeam;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class unEat extends SequentialCommandGroup {
  /** Creates a new unEat. */
  public unEat(Intake s_Intake, Kicker s_Kicker, woowee s_woowee, lazerbeam s_lazerbeam) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new wooweetest(s_woowee, 318).andThen(
      new OuttakeCommand(s_Intake, 0.35).alongWith(new KickerCommand(s_Kicker, -.2)).
      alongWith(new WooweePosition(s_woowee, 318)).withTimeout(2))
      .andThen(new WooweePosition(s_woowee, 30))

    );
  }
}
