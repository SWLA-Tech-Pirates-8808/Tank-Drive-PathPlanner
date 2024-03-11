// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.woowee;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

public class WooweeAprilTag extends Command {

  double yBro;
  NetworkTableEntry getty;
  woowee s_woowee;
  //PIDController pidy;
  double speed;
  NetworkTable table;
  NetworkTableEntry tid;
  double getlime;

  /** Creates a new WooweeAprilTag. */
  public WooweeAprilTag(woowee s_woowee) {
    // Use addRequirements() here to declare subsystem dependencies.

    //pidy = new PIDController(0.009, 0.001, 0);


    this.s_woowee = s_woowee;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //pidy.reset();
    s_woowee.resetPID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    getty = table.getEntry("ty");
    tid = table.getEntry("tid");

    yBro = getty.getDouble(0.0);
    getlime = tid.getDouble(-1);

    speed = s_woowee.calc(s_woowee.getEnc(), getlime);

    if(getlime == 7){
    s_woowee.woo(-speed);
    }
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
