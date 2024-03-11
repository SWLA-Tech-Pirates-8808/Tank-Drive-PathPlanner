// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  TalonFX Intake;

  public Intake() {

    Intake = new TalonFX(10);


  }

  public void nom(double speed){

    Intake.set(speed);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
