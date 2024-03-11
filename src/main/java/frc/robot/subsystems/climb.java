// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 

public class climb extends SubsystemBase {

  CANSparkMax climb;
  Encoder polevault;
  /** Creates a new climb. */
  public climb() {
    climb = new CANSparkMax(9, MotorType.kBrushless);
    polevault = new Encoder(3, 4,true, EncodingType.k1X);
    climb.setSmartCurrentLimit(12);
  }

  public void acending(double speed){
    climb.set(-speed);
  }

  public double getClimb(){
    return polevault.get();
  }

  public void resetClimbEnc(){
    polevault.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climb Enc", polevault.get());
  }
}
