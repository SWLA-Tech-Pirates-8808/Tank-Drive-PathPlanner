// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class woowee extends SubsystemBase {
  /** Creates a new woowee. */
  CANSparkMax wooweer;
  Encoder shootEnc;
  DigitalInput EpiclazerBeam;
  PIDController InPauseController;

  public woowee() {

     InPauseController = new  PIDController(0.005, 0.0001, 0);

    wooweer = new CANSparkMax(8,MotorType.kBrushless);

    shootEnc = new Encoder(1, 2, true, EncodingType.k1X);

    EpiclazerBeam = new DigitalInput(0);
    wooweer.setSmartCurrentLimit(4);

  }

  public void woo(double speed) {

    wooweer.set(speed);

  }
  public double getEnc(){
   return shootEnc.getRaw();
  }

  public void resetShootEnc(){
    shootEnc.reset();
  }

 

  public double calc(double Encoder, double position){
   return InPauseController.calculate(Encoder, position);
  }

  public void resetPID(){
    InPauseController.reset();
  }


 /*  public boolean EpicLaz(){
    return EpiclazerBeam.get();
   } */


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Position", shootEnc.getRaw());
  }
}
