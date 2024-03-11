// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lets_freakin_DRIVEEEE;
import frc.robot.subsystems.woowee;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class trajectoryAime extends Command {

  Lets_freakin_DRIVEEEE s_Lets_freakin_DRIVEEEE;
  woowee s_woowee;

  double turnSpeed;
  double moveSpeed;
  double shootMove;

  NetworkTable table;
  double targetOffsetAngle_Distance;
  double swoopswoop;
  double getLime;
  NetworkTableEntry tlong;
  NetworkTableEntry tx;
  NetworkTableEntry tid;
  double sped;
  NetworkTableEntry botpose;

  PIDController frickPidController;
  PIDController frickPidControllerx;

  //how many degrees is the lime light off from vertical
  double limelightMountAngleDegrees;
   // distance from the center of the lime light lens to the floow
  double limelightLensHeightInches;
   //distance from the target to the floor
  double goalHeightInches;



  /** Creates a new trajectoryAime. */
  public trajectoryAime(Lets_freakin_DRIVEEEE s_Lets_freakin_DRIVEEEE, woowee s_woowee) {
    // Use addRequirements() here to declare subsystem dependencies.

    frickPidController = new  PIDController(0.025, 0.015, 0.0);
    frickPidControllerx = new  PIDController(0.025, 0.015, 0.0);

    this.s_Lets_freakin_DRIVEEEE = s_Lets_freakin_DRIVEEEE;
    this.s_woowee = s_woowee;

    limelightMountAngleDegrees = 24;
    limelightLensHeightInches = 10;
    goalHeightInches = 57.13;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    frickPidController.reset();
    frickPidControllerx.reset();
    s_woowee.resetPID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tid = table.getEntry("tid");
    tlong = table.getEntry("tlong");
    tx = table.getEntry("tx");
    botpose = table.getEntry("botpose");
    NetworkTableEntry ty = table.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = ((goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians));
    
    targetOffsetAngle_Distance = tlong.getDouble(0.0); 
    swoopswoop = tx.getDouble(0.0);
    getLime = tid.getDouble(-1);

    

    if(getLime == 7){

     if(distanceFromLimelightToGoalInches > 165){

    moveSpeed = frickPidController.calculate(distanceFromLimelightToGoalInches, 195);
    turnSpeed = frickPidControllerx.calculate(swoopswoop, 0);
    sped = s_woowee.calc(s_woowee.getEnc(), 400);

    s_Lets_freakin_DRIVEEEE.goOnAnGetBOY(-turnSpeed * .6, -moveSpeed * .75);
    s_woowee.woo(-sped);

    } else if(distanceFromLimelightToGoalInches > 107 /*&& distanceFromLimelightToGoalInches < 165*/){

    moveSpeed = frickPidController.calculate(distanceFromLimelightToGoalInches, 132);
    turnSpeed = frickPidControllerx.calculate(swoopswoop, 0);
    sped = s_woowee.calc(s_woowee.getEnc(), 325);

    s_Lets_freakin_DRIVEEEE.goOnAnGetBOY(-turnSpeed * .6, -moveSpeed * .75);
    s_woowee.woo(-sped);

    } else if(distanceFromLimelightToGoalInches >= 75 /*&& distanceFromLimelightToGoalInches < 165*/){

    moveSpeed = frickPidController.calculate(distanceFromLimelightToGoalInches, 82);
    turnSpeed = frickPidControllerx.calculate(swoopswoop, 0);
    sped = s_woowee.calc(s_woowee.getEnc(), 280);

    s_Lets_freakin_DRIVEEEE.goOnAnGetBOY(-turnSpeed * .6, -moveSpeed * .75);
    s_woowee.woo(-sped);

    } else if(distanceFromLimelightToGoalInches < 75){

    moveSpeed = frickPidController.calculate(distanceFromLimelightToGoalInches, 82);
    turnSpeed = frickPidControllerx.calculate(swoopswoop, 0);
    sped = s_woowee.calc(s_woowee.getEnc(), 280);

    s_Lets_freakin_DRIVEEEE.goOnAnGetBOY(-turnSpeed * .6, -moveSpeed * .75);
    s_woowee.woo(-sped);
    }
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
