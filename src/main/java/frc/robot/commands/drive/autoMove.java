// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Constants;
import frc.robot.subsystems.Lets_freakin_DRIVEEEE;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick; 


public class autoMove extends Command {
  /** Creates a new DrivinCommand. */
  Lets_freakin_DRIVEEEE s_Lets_freakin_DRIVEEEE;
  Joystick Driver;

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
  double setpointDistance;


  PIDController frickPidController;
  PIDController frickPidControllerx;
 // PIDController frickPidControllery;

  //how many degrees is the lime light off from vertical
  double limelightMountAngleDegrees;
   // distance from the center of the lime light lens to the floow
  double limelightLensHeightInches;
   //distance from the target to the floor
  double goalHeightInches;


  

  public autoMove(Lets_freakin_DRIVEEEE s_Lets_freakin_DRIVEEEE, double setpointDistance) {
    // Use addRequirements() here to declare subsystem dependencies.

    frickPidController = new  PIDController(0.03, 0.000000001, 0.0);
    frickPidControllerx = new  PIDController(0.03, 0.000000001, 0.0);
   
  
    this.s_Lets_freakin_DRIVEEEE = s_Lets_freakin_DRIVEEEE;
    this.setpointDistance = setpointDistance;

    limelightMountAngleDegrees = 24;
    limelightLensHeightInches = 10;
    goalHeightInches = 57.13;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    frickPidController.reset();
    frickPidControllerx.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tid = table.getEntry("tid");
    tlong = table.getEntry("tlong");
    tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");


    double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    targetOffsetAngle_Distance = tlong.getDouble(0.0); 
    swoopswoop = tx.getDouble(0.0);
    getLime = tid.getDouble(-1);

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

    double distanceFromLimelightToGoalInches = ((goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians));

    
    moveSpeed = frickPidController.calculate(distanceFromLimelightToGoalInches, setpointDistance);
    turnSpeed = frickPidControllerx.calculate(swoopswoop, 0);

   if(getLime == 7){
    s_Lets_freakin_DRIVEEEE.goOnAnGetBOY(-turnSpeed, -moveSpeed);
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
