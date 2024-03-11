// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import edu.wpi.first.math.util.Units;
import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.path.PathPlannerTrajectory;
//import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;


public class Lets_freakin_DRIVEEEE extends SubsystemBase {
  /** Creates a new Lets_freakin_DRIVEEEE. */

  DifferentialDrive UnderBoy;
  AHRS gyro;
  RamseteController controller1;
  DifferentialDriveOdometry odometry;
  DifferentialDriveKinematics kinematics;
  double speeds;
  TalonFX frontLeftLeader;
  TalonFX frontRightLeader;
  TalonFX backLeftFollower;
  TalonFX backRightFollower;
  StatusSignal<Double> enc;
  Joystick Driver;
  JoystickButton ky;
  JoystickButton kx;
  JoystickButton a;
  JoystickButton b;
  JoystickButton left;
  JoystickButton right;
  public double leftEnc;
  double rightEnc;
  double error;
  PIDController leftController;
  PIDController rightController;


  
//gor giga goof pluh 87 !@#$%^&*() pluh orpleeeeeeeeeeeeeeeee oap ur mom
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry id = table.getEntry("tid");


  public Lets_freakin_DRIVEEEE() {

    frontLeftLeader = new TalonFX(4);
    backLeftFollower = new TalonFX(3);
    frontRightLeader = new TalonFX(1);
    backRightFollower = new TalonFX(2);

    backLeftFollower.setControl(new Follower(frontLeftLeader.getDeviceID(), false));
    backRightFollower.setControl(new Follower(frontRightLeader.getDeviceID(), false));

    //var closedLP = new TalonFXConfiguration();
    //frontLeftLeader.getConfigurator().refr
    //frontLeftLeader.getConfigurator().apply(new TalonFXConfiguration());
    
    enc = frontLeftLeader.getPosition();

    Driver = new Joystick(0);

    gyro = new AHRS(I2C.Port.kOnboard);
    controller1 = new RamseteController();

    leftEnc = frontLeftLeader.getPosition().getValue(); //* Constants.OperatorConstants.kLinearDistanceConversionFactor;
    rightEnc = frontRightLeader.getPosition().getValue(); // * Constants.OperatorConstants.kLinearDistanceConversionFactor;

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), 
                    ((frontLeftLeader.getPosition().getValue())),
                    ((-frontRightLeader.getPosition().getValue())));
    kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(26.5));

    UnderBoy = new DifferentialDrive(frontLeftLeader, frontRightLeader);

    AutoBuilder.configureRamsete(
                
                this::getPose,
                this::resetPose, // Robot pose supplier
                this::getChassisSpeeds, // Current ChassisSpeeds supplier
                this::drive, // Method that will drive the robot given ChassisSpeeds
                new ReplanningConfig(true,true), 
                  // Default path replanning config. See the API for the options here
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    
  } 

  // func for drive train
   public void goOnAnGetBOY(double turnspeed, double moveSpeed){
    UnderBoy.arcadeDrive(turnspeed, moveSpeed);
   }

   public void resetPose(Pose2d pose){
    resetEnc();
    odometry.resetPosition(gyro.getRotation2d(),
    ((frontLeftLeader.getPosition().getValue())),
    ((-frontRightLeader.getPosition().getValue())),
    pose);
   }

   public double avgEncDistance(){
    return (((-frontLeftLeader.getPosition().getValue())
    + frontRightLeader.getPosition().getValue()
    + (-backLeftFollower.getPosition().getValue())
    + backRightFollower.getPosition().getValue())/ 4);
   }


   public void resetEnc(){
    frontLeftLeader.setPosition(0);
    frontRightLeader.setPosition(0);
    backLeftFollower.setPosition(0);
    backRightFollower.setPosition(0);
   }
   //start pathplanner code

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      (frontLeftLeader.getVelocity().getValue()),
      (frontRightLeader.getVelocity().getValue()));
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getWheelSpeeds());
  }

   public void drive(ChassisSpeeds ChassisSpeeds){

    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(ChassisSpeeds);
    wheelSpeeds.desaturate(.5);

    UnderBoy.tankDrive( wheelSpeeds.leftMetersPerSecond, -wheelSpeeds.rightMetersPerSecond);
   }

     // end stuff for path planner

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    odometry.update(gyro.getRotation2d(), 
                    ((((frontLeftLeader.getPosition().getValue())) / Constants.DriveConstants.GearRatio * Constants.DriveConstants.EncoderTPR * Constants.DriveConstants.WheelCircumferenceMeters) / 1000),
                    ((((-frontRightLeader.getPosition().getValue())) / Constants.DriveConstants.GearRatio * Constants.DriveConstants.EncoderTPR * Constants.DriveConstants.WheelCircumferenceMeters) / 1000));
;
    
    a = new JoystickButton(Driver, XboxController.Button.kA.value);
    ky = new JoystickButton(Driver, XboxController.Button.kY.value);
    kx = new JoystickButton(Driver, XboxController.Button.kX.value);
    b = new JoystickButton(Driver, XboxController.Button.kB.value);
    right = new JoystickButton(Driver, XboxController.Button.kRightBumper.value);
    left = new JoystickButton(Driver, XboxController.Button.kLeftBumper.value);

    NetworkTableEntry ty = table.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);

    //how many degrees is the lime light off from vertical
    double limelightMountAngleDegrees = 24;

    // distance from the center of the lime light lens to the floow
    double limelightLensHeightInches = 10;

    //distance from the target to the floor
    double goalHeightInches = 57.13;

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = ((goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians));

    SmartDashboard.putNumber("Distance In", distanceFromLimelightToGoalInches);
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Gyro Pitch", gyro.getPitch());
    SmartDashboard.putNumber("Angle", gyro.getAngle());
    SmartDashboard.putNumber("Heading", gyro.getCompassHeading());
    SmartDashboard.putNumber("Front Left Enc Position", frontLeftLeader.getPosition().getValue());
    SmartDashboard.putNumber("Front Right Enc Position", frontRightLeader.getPosition().getValue());
    SmartDashboard.putNumber("Back Left Enc Position", backLeftFollower.getPosition().getValue());
    SmartDashboard.putNumber("Back Right Enc Position", backRightFollower.getPosition().getValue());
    SmartDashboard.putNumber("Position Meter Left", ((frontLeftLeader.getPosition().getValue()) / Constants.DriveConstants.GearRatio * Constants.DriveConstants.EncoderTPR * Constants.DriveConstants.WheelCircumferenceMeters) / 1000);
    SmartDashboard.putNumber("Position Right Meters", ((-frontRightLeader.getPosition().getValue()) / Constants.DriveConstants.GearRatio * Constants.DriveConstants.EncoderTPR * Constants.DriveConstants.WheelCircumferenceMeters) / 1000);
    SmartDashboard.putNumber("Left Enc", leftEnc);
    SmartDashboard.putNumber("Right Enc", rightEnc);;
    SmartDashboard.putBoolean("y butt", ky.getAsBoolean());
    SmartDashboard.putBoolean("x butt", kx.getAsBoolean());
    SmartDashboard.putBoolean("a butt", a.getAsBoolean());
    SmartDashboard.putBoolean("b butt", b.getAsBoolean());
    SmartDashboard.putBoolean("right bum", right.getAsBoolean());
    SmartDashboard.putBoolean("left bum", left.getAsBoolean());
  }
}