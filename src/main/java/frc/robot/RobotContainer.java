// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.Optional;
import java.util.OptionalInt;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.AimAndLaunch;
import frc.robot.Commands.RunIntakeSmart;
import frc.robot.Commands.StaticLaunch;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ScoringArmConstants;
import frc.robot.subsystems.FieldDriverStick;
import frc.robot.subsystems.ScoringArm;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;



/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  
  public final ScoringArm m_ScoringArm = new ScoringArm();
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),Constants.ROBOT_SUPERSONIC_CONFIG_LOCATION));

  private static VisionSubsystem m_robotVision = new VisionSubsystem();

  SendableChooser<Command> m_autoChooser = AutoBuilder.buildAutoChooser();



  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  FieldDriverStick m_driveStick = new FieldDriverStick(m_driverController);

  // the copilot controller
  Joystick m_copilotController = new Joystick(OIConstants.kCopilotControllerPort);


  //Buttons on the drivers controller
  JoystickButton m_calibrateButton = new JoystickButton(m_driverController, 8);


  //Buttons on the copilots controller
  // JoystickButton m_incLauncherRPM = new JoystickButton(m_copilotController, 4);
  // JoystickButton m_decLauncherRPM = new JoystickButton(m_copilotController, 5);

  // JoystickButton m_intakeButton = new JoystickButton(m_copilotController, 8);
  // JoystickButton m_outtakeButton = new JoystickButton(m_copilotController, 9);


  // JoystickButton m_visionLaunchButton = new JoystickButton(m_copilotController, 7);

  // JoystickButton m_climbPrepArmPosButton = new JoystickButton(m_copilotController, 8);
  // JoystickButton m_climbFinishArmPosButton = new JoystickButton(m_copilotController, 9);
  // JoystickButton m_setLaunchArmPosButton = new JoystickButton(m_copilotController, 10);
  // JoystickButton m_ampArmPosButton = new JoystickButton(m_copilotController, 11);
  


  //Face forward
  Pose2d defaultFaceForwardPose = new Pose2d(2,7,Rotation2d.fromDegrees(0));

  //Face Right, move diagonal
  Pose2d defaultZeroPosition = new Pose2d(0.33 ,0.33,Rotation2d.fromDegrees(0));
  

  //PathPlannerTrajectory path = PathPlannerPath.loadPath("Froggy Demo Path", 1, 1);
  public String myAlliance = DriverStation.getAlliance().toString();

  public boolean driveBrakeMode;
  
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {



    if (Constants.VisionConstants.hasCamera)
    {
      RobotContainer.setRobotVision(new VisionSubsystem(Constants.VisionConstants.cameraY, Constants.VisionConstants.cameraX, Constants.VisionConstants.cameraZ, Constants.VisionConstants.cameraName));
    } 

    // Configure the button bindings
    configureButtonBindings();


    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-m_driveStick.getX(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driveStick.getY(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(m_driveStick.getZ(), OperatorConstants.RIGHT_X_DEADBAND));

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(-m_driveStick.getX(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-m_driveStick.getY(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(m_driveStick.getZ(), OperatorConstants.RIGHT_X_DEADBAND));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
   

        
    drivebase.zeroGyro();


    //Set default to robot on field position
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
    SmartDashboard.putNumber("ArmAngleSlider", 10);
  }

  public void robotInit(){
    Shuffleboard.getTab("Game HUD").addDouble("Robot Pitch", (()-> drivebase.getPitch().getDegrees())).withWidget(BuiltInWidgets.kDial);
    Shuffleboard.getTab("Game HUD").addDouble("Arm Angle", m_ScoringArm::GetArmAngle);
    //Shuffleboard.getTab("Game HUD").add(autoChooser).withSize(2,1);
    configureAutoNamedCommands();

  }

  public void configureAutoNamedCommands(){
    
    NamedCommands.registerCommand("Smart Intake", new RunIntakeSmart(m_ScoringArm));
    NamedCommands.registerCommand("Near Static Launch", new StaticLaunch(m_ScoringArm, 150));

  }

  public static DriverStation.Alliance allianceColor = DriverStation.Alliance.Blue;

  public static boolean isRedAlliance() 
  {
    return allianceColor.equals(DriverStation.Alliance.Red);
  }

  public static boolean isBlueAlliance() {
    return !isRedAlliance();
  }

  public static void setAlliance(Optional<DriverStation.Alliance> color) {
    if (color.isPresent()) {
      DriverStation.Alliance currentColor = allianceColor;      
      allianceColor = color.get();
      if (!allianceColor.equals(currentColor))
      {
        System.out.println("Changed alliance to " +(isBlueAlliance()?"Blue":"Red"));
      }
    }
  }

  public static VisionSubsystem getRobotVision() {
    return m_robotVision;
  }
  public static void setRobotVision(VisionSubsystem robotVision) {
    m_robotVision = robotVision;
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    m_calibrateButton.onTrue((new InstantCommand(drivebase::zeroGyro)));

    //m_incLauncherRPM.onTrue(new InstantCommand(() -> m_ScoringArm.ChangeLaunchSpeed(5)));
    //m_decLauncherRPM.onTrue(new InstantCommand(() -> m_ScoringArm.ChangeLaunchSpeed(-5)));

    //m_visionLaunchButton.whileTrue(new AimAndLaunch(m_ScoringArm));

    new JoystickButton(m_copilotController, 8).whileTrue(new StartEndCommand(() ->m_ScoringArm.Intake() , () -> m_ScoringArm.StopIntake()));
    new JoystickButton(m_copilotController, 9).whileTrue(new StartEndCommand(() ->m_ScoringArm.Outtake() , () -> m_ScoringArm.StopIntake()));

    
    new JoystickButton(m_copilotController, 5).onTrue(new InstantCommand(() -> m_ScoringArm.SetArmAngle(ScoringArmConstants.kArmPosClimbPrep)));
    new JoystickButton(m_copilotController, 6).onTrue(new InstantCommand(() -> m_ScoringArm.Climb()));
    new JoystickButton(m_copilotController, 3).onTrue(new InstantCommand(() -> m_ScoringArm.SetArmAngle(ScoringArmConstants.kArmPosNearStaticLaunch)));
    new JoystickButton(m_copilotController, 2).onTrue(new InstantCommand(() -> m_ScoringArm.SetArmAngle(ScoringArmConstants.kArmPosFarStaticLaunch)));
    new JoystickButton(m_copilotController, 1).onTrue(new InstantCommand(() -> m_ScoringArm.SetArmAngle(ScoringArmConstants.kArmPosAmp)));
    new JoystickButton(m_copilotController, 4).onTrue(new InstantCommand(() -> m_ScoringArm.SetArmAngle(ScoringArmConstants.kArmPosPickup)));


    //new JoystickButton(m_driverController, 14).onTrue(new InstantCommand(drivebase::visionPose));

    new JoystickButton(m_copilotController, 7).whileTrue(new StartEndCommand(() -> m_ScoringArm.Launch(), ()-> m_ScoringArm.StopIntake()));
    new JoystickButton(m_copilotController, 10).whileTrue(new StartEndCommand(() -> m_ScoringArm.SetLaunchSpeed(200), () -> m_ScoringArm.SetLaunchSpeed(0)));
    new JoystickButton(m_copilotController, 11).onTrue(new InstantCommand(()-> m_ScoringArm.SetArmAngleToSDBValue()));
    //new JoystickButton(m_copilotController, 7).onTrue(new StaticLaunch(m_ScoringArm));

    // new JoystickButton(m_driverController, 3).onTrue(new InstantCommand( ()-> m_ScoringArm.LatchClimb() ) );
    // new JoystickButton(m_driverController, 4).onTrue(new InstantCommand( ()-> m_ScoringArm.UnlatchClimb() ) );
    //new JoystickButton(m_driverController, 3).onTrue(new InstantCommand( ()-> m_ScoringArm.SetFlap(true) ) );
    //new JoystickButton(m_driverController, 4).onTrue(new InstantCommand( ()-> m_ScoringArm.SetFlap(false) ) );
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    

    // An example trajectory to follow.  All units in meters.
    // Trajectory exampleTrajectory =
    //     TrajectoryGenerator.generateTrajectory(
    //         // Start at the origin facing the +X direction
    //         new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    //         // Pass through these two interior waypoints, making an 's' curve path
    //         List.of(new Translation2d(0.5, 0.5)),
    //         // End 3 meters straight ahead of where we started, facing forward
    //         new Pose2d(1, 0, Rotation2d.fromDegrees(30)),
    //         AutoConstants.kMaxSpeedTrajectoryConfig);



    //FollowTrajectory trajectoryDrive = new FollowTrajectory(exampleTrajectory, m_robotDrive, true, true);
    
    // Run path following command, then stop at the end.
    
    //return m_autoChooser.getSelected();
    
     //m_robotDrive.setDisplayTrajectory(exampleTrajectory);
   return m_autoChooser.getSelected();
  }

/**
   * Set the initial pose of the robot based on driverstation selection
   */
   public void setInitialPose()
  {
    //Read position
    OptionalInt driverStationLocation = DriverStation.getLocation();
    int location = (driverStationLocation.isPresent())?driverStationLocation.getAsInt():1;
    //set zero based
    location--;

    //Set default to robot on field position
    if (isRedAlliance()) 
    {
      drivebase.resetOdometry(Constants.PoseConstants.initRobotPoses[3+location]);
    } else {
      drivebase.resetOdometry(Constants.PoseConstants.initRobotPoses[location]);
    }    
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  
  public void setMotorBrake(boolean brake)
  {

    if (brake != driveBrakeMode) {
      drivebase.setMotorBrake(brake);
      driveBrakeMode = brake;
    }
    
  }


  public void zeroGyroWithAlliance()
  {
      if (isRedAlliance())
      {
        drivebase.zeroGyro();
        //Set the pose 180 degrees
        drivebase.resetOdometry(new Pose2d(drivebase.getPose().getTranslation(), Rotation2d.fromDegrees(180)));
      } else {
         drivebase.zeroGyro();      
      }
  }
}
