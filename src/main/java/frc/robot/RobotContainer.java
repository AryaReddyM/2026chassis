// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.DriveManual;
import frc.robot.Commands.FollowPath;
import frc.robot.Subsystems.Gyro;
import frc.robot.Subsystems.Drivetrain.Drive;

public class RobotContainer {
  Timer timer = new Timer();
  Timer testTimer = new Timer();

  private XboxController joyDrive = new XboxController(0);
  private Drive drive = new Drive();
  private DriveManual teleopDriveCmd = new DriveManual(drive, joyDrive);

  public RobotContainer() {
    configureBindings();
    
    // Log telemetry to SmartDashboard and Shuffleboard
    drive.log();
    drive.setDefaultCommand(teleopDriveCmd);
    
    configureNamedCommands();
    // updateAutoSelector must be the last thing called !!!
    FollowPath.updateAutoSelector(drive);
  }

  private void configureBindings() {
    
    // new Trigger(() -> joyDrive.getRawButton(6)).debounce(.1).onTrue(speakerAimer);
    // joyDrive.getBButtonPressed()
    new Trigger(() -> joyDrive.getBButtonPressed()).onTrue(new InstantCommand(() -> drive.resetGyro()));

  }
  private void configureNamedCommands(){

   
  }

  public Command getAutonomousCommand() {
    return FollowPath.autoChooser.getSelected();
  }

}
