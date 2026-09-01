package frc.robot.Commands;

import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Subsystems.Gyro;
import frc.robot.Subsystems.Drivetrain.Drive;

public class DriveManual extends Command {
  XboxController joy;
  Drive driveSubsystem;
  SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.Drive.maxJoystickAccelXYMpS2);
  SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.Drive.maxJoystickAccelXYMpS2);
  SlewRateLimiter thetaLimiter = new SlewRateLimiter(Constants.Drive.maxJoystickAccelThetaRpS2);
  Gyro gyro;

  public DriveManual(Drive driveSubsystem, XboxController joy) {
    this.driveSubsystem = driveSubsystem;
    this.joy = joy;
    addRequirements(driveSubsystem);
  }

  public ChassisSpeeds getChassisSpeeds() {
    double joyX = -joy.getLeftY();
    double joyY = -joy.getLeftX();
    double joyZ = joy.getRightX();

    joyX = MathUtil.applyDeadband(joyX, .012);
    joyY = MathUtil.applyDeadband(joyY, .012);
    joyZ = MathUtil.applyDeadband(joyZ, .012);

    joyX = xLimiter.calculate(joyX * joyX * Math.signum(joyX));
    joyY = yLimiter.calculate(joyY * joyY * Math.signum(joyY));
    joyZ = thetaLimiter.calculate(joyZ * joyZ * Math.signum(joyZ));

    return new ChassisSpeeds(joyX * Constants.Drive.maxDriveSpeedMpS, joyY * Constants.Drive.maxDriveSpeedMpS,
        joyZ * Constants.Drive.maxTurnSpeedRpS);
  }

  @Override
  public void execute() {
    double multiplier = joy.getRightBumperButton() ? .25 : 1;
    if (joy.getLeftBumperButton()) {
      driveSubsystem.setRobotSpeeds(getChassisSpeeds().times(multiplier));
    } else {
      driveSubsystem.SetFieldSpeeds(getChassisSpeeds().times(multiplier));
    }
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return Set.of(driveSubsystem);
  }
}