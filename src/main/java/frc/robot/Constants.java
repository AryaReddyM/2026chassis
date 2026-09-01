package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static class Module{

        public static double convertDriveRot2M = (.0508) * 2 * Math.PI / 7.31;

        public static double wheelRadius = 0.028575;

        //Offset for each module in rotations
        public static double frAngleOffset = 0.5;
        public static double flAngleOffset = 0.5;
        public static double brAngleOffset = 0.5;
        public static double blAngleOffset = 0.5;

        // Formula is (-y,  x), where (x, y) is the untransformed coordinates
        // Ex: Front-right wheel original: (1, 3), Transformed: (-3, 1)
        public static Translation2d frRadius = new Translation2d(-0.3998, 0.3998);
        public static Translation2d flRadius = new Translation2d(-0.3998, -0.3998);
        public static Translation2d brRadius = new Translation2d(0.3998, 0.3998);
        public static Translation2d blRadius = new Translation2d(0.3998, -0.3998);

        public static int turnMotorCurrentLimit = 35;
        public static int driveMotorCurrentLimit = 35;

        public static double turnIntegrationCap = .5;
        public static double driveIntegrationCap = .001;

        public static double[] drivePIDGains = {0.006,0.000,0.0,.29};
        public static double[] turnPIDGains = {.005, 0, 0.0, 0};

    }
    public static class Drive{
        public static double maxDriveSpeedMpS = 4.5;        
        public static double maxTurnSpeedRpS = 2.5 * Math.PI;

        // public static double maxJoystickAccelXYMpS2 = maxDriveSpeedMpS * 1.5;        
        // public static double maxJoystickAccelThetaRpS2 = maxTurnSpeedRpS * 1.5;

        public static double maxJoystickAccelXYMpS2 = 4;//Double.MAX_VALUE;        
        public static double maxJoystickAccelThetaRpS2 = Math.PI * 3/2;

        public static int frDriveCANID = 5;
        public static int flDriveCANID = 4;
        public static int brDriveCANID = 8;
        public static int blDriveCANID = 6;

        public static int frTurnCANID = 7;
        public static int flTurnCANID = 3;
        public static int brTurnCANID = 41;
        public static int blTurnCANID = 2;

        public static boolean frDriveInvert = true;
        public static boolean frlDriveInvert = false;
        public static boolean brDriveInvert = false;
        public static boolean brlDriveInvert = false;
        
        public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            //(y, -x) I have no clue why this is not the same as the one in Module
            new Translation2d(0.3998, -0.3998),
            new Translation2d(0.3998, 0.3998),
            new Translation2d(-0.3998, -0.3998),
            new Translation2d(-0.3998, 0.3998)
        );

        public static  DCMotor driveGearbox = DCMotor.getNEO(1);
    }
    public static class Robot{
        public static double L = 24.075;
        public static double thetaEst = 45;
        public static double offsetDeg = 13.544;
        public static double alphaRad = Units.degreesToRadians(163.57);
    }
    
    public static class Auto{
        public static PIDConstants xyController = new PIDConstants(4.5,0,0.01);
        public static PIDConstants thetaController = new PIDConstants(3.9,0,0.03);
        public static double maxDriveSpeed = 5;
        public static double robotMassKg = 74.088;
        public static double robotMOI = 6.883;
        public static double wheelCOF = 1.2;
        public static RobotConfig ppConfig =
            new RobotConfig(
                robotMassKg,
                robotMOI,
                new ModuleConfig(
                    Module.wheelRadius,
                    maxDriveSpeed,
                    wheelCOF,
                    Drive.driveGearbox.withReduction(6),
                    Module.driveMotorCurrentLimit,
                    1),
                Module.frRadius, Module.flRadius, Module.brRadius, Module.blRadius);
    }


     public static class Location {
        public static AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout
                .loadField(AprilTagFields.k2026RebuiltAndymark);
        public static List<AprilTag> APRIL_TAGS = APRIL_TAG_FIELD_LAYOUT.getTags();
        public static int APRIL_TAGS_COUNT = 22;

        public static final Map<Integer, Pose2d> TAG_PROPERTIES = APRIL_TAGS.stream()
                .map(tag -> Map.entry(tag.ID, tag.pose.toPose2d()))
                .collect(Collectors.toMap(Map.Entry::getKey, Map.Entry::getValue));

        public static final int[] REEF_IDS = { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };
        public static final int[] CORAL_STATION_IDS = { 1, 2, 12, 13 };
        public static final int[] PROCESSOR_IDS = { 3, 16 };
        public static final int[] BARGE_IDS = { 4, 5, 14, 15 };

        public static final Transform2d CENTER_REEF_OFFSET = new Transform2d(Inches.of(18), Inches.of(0),
                new Rotation2d(Math.toRadians(-90)));
        public static final Transform2d FRONT_REEF_LEFT_OFFSET = new Transform2d(Inches.of(18), Inches.of(-16.35),
                new Rotation2d(Math.toRadians(180)));

        public static final Transform2d FRONT_REEF_RIGHT_OFFSET = new Transform2d(Inches.of(18), Inches.of(-4),
                new Rotation2d(Math.toRadians(180)));

        public static final Transform2d BACK_REEF_LEFT_OFFSET = new Transform2d(Inches.of(0), Inches.of(0),
                new Rotation2d(Math.PI));
        public static final Transform2d BACK_REEF_RIGHT_OFFSET = new Transform2d(Inches.of(0), Inches.of(0),
                new Rotation2d(Math.PI));

        public static final Transform2d CORAL_STATION_LEFT_OFFSET = new Transform2d(Inches.of(18), Inches.of(18),
                new Rotation2d(0));
        public static final Transform2d CORAL_STATION_RIGHT_OFFSET = new Transform2d(Inches.of(18), Inches.of(-18),
                new Rotation2d(0));
        public static final Transform2d CORAL_STATION_CENTER_OFFSET = new Transform2d(Inches.of(18), Inches.of(0),
                new Rotation2d(0));

        public static final Transform2d PROCESSOR_OFFSET = new Transform2d(Inches.of(18), Inches.of(0),
                new Rotation2d(Math.toRadians(270)));
    }
}