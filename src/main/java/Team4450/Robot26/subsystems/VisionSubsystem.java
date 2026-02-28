package Team4450.Robot26.subsystems;

import Team4450.Lib.Util;
import Team4450.Robot26.Constants;
import Team4450.Robot26.utility.RobotOrientation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.Math;

public class VisionSubsystem extends SubsystemBase {
    // Info from: https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltags
    //
    // TODO: We need to aim the Limelight upwards at an angle when the april tag is as centered as possible at 10-20 feet or so (Whatever the average sight distance will be)
    //
    // The tag collection family in "AprilTag Classic 36h11
    // The Limelight offset from robot pose must be set in the web view for each Limelight
    //
    // The Coordinate plane for Limelight is as follows
    // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems
    //
    // Thermal throttling is a big concern

    // Speed tips
    //
    // Increase detector downscale for insteased framerate without affecting 3D accuracy
    // Lower brightness and contrast values will improve pipeline framerate at the cost of range
    // If the Limelight has a large area of the frame that will never have a tag, then the area that the april tag algo will be reduced and result in huge performance
    //
    // MegaTag 2
    //
    // The heading of the robot is required to use this, Idealy pas a complete robot orientation and angular velocities
    // LimelightHelpers.SetRobotOrientation(robotYawInDegrees,0,0,0,0,0))
    // SetRobotOrientation assumes a centered (see the map generator) or blue-corner origin. CCW-positive, 0 degrees -> facing red alliance wall in FRC.
    //
    Drivebase drivebase;
    boolean enabled = true;

    public VisionSubsystem(Drivebase drivebase) {

        // Need to add a null check for the cameras
        this.drivebase = drivebase;
        RobotOrientation rO = drivebase.getRobotOrientation(); // IDK if RobotOrientation works correctly, look there to see
                                                               

        // Quick notes on the rotation 3d getX() return the roll in radians, getY() return the pitch in radians, getZ() return the yaw in radians

        // Set the Limelight offsets from the center of the robot
        LimelightHelpers.setCameraPose_RobotSpace(Constants.LIMELIGHT_FRONT, Constants.ROBOT_TO_LIMELIGHT_FRONT.getX(), Constants.ROBOT_TO_LIMELIGHT_FRONT.getY(),
                Constants.ROBOT_TO_LIMELIGHT_FRONT.getZ(), Math.toDegrees(Constants.ROBOT_TO_LIMELIGHT_FRONT.getRotation().getX()),
                Math.toRadians(Constants.ROBOT_TO_LIMELIGHT_FRONT.getRotation().getY()), Math.toDegrees(Constants.ROBOT_TO_LIMELIGHT_FRONT.getRotation().getZ()));

        LimelightHelpers.setCameraPose_RobotSpace(Constants.LIMELIGHT_RIGHT, Constants.ROBOT_TO_LIMELIGHT_RIGHT.getX(), Constants.ROBOT_TO_LIMELIGHT_RIGHT.getY(),
                Constants.ROBOT_TO_LIMELIGHT_RIGHT.getZ(), Math.toDegrees(Constants.ROBOT_TO_LIMELIGHT_RIGHT.getRotation().getX()),
                Math.toRadians(Constants.ROBOT_TO_LIMELIGHT_RIGHT.getRotation().getY()), Math.toDegrees(Constants.ROBOT_TO_LIMELIGHT_RIGHT.getRotation().getZ()));

        zeroLimelightIMU(rO);

    }

    public void enableInternalIMU() {
        RobotOrientation rO = drivebase.getRobotOrientation(); // IDK if RobotOrientation works correctly, look there to see
        Util.consoleLog("Init Limelight Internal IMU Front");
        LimelightHelpers.SetRobotOrientation(Constants.LIMELIGHT_FRONT, rO.yaw, rO.yawRate, rO.pitch, rO.pitchRate, rO.roll, rO.rollRate);
        Util.consoleLog("Init Limelight Internal IMU Right");
        LimelightHelpers.SetRobotOrientation(Constants.LIMELIGHT_RIGHT, rO.yaw, rO.yawRate, rO.pitch, rO.pitchRate, rO.roll, rO.rollRate);
        // IMU mode 2 uses to Limelight 4 internal IMU
        LimelightHelpers.SetIMUMode(Constants.LIMELIGHT_FRONT, 2);
        LimelightHelpers.SetIMUMode(Constants.LIMELIGHT_RIGHT, 2);

    }

    @Override
    public void periodic() {
        if (!enabled) {
            return;
        }
        // How to stpo this when disabled
        boolean useLeftLimelight = true;
        boolean useRightLimelight = true;
        // Get latest pose estimage from each camera
        
        // LimelightHelpers.PoseEstimate front_mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LIMELIGHT_FRONT);
        LimelightHelpers.PoseEstimate front_mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.LIMELIGHT_FRONT);
        // Pose2d front_mt2 = LimelightHelpers.getBotPose2d_wpiBlue(Constants.LIMELIGHT_LEFT);
        //
        // LimelightHelpers.PoseEstimate right_mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LIMELIGHT_RIGHT);
        LimelightHelpers.PoseEstimate right_mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.LIMELIGHT_RIGHT);
        // Pose2d right_mt2 = LimelightHelpers.getBotPose2d_wpiBlue(Constants.LIMELIGHT_RIGHT);

        // If the angular velocity is greater than 720 degrees per second ignore the vision update
        //
        // IDK if this is yaw rate or what units this is in
        if (Math.abs(drivebase.pigeonWrapper.pigeon.getAngularVelocityXDevice().getValueAsDouble()) > 720) {
            return;
        }

        // Get rid of any result that says we are out of the field

        // IDK what units the getX() return
        // IDK what units the getY() return
        

        if (front_mt2 != null) {
            if (Math.abs(front_mt2.pose.getX()) > Constants.FIELD_MAX_X) {
                useLeftLimelight = false;
            }

            if (Math.abs(front_mt2.pose.getY()) > Constants.FIELD_MAX_Y) {
                useLeftLimelight = false;
            }

            double numTags = front_mt2.rawFiducials.length;
            for (LimelightHelpers.RawFiducial tag : front_mt2.rawFiducials) {
                if (Math.abs(tag.txnc) > 20) { // TODO: Convert 20 to a constants value
                    numTags--;
                } else if (Math.abs(tag.tync) > 20) {
                    numTags--;
                }
            }

            if (numTags < 2) {
                useLeftLimelight = false;
            }

            if (useLeftLimelight) {
                SmartDashboard.putBoolean("Send Front Limelight info", true);
                drivebase.addLimelightMeasurement(front_mt2.pose, front_mt2.timestampSeconds);
            } else {
                SmartDashboard.putBoolean("Send Front Limelight info", false);
            }
        }

        if (right_mt2 != null) {
            if (Math.abs(right_mt2.pose.getX()) > Constants.FIELD_MAX_X) {
                useRightLimelight = false;
            }

            if (Math.abs(right_mt2.pose.getY()) > Constants.FIELD_MAX_Y) {
                useRightLimelight = false;
            }

            double numTags = right_mt2.rawFiducials.length;
            for (LimelightHelpers.RawFiducial tag : right_mt2.rawFiducials) {
                if (Math.abs(tag.txnc) > 20) { // TODO: Convert 20 to a constants value
                    numTags--;
                } else if (Math.abs(tag.tync) > 20) {
                    numTags--;
                }
            }

            if (numTags < 2) {
                useRightLimelight = false;
            }

            if (right_mt2.rawFiducials.length < 2) {
                useRightLimelight = false;
            }

            if (useRightLimelight) {
                SmartDashboard.putBoolean("Send Right Limelight info", true);
                drivebase.addLimelightMeasurement(right_mt2.pose, right_mt2.timestampSeconds);
            } else {
                SmartDashboard.putBoolean("Send Right Limelight info", false);
            }
        }

        // Get rid of the result if the yaw of the resulting pose is impossible
        //
        // I think the yaw is between -180 and 180 instead of 0 - 360
        // if (front_mt2.pose.getRotation().getDegrees() < 0 || front_mt2.pose.getRotation().getDegrees() > 360 || right_mt2.pose.getRotation().getDegrees() < 0 || right_mt2.pose.getRotation().getDegrees() > 360) {
        //     return;
        // }
        
    }

    public void zeroLimelightIMU(RobotOrientation rO) { // Set to IMU mode 0 to diable the internal limelight IMU
        // Setting to IMU mode 1 will use the setRobotOrientation to set the internal Limelight IMU
        LimelightHelpers.SetIMUMode(Constants.LIMELIGHT_FRONT, 1);
        LimelightHelpers.SetIMUMode(Constants.LIMELIGHT_RIGHT, 1);
        
        LimelightHelpers.SetRobotOrientation(Constants.LIMELIGHT_FRONT, rO.yaw, rO.yawRate, rO.pitch, rO.pitchRate, rO.roll, rO.rollRate);
        LimelightHelpers.SetRobotOrientation(Constants.LIMELIGHT_RIGHT, rO.yaw, rO.yawRate, rO.pitch, rO.pitchRate, rO.roll, rO.rollRate);
    }
}
