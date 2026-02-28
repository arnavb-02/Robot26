package Team4450.Robot26.subsystems;

import static Team4450.Robot26.Constants.*;

import Team4450.Robot26.Constants;
import Team4450.Robot26.RobotContainer;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.PositionVoltage;

import Team4450.Robot26.utility.LinkedMotors;

public class Shooter extends SubsystemBase {
    // This motor is a Falcon 500
    private final TalonFX flywheelMotorTopLeft = new TalonFX(Constants.FLYWHEEL_MOTOR_TOP_LEFT_CAN_ID, new CANBus(Constants.CANIVORE_NAME));
    // This motor is a Falcon 500
    private final TalonFX flywheelMotorTopRight = new TalonFX(Constants.FLYWHEEL_MOTOR_TOP_RIGHT_CAN_ID, new CANBus(Constants.CANIVORE_NAME));
    // This motor is a Falcon 500
    private final TalonFX flywheelMotorBottomLeft = new TalonFX(Constants.FLYWHEEL_MOTOR_BOTTOM_LEFT_CAN_ID, new CANBus(Constants.CANIVORE_NAME));
    // This motor is Falcon 500
    private final TalonFX flywheelMotorBottomRight = new TalonFX(Constants.FLYWHEEL_MOTOR_BOTTOM_RIGHT_CAN_ID, new CANBus(Constants.CANIVORE_NAME));
    private final LinkedMotors flywheelMotors = new LinkedMotors(flywheelMotorTopLeft, flywheelMotorTopRight, flywheelMotorBottomLeft, flywheelMotorBottomRight);
    // This motor is a Kraken x60
    private final TalonFX hoodLeft = new TalonFX(Constants.HOOD_MOTOR_LEFT_CAN_ID, new CANBus(Constants.CANIVORE_NAME));
    // This motor is a Kraken x60
    private final TalonFX hoodRight = new TalonFX(Constants.HOOD_MOTOR_RIGHT_CAN_ID, new CANBus(Constants.CANIVORE_NAME));
    // This motor is a Kraken x44
    private final TalonFX infeedMotorLeft = new TalonFX(Constants.INFEED_MOTOR_LEFT_CAN_ID, new CANBus(Constants.CANIVORE_NAME));
    // This motor is a Kraken x44
    private final TalonFX infeedMotorRight = new TalonFX(Constants.INFEED_MOTOR_RIGHT_CAN_ID, new CANBus(Constants.CANIVORE_NAME));

    private boolean canFlywheel;
    private boolean canHood;
    private boolean canInfeed;

    private boolean runInfeed;

    // The format of this value is in rotations of the pivit motor
    private double hoodMotorPosition;
    private double hoodTargetMotorPosition;
    //Hood Rotation Offset
    private double hoodRotationOffset;

    DigitalInput beamBreak;

    // Constants for launch calculations
    private static final double GRAVITY = 9.81;
    private static final double DESIRED_MAX_HEIGHT = 2.5; // meters (8.2 feet)
    private static final double GOAL_HEIGHT = 1.8288; // meters (6 feet)
    private static final double FLYWHEEL_HEIGHT = 0.5334; // meters (21 inches)
    private static final double CONVERSION_FACTOR_MPS_TO_RPM = 10000 / 47.93;

    private double targetRPM = Constants.FLYWHEEL_TARGET_RPM;
    private double currentRPM = 0.0;

    private final double maxRpm = Constants.FLYWHEEL_MAX_THEORETICAL_RPM;

    private boolean flywheelEnabled = false; // Button-controlled enable

    // Shuffleboard cached values
    private boolean sdInit = false;

    private boolean disableAutomaticFlywheelUpdate = false;

    private double sd_kP, sd_kI, sd_kD;
    private double sd_kS, sd_kV, sd_kA;

    private Drivebase drivebase;

    public Shooter(Drivebase drivebase) {
        this.drivebase = drivebase;

        this.canFlywheel = flywheelMotorTopLeft.isConnected() && flywheelMotorTopRight.isConnected() && flywheelMotorBottomLeft.isConnected() && flywheelMotorBottomRight.isConnected();
        this.canHood = hoodLeft.isConnected() && hoodRight.isConnected();
        this.canInfeed = infeedMotorLeft.isConnected() && infeedMotorRight.isConnected();

        this.hoodMotorPosition = 0;

        this.hoodRotationOffset = this.hoodLeft.getPosition(true).getValueAsDouble();

        beamBreak = new DigitalInput(Constants.SHOOTER_UPPER_BEAM_BREAK_PORT);

        applyFlywheelConfig(
            Constants.FLYWHEEL_kP, Constants.FLYWHEEL_kI, Constants.FLYWHEEL_kD,
            Constants.FLYWHEEL_kS, Constants.FLYWHEEL_kV, Constants.FLYWHEEL_kA);

        applyInfeedConfig();

        applyHoodConfig();

        this.hoodMotorPosition = 0;
        this.hoodLeft.setPosition(this.hoodMotorPosition);

        SmartDashboard.putNumber(Constants.ShooterKeys.HOOD_POSITION, 0);

        SmartDashboard.putNumber(Constants.ShooterKeys.FLYWHEEL_TARGET_RPM, Constants.FLYWHEEL_TARGET_RPM);

        SmartDashboard.putNumber(Constants.ShooterKeys.FLYWHEEL_KP, Constants.FLYWHEEL_kP);
        SmartDashboard.putNumber(Constants.ShooterKeys.FLYWHEEL_KI, Constants.FLYWHEEL_kI);
        SmartDashboard.putNumber(Constants.ShooterKeys.FLYWHEEL_KD, Constants.FLYWHEEL_kD);

        SmartDashboard.putNumber(Constants.ShooterKeys.FLYWHEEL_KS, Constants.FLYWHEEL_kS);
        SmartDashboard.putNumber(Constants.ShooterKeys.FLYWHEEL_KV, Constants.FLYWHEEL_kV);
        SmartDashboard.putNumber(Constants.ShooterKeys.FLYWHEEL_KA, Constants.FLYWHEEL_kA);
        SmartDashboard.putNumber(Constants.ShooterKeys.HOOD_POSITION, hoodMotorPosition);

        sd_kP = Constants.FLYWHEEL_kP;
        sd_kI = Constants.FLYWHEEL_kI;
        sd_kD = Constants.FLYWHEEL_kD;

        sd_kS = Constants.FLYWHEEL_kS;
        sd_kV = Constants.FLYWHEEL_kV;
        sd_kA = Constants.FLYWHEEL_kA;

        sdInit = true;

        SmartDashboard.putNumber(Constants.ShooterKeys.HOOD_POWER, 0.05);
        SmartDashboard.putNumber(Constants.ShooterKeys.INFEED_TARGET_RPM, Constants.INFEED_DEFAULT_TARGET_RPM);
        SmartDashboard.putBoolean(Constants.ShooterKeys.DISABLE_AUTO_FLYWHEEL_UPDATE, this.disableAutomaticFlywheelUpdate);
    }

    @Override
    public void periodic() {
        // Flywheel is controlled by TestSubsystem via Constants; no dashboard reads here.
        
        // This line should be all that is needed when the flywheel should be spun up
        updateLaunchValues(true);

        // Update the beam break sensors
        SmartDashboard.putBoolean(Constants.ShooterKeys.BEAM_BREAK, beamBreak.get());

        hoodMotorPosition = hoodLeft.getPosition().getValueAsDouble();

        if (!SmartDashboard.getBoolean(Constants.ShooterKeys.DISABLE_AUTO_FLYWHEEL_UPDATE, this.disableAutomaticFlywheelUpdate)) {
            updateHoodPosition(SmartDashboard.getNumber(Constants.ShooterKeys.HOOD_POSITION, 0));
        } else {
            updateHoodPosition(SmartDashboard.getNumber(Constants.ShooterKeys.HOOD_POSITION, this.hoodMotorPosition));
        }

        SmartDashboard.putNumber(Constants.ShooterKeys.HOOD_ANGLE, getHoodMotorAngleRadians());
        SmartDashboard.putNumber(Constants.ShooterKeys.HOOD_MOTOR_POSITION, getHoodMotorPosition());

        double measuredRps =
                flywheelMotorTopLeft.getRotorVelocity()
                        .refresh()
                        .getValueAsDouble();

        currentRPM = measuredRps * 60.0;
        SmartDashboard.putNumber(Constants.ShooterKeys.FLYWHEEL_MEASURED_RPM_LEGACY, currentRPM);

        // -------- Shuffleboard tuning --------

        // Only let the dashboard override targetRPM when automatic update is disabled
        if (disableAutomaticFlywheelUpdate) {
            targetRPM = SmartDashboard.getNumber(Constants.ShooterKeys.FLYWHEEL_TARGET_RPM, targetRPM);
        }

        double kP = SmartDashboard.getNumber(Constants.ShooterKeys.FLYWHEEL_KP, sd_kP);
        double kI = SmartDashboard.getNumber(Constants.ShooterKeys.FLYWHEEL_KI, sd_kI);
        double kD = SmartDashboard.getNumber(Constants.ShooterKeys.FLYWHEEL_KD, sd_kD);

        double kS = SmartDashboard.getNumber(Constants.ShooterKeys.FLYWHEEL_KS, sd_kS);
        double kV = SmartDashboard.getNumber(Constants.ShooterKeys.FLYWHEEL_KV, sd_kV);
        double kA = SmartDashboard.getNumber(Constants.ShooterKeys.FLYWHEEL_KA, sd_kA);

        // Apply only if changed
        if (!sdInit ||
                kP != sd_kP || kI != sd_kI || kD != sd_kD ||
                kS != sd_kS || kV != sd_kV || kA != sd_kA) {

            applyFlywheelConfig(kP, kI, kD, kS, kV, kA);

            sd_kP = kP;
            sd_kI = kI;
            sd_kD = kD;

            sd_kS = kS;
            sd_kV = kV;
            sd_kA = kA;

            sdInit = true;
        }

        double targetRPS;

        if (flywheelEnabled && canFlywheel) {
            targetRPS = targetRPM / 60.0;
            MotionMagicVelocityVoltage req =
                    new MotionMagicVelocityVoltage(targetRPS)
                            .withSlot(Constants.FLYWHEEL_PID_SLOT);

            this.flywheelMotorTopLeft.setControl(req);
            this.flywheelMotorTopRight.setControl(new Follower(flywheelMotorTopLeft.getDeviceID(), MotorAlignmentValue.Opposed));
            this.flywheelMotorBottomLeft.setControl(new Follower(flywheelMotorTopLeft.getDeviceID(), MotorAlignmentValue.Aligned));
            this.flywheelMotorBottomRight.setControl(new Follower(flywheelMotorTopLeft.getDeviceID(), MotorAlignmentValue.Opposed));
        } else {
            targetRPS = 0;
            CoastOut req =
                    new CoastOut();

            this.flywheelMotorTopLeft.setControl(req);
            this.flywheelMotorTopRight.setControl(new Follower(flywheelMotorTopLeft.getDeviceID(), MotorAlignmentValue.Opposed));
            this.flywheelMotorBottomLeft.setControl(new Follower(flywheelMotorTopLeft.getDeviceID(), MotorAlignmentValue.Aligned));
            this.flywheelMotorBottomRight.setControl(new Follower(flywheelMotorTopLeft.getDeviceID(), MotorAlignmentValue.Opposed));
        }

        double percent = currentRPM / maxRpm;

        SmartDashboard.putNumber(Constants.ShooterKeys.FLYWHEEL_MEASURED_RPM, currentRPM);

        SmartDashboard.putNumber(Constants.ShooterKeys.FLYWHEEL_PERCENT_OUT, percent);

        SmartDashboard.putNumber(Constants.ShooterKeys.INFEED_RPM, getInfeedRPM());

        if (this.runInfeed) {
            setInfeedRPM(SmartDashboard.getNumber(Constants.ShooterKeys.INFEED_TARGET_RPM, Constants.INFEED_DEFAULT_TARGET_RPM));
        }

        SmartDashboard.putNumber(Constants.ShooterKeys.FLYWHEEL_CURRENT_DRAW, getFlywheelCurrent());
        SmartDashboard.putNumber(Constants.ShooterKeys.INFEED_CURRENT_DRAW, getInfeedCurrent());
   }

    public void updateLaunchValues(boolean interpolate){
        // Calculate distance to goal & diffs
        double xDiff = Math.abs(getGoalPose().getX() - drivebase.getPose().getX());

        double yDiff = Math.abs(getGoalPose().getY() - drivebase.getPose().getY());
        double distToGoal = Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));

        SmartDashboard.putNumber(Constants.ShooterKeys.ROBOT_LAUNCH_X, drivebase.getPose().getX());
        SmartDashboard.putNumber(Constants.ShooterKeys.ROBOT_LAUNCH_Y, drivebase.getPose().getY());
        SmartDashboard.putString(Constants.ShooterKeys.GOAL_POSE, getGoalPose().toString());
        SmartDashboard.putString(Constants.ShooterKeys.DIFFS, String.format("%.2f, %.2f", xDiff, yDiff));
        SmartDashboard.putNumber(Constants.ShooterKeys.ROBOT_DISTANCE, distToGoal);

        if (interpolate) {
            if (!SmartDashboard.getBoolean(Constants.ShooterKeys.DISABLE_AUTO_FLYWHEEL_UPDATE, this.disableAutomaticFlywheelUpdate)) {
                //Changed to ensure that the targetRPM stays on robot for
                //fastest updates and to avoid dashboard latency especially at competition
                this.targetRPM = interpolateFlywheelSpeedByDistance(distToGoal);
                SmartDashboard.putNumber(Constants.ShooterKeys.FLYWHEEL_TARGET_RPM, this.targetRPM);
            }
            setHoodMotorPosition(interpolateHoodByDistance(distToGoal));
        }   
    }

    public Pose2d getGoalPose() {
        // If blue side
        if (alliance == Alliance.Blue) {
            return HUB_BLUE_WELDED_POSE;
        // If red side
        } else if (alliance == Alliance.Red) {
            return HUB_RED_WELDED_POSE;
        } else {
            return null; // Error
        }
    }

    public double getAngleToFaceGoalDegrees(Pose2d robotPosition) {
        // Find the difference betweeen the robot and the goal
        double xDiff = getGoalPose().getX() - RobotContainer.drivebase.getPose().getX();
        double yDiff = getGoalPose().getY() - RobotContainer.drivebase.getPose().getY();

        // use atan2 to get teh correct angle to face goal & convert to degrees
        double angleToFaceGoal = Math.toDegrees(Math.atan(yDiff / xDiff));

        return (angleToFaceGoal - robotPosition.getRotation().getDegrees());
    }

    // This method sets the lower and higher points to interpolate between 
    // Based on the hardcoded flywheel speed & distance tables and the robots current distance away from the goal
    public double interpolateFlywheelSpeedByDistance(double distToGoal) {
        
        double lowerPoint = FLYWHEEL_SPEED_DISTANCE_TABLE[0];
        int lowerPointIndex = 0;

        double higherPoint = FLYWHEEL_SPEED_DISTANCE_TABLE[FLYWHEEL_SPEED_DISTANCE_TABLE.length - 1];
        int higherPointIndex = FLYWHEEL_SPEED_DISTANCE_TABLE.length - 1;

        double currentDistance;

        for (int i = FLYWHEEL_SPEED_DISTANCE_TABLE.length - 2; i > 0; i--){
            currentDistance = FLYWHEEL_SPEED_DISTANCE_TABLE[i];
            if(currentDistance > distToGoal){
                if (currentDistance <= higherPoint) {
                    higherPoint = currentDistance;
                    higherPointIndex = i;
                }
            }else if (currentDistance < distToGoal){
                if (currentDistance >= lowerPoint) {
                    lowerPoint = currentDistance;
                    lowerPointIndex = i;
                }
            }else if (currentDistance == distToGoal){
                return FLYWHEEL_SPEED_TABLE[i];
            }
        }
        double lowerSpeed = FLYWHEEL_SPEED_TABLE[lowerPointIndex];
        double higherSpeed = FLYWHEEL_SPEED_TABLE[higherPointIndex];

        return linearInterpolate(lowerSpeed, higherSpeed, (distToGoal - lowerPoint) / (higherPoint - lowerPoint));
    }

    public double interpolateHoodByDistance(double distToGoal) {
        
        double lowerPoint = FLYWHEEL_SPEED_DISTANCE_TABLE[0];
        int lowerPointIndex = 0;

        double higherPoint = FLYWHEEL_SPEED_DISTANCE_TABLE[FLYWHEEL_SPEED_DISTANCE_TABLE.length - 1];
        int higherPointIndex = FLYWHEEL_SPEED_DISTANCE_TABLE.length - 1;

        double currentDistance;

        for (int i = FLYWHEEL_SPEED_DISTANCE_TABLE.length - 2; i > 0; i--){
            currentDistance = HOOD_ARC_TABLE[i];
            if(currentDistance > distToGoal){
                if (currentDistance <= higherPoint) {
                    higherPoint = currentDistance;
                    higherPointIndex = i;
                }
            }else if (currentDistance < distToGoal){
                if (currentDistance >= lowerPoint) {
                    lowerPoint = currentDistance;
                    lowerPointIndex = i;
                }
            }else if (currentDistance == distToGoal){
                return HOOD_ARC_TABLE[i];
            }
        }
        double lowerSpeed = HOOD_ARC_TABLE[lowerPointIndex];
        double higherSpeed = HOOD_ARC_TABLE[higherPointIndex];

        return linearInterpolate(lowerSpeed, higherSpeed, (distToGoal - lowerPoint) / (higherPoint - lowerPoint));
    }

    public static double linearInterpolate(double point1, double point2, double percentageSplit) {
        return point1 + ((point2 - point1) * percentageSplit);
    }

    
    // Linear interpolate the hood angle between zero and one with the motor rotations of up and down on the hood
    public double hoodAngleToMotorPosition(double hoodAngle) {
        return ((hoodAngle - Constants.HOOD_DOWN_ANGLE_DEGREES) / Constants.HOOD_GEAR_RATIO);
    }

    public double motorPositionToHoodAngle(double motorPosition) {
        return ((motorPosition * Constants.HOOD_GEAR_RATIO * 360) + Constants.HOOD_DOWN_ANGLE_DEGREES);
    }

    public void updateHoodPosition(double pos) {
        // MotionMagicVoltage req = new MotionMagicVoltage(pos);
        PositionVoltage req = new PositionVoltage(pos);

        this.hoodLeft.setControl(req);
        this.hoodRight.setControl(new Follower(this.hoodLeft.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void startFlywheel() {
        this.flywheelEnabled = true;
    }

    public void stopFlywheel() {
        this.flywheelEnabled = false;
    }

    public double getFlywheelRPM() {
        return currentRPM;
    }

    public double getFlywheelTargetRPM() {
        return targetRPM;
    }

    public double getFlywheelError() {
        return targetRPM - currentRPM;
    }

    public double getFlywheelCurrent() {
        return flywheelMotorBottomLeft.getSupplyCurrent(true).getValueAsDouble() + flywheelMotorBottomRight.getSupplyCurrent(true).getValueAsDouble() + flywheelMotorTopLeft.getSupplyCurrent(true).getValueAsDouble() + flywheelMotorTopRight.getSupplyCurrent(true).getValueAsDouble();
    }

    public double getFlywheelTopLeftMotorCurrent() {
        return flywheelMotorTopLeft.getSupplyCurrent(true).getValueAsDouble();
    }

    public double getFlywheelTopRightMotorCurrent() {
        return flywheelMotorTopRight.getSupplyCurrent(true).getValueAsDouble();
    }

    public double getFlywheelBottomLeftMotorCurrent() {
        return flywheelMotorBottomLeft.getSupplyCurrent(true).getValueAsDouble();
    }

    public double getFlywheelBottomRightMotorCurrent() {
        return flywheelMotorBottomRight.getSupplyCurrent(true).getValueAsDouble();
    }

    public void startInfeed() {
        if (canInfeed) {
            this.runInfeed = true;
        }
    }

    public void testInfeed() {
        if (canInfeed) {
            this.infeedMotorLeft.set(0.05);
            this.infeedMotorRight.setControl(new Follower(this.infeedMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed));
        }
    }

    public void stopInfeed() {
        if (canInfeed) {
            this.runInfeed = false;
            this.infeedMotorLeft.set(0);
            this.infeedMotorRight.setControl(new Follower(this.infeedMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed));
        }
    }

    public double getInfeedRPM() {
        return infeedMotorLeft.getRotorVelocity(true).getValueAsDouble() * 60;
    }

    public double getInfeedCurrent() {
        return infeedMotorLeft.getSupplyCurrent(true).getValueAsDouble() + infeedMotorRight.getSupplyCurrent(true).getValueAsDouble();
    }

    public double getTransferLeftMotorCurrent() {
        return infeedMotorLeft.getSupplyCurrent(true).getValueAsDouble();
    }

    public double getTransferRightMotorCurrent() {
        return infeedMotorRight.getSupplyCurrent(true).getValueAsDouble();
    }

    // The position input is between 0 and 1 with 0 being up and 1 being down
    public void setHoodMotorPosition(double position) {
        hoodTargetMotorPosition = position;
    }

    public double getHoodMotorAngleRadians() {
        return (hoodLeft.getPosition(true).getValueAsDouble() - this.hoodRotationOffset);
    }

    public double getHoodMotorPosition() {
        return hoodLeft.getPosition().getValueAsDouble();
    }

    public double getHoodCurrent() {
        return hoodLeft.getSupplyCurrent(true).getValueAsDouble() + hoodRight.getSupplyCurrent(true).getValueAsDouble();
    }

    public double getHoodLeftMotorCurrent() {
        return hoodLeft.getSupplyCurrent(true).getValueAsDouble();
    }

    public double getHoodRightMotorCurrent() {
        return hoodRight.getSupplyCurrent(true).getValueAsDouble();
    }

    /**
     * Calculates the RPM for the flywheel based on the robot's pose and the hub's pose.
     * @param robotPose The current pose of the robot (x, y, rotation).
     * @param hubPose The pose of the hub (x, y).
     * @param hoodAngle The constant hood angle in radians.
     * @return The calculated RPM value.
     */
    public double calculateRPM(Pose2d robotPose, Pose2d hubPose, double hoodAngle) {
        double deltaX = hubPose.getX() - robotPose.getX();
        double deltaY = hubPose.getY() - robotPose.getY();
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        // Calculate the required velocity to reach the hub
        double velocity = Math.sqrt((GRAVITY * distance * distance) / 
            (2 * (distance * Math.tan(hoodAngle) - (GOAL_HEIGHT - FLYWHEEL_HEIGHT))));

        // Convert velocity to RPM
        double rpmMath = velocity * CONVERSION_FACTOR_MPS_TO_RPM;
        return rpmMath;
    }

    /**
     * Interpolates RPM values based on predefined data points.
     * @param distance The distance between the robot and the hub.
     * @return The interpolated RPM value.
     */
    public double interpolateRPM(double distance) {
        // Example interpolation table (distance in meters, RPM values)
        double[][] rpmTable = {
            {1.0, 2000},
            {2.0, 2500},
            {3.0, 3000},
            {4.0, 3500},
            {5.0, 4000}
        };

        // Linear interpolation logic
        for (int i = 0; i < rpmTable.length - 1; i++) {
            if (distance >= rpmTable[i][0] && distance <= rpmTable[i + 1][0]) {
                double x1 = rpmTable[i][0];
                double y1 = rpmTable[i][1];
                double x2 = rpmTable[i + 1][0];
                double y2 = rpmTable[i + 1][1];

                // Linear interpolation formula
                return y1 + (distance - x1) * (y2 - y1) / (x2 - x1);
            }
        }

        // Return the closest value if out of bounds
        if (distance < rpmTable[0][0]) return rpmTable[0][1];
        if (distance > rpmTable[rpmTable.length - 1][0]) return rpmTable[rpmTable.length - 1][1];

        return 0; // Default case (should not occur)
    }

    /**
     * Calculates the final RPM by averaging RPMmath and interpolated RPM.
     * @param robotPose The current pose of the robot (x, y, rotation).
     * @param hubPose The pose of the hub (x, y).
     * @param hoodAngle The constant hood angle in radians.
     * @return The final RPM value.
     */
    public double calculateFinalRPM(Pose2d robotPose, Pose2d hubPose, double hoodAngle) {
        double deltaX = hubPose.getX() - robotPose.getX();
        double deltaY = hubPose.getY() - robotPose.getY();
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        SmartDashboard.putNumber(Constants.ShooterKeys.ROBOT_DISTANCE, distance);

        double rpmMath = calculateRPM(robotPose, hubPose, hoodAngle);
        double interpolatedRPM = interpolateRPM(distance);

        return (rpmMath + interpolatedRPM) / 2.0;
    }

    public void setInfeedRPM(double targetRPM) {
        double currentRPM = getInfeedRPM();
        double error = targetRPM - currentRPM;
        double adjustment = Constants.INFEED_kP * error; // Adjustment to approach target
        double newRPM = targetRPM + adjustment; // Adjust current RPM towards target
        //this.infeedMotorLeft.set(newRPM / Constants.FLYWHEEL_MAX_THEORETICAL_RPM);
        this.infeedMotorLeft.set(0.8);
        this.infeedMotorRight.setControl(new Follower(this.infeedMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    // -------------------------------------------------------------------------
    // Motor config helpers
    // -------------------------------------------------------------------------

    /**
     * Builds and applies a TalonFX configuration to all four flywheel motors.
     * Called from the constructor and from periodic() whenever PID/FF values change.
     */
    private void applyFlywheelConfig(double kP, double kI, double kD,
                                     double kS, double kV, double kA) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        cfg.CurrentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Constants.SHOOTER_FLYWHEEL_CURRENT_LIMIT);

        cfg.Slot0.kP = kP;
        cfg.Slot0.kI = kI;
        cfg.Slot0.kD = kD;
        cfg.Slot0.kS = kS;
        cfg.Slot0.kV = kV;
        cfg.Slot0.kA = kA;

        cfg.MotionMagic.MotionMagicAcceleration = Constants.FLYWHEEL_MOTION_ACCEL_RPMS / 60.0;
        cfg.MotionMagic.MotionMagicJerk = Constants.FLYWHEEL_MOTION_JERK;

        for (int i = 0; i < flywheelMotors.getTotalMotors(); i++) {
            TalonFX motor = flywheelMotors.getMotorByIndex(i);
            if (motor != null) motor.getConfigurator().apply(cfg);
        }
    }

    /**
     * Builds and applies a TalonFX configuration to both hood motors.
     * Values come entirely from Constants so there is one source of truth.
     */
    private void applyHoodConfig() {
        TalonFXConfiguration hoodCFG = new TalonFXConfiguration();

        hoodCFG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        hoodCFG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        hoodCFG.CurrentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Constants.SHOOTER_HOOD_CURRENT_LIMIT);

        hoodCFG.Slot0.kP = Constants.HOOD_kP;
        hoodCFG.Slot0.kI = Constants.HOOD_kI;
        hoodCFG.Slot0.kD = Constants.HOOD_kD;
        hoodCFG.Slot0.kS = Constants.HOOD_kS;
        hoodCFG.Slot0.kV = Constants.HOOD_kV;
        hoodCFG.Slot0.kA = Constants.HOOD_kA;

        hoodCFG.MotionMagic.MotionMagicAcceleration = Constants.HOOD_MOTION_ACCEL;
        hoodCFG.MotionMagic.MotionMagicJerk = Constants.HOOD_MOTION_JERK;

        this.hoodLeft.getConfigurator().apply(hoodCFG);
        this.hoodRight.getConfigurator().apply(hoodCFG);
    }

    /**
     * Builds and applies a TalonFX configuration to both infeed motors.
     * Inversion is kept local here to avoid vendor-specific enums in Constants.
     */
    private void applyInfeedConfig() {
        TalonFXConfiguration infeedCFG = new TalonFXConfiguration();

        infeedCFG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        infeedCFG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        infeedCFG.CurrentLimits = new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Constants.SHOOTER_INFEED_CURRENT_LIMIT);

        this.infeedMotorLeft.getConfigurator().apply(infeedCFG);
        this.infeedMotorRight.getConfigurator().apply(infeedCFG);
    }
}
