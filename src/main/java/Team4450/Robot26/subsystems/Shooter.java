package Team4450.Robot26.subsystems;

import static Team4450.Robot26.Constants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import Team4450.Robot26.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

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
    private final TalonFX hoodRollerLeft = new TalonFX(Constants.HOOD_MOTOR_LEFT_CAN_ID, new CANBus(Constants.CANIVORE_NAME));
    // This motor is a Kraken x60
    private final TalonFX hoodRollerRight = new TalonFX(Constants.HOOD_MOTOR_RIGHT_CAN_ID, new CANBus(Constants.CANIVORE_NAME));
    // This motor is a Kraken x44
    private final TalonFX rollerLeft = new TalonFX(Constants.ROLLER_MOTOR_LEFT_CAN_ID, new CANBus(Constants.CANIVORE_NAME));
    // This motor is a Kraken x44
    private final TalonFX rollerRight = new TalonFX(Constants.ROLLER_MOTOR_RIGHT_CAN_ID, new CANBus(Constants.CANIVORE_NAME));

    // Link the two roller motors for use when setting the power
    private final LinkedMotors rollerMotors = new LinkedMotors(rollerLeft, rollerRight);

    private boolean canFlywheel;
    private boolean canHood;
    private boolean canInfeed;

    // This value is expected to be between 0 and 1
    private double hoodTargetAngle;
    // The format of this value is in rotations of the pivit motor
    private double hoodTargetAngleMotorPosition;

    // This value is expected to be between 0 and 1
    private double hoodCurrentAngle;
    // The format of this value is in rotations of the pivit motor
    private double hoodCurrentAngleMotorPosition;
    // Current RPM of the flywheel
    private double flywheelCurrentRPM;
    // Target RPM of the flywheel
    private double flywheelTargetRPM;
    // Current Error of the flywheel
    private double flywheelError;

    private Drivebase drivebase;

    DigitalInput beamBreak;

    // Constants for launch calculations
    private static final double GRAVITY = 9.81;
    private static final double DESIRED_MAX_HEIGHT = 2.5; // meters (8.2 feet)
    private static final double GOAL_HEIGHT = 1.8288; // meters (6 feet)
    private static final double FLYWHEEL_HEIGHT = 0.5334; // meters (21 inches)
    private static final double CONVERSION_FACTOR_MPS_TO_RPM = 10000 / 47.93;

    private double targetRPM = 0.0;
    private double currentRPM = 0.0;

    private final double maxRpm = Constants.FLYWHEEL_MAX_THEORETICAL_RPM;

    private boolean flywheelEnabled = false; // Button-controlled enable

    // Shuffleboard cached values
    private boolean sdInit = false;

    private double sd_kP, sd_kI, sd_kD;
    private double sd_kS, sd_kV, sd_kA;

    private final TalonFX flywheelMotor;

    private double infeedTargetRPM;

    public Shooter(Drivebase drivebase) {
        this.drivebase = drivebase;
        this.flywheelMotor = new TalonFX(Constants.FLYWHEEL_MOTOR_CAN_ID);

        this.canFlywheel = flywheelMotorTopLeft.isConnected() && flywheelMotorTopRight.isConnected() && flywheelMotorBottomLeft.isConnected() && flywheelMotorBottomRight.isConnected();
        this.canHood = hoodRollerLeft.isConnected() && hoodRollerRight.isConnected();
        this.canInfeed = rollerLeft.isConnected() && rollerRight.isConnected();

        this.hoodTargetAngle = 0;
        this.hoodTargetAngleMotorPosition = 0;
        this.hoodCurrentAngle = 0;
        this.hoodCurrentAngleMotorPosition = 0;

        this.flywheelCurrentRPM = 0;
        this.flywheelTargetRPM = 0;
        this.flywheelError = 0;

        beamBreak = new DigitalInput(3);

        for (int i = 0; i < flywheelMotors.getTotalMotors(); i++) {
            TalonFXConfiguration cfg = new TalonFXConfiguration();

            // Neutral + inversion
            cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

            if (Constants.FLYWHEEL_MOTOR_CLOCKWISE[i]) {
                cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            } else {
                cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            }

            // Slot 0 PID
            cfg.Slot0.kP = Constants.FLYWHEEL_kP;
            cfg.Slot0.kI = Constants.FLYWHEEL_kI;
            cfg.Slot0.kD = Constants.FLYWHEEL_kD;

            // Slot 0 Feedforward (Talon internal)
            cfg.Slot0.kS = Constants.FLYWHEEL_kS;
            cfg.Slot0.kV = Constants.FLYWHEEL_kV;
            cfg.Slot0.kA = Constants.FLYWHEEL_kA;

            // Motion Magic acceleration limits
            cfg.MotionMagic.MotionMagicAcceleration =
                Constants.FLYWHEEL_MOTION_ACCEL_RPMS / 60.0;
            cfg.MotionMagic.MotionMagicJerk =
                Constants.FLYWHEEL_MOTION_JERK;

            if (flywheelMotors.getMotorByIndex(i) != null) {
                flywheelMotors.getMotorByIndex(i).getConfigurator().apply(cfg);
            }
        }

        SmartDashboard.putNumber("Hood Target Position", 0);

        SmartDashboard.putNumber("Flywheel/TargetRPM", Constants.FLYWHEEL_TARGET_RPM);

        SmartDashboard.putNumber("Flywheel/kP", Constants.FLYWHEEL_kP);
        SmartDashboard.putNumber("Flywheel/kI", Constants.FLYWHEEL_kI);
        SmartDashboard.putNumber("Flywheel/kD", Constants.FLYWHEEL_kD);

        SmartDashboard.putNumber("Flywheel/kS", Constants.FLYWHEEL_kS);
        SmartDashboard.putNumber("Flywheel/kV", Constants.FLYWHEEL_kV);
        SmartDashboard.putNumber("Flywheel/kA", Constants.FLYWHEEL_kA);

        sd_kP = Constants.FLYWHEEL_kP;
        sd_kI = Constants.FLYWHEEL_kI;
        sd_kD = Constants.FLYWHEEL_kD;

        sd_kS = Constants.FLYWHEEL_kS;
        sd_kV = Constants.FLYWHEEL_kV;
        sd_kA = Constants.FLYWHEEL_kA;

        sdInit = true;

        SmartDashboard.putNumber("Hood Power", 0.05);
    }

    @Override
    public void periodic() {
        // Flywheel is controlled by TestSubsystem via Constants; no dashboard reads here.
        
        //This line should be all that is needed when the flywheel should be spun up
        //updateLaunchValues(true);

        //Update the beam break sensors
        SmartDashboard.putBoolean("Beam Break", beamBreak.get());


        double measuredRps =
                flywheelMotorTopLeft.getRotorVelocity()
                        .refresh()
                        .getValueAsDouble();

        currentRPM = measuredRps * 60.0;
        SmartDashboard.putNumber("Flywheel measured RPM", currentRPM);

        // -------- Shuffleboard tuning --------

        targetRPM = SmartDashboard.getNumber(
                "Flywheel/TargetRPM",
                Constants.FLYWHEEL_TARGET_RPM);

        double kP = SmartDashboard.getNumber("Flywheel/kP", sd_kP);
        double kI = SmartDashboard.getNumber("Flywheel/kI", sd_kI);
        double kD = SmartDashboard.getNumber("Flywheel/kD", sd_kD);

        double kS = SmartDashboard.getNumber("Flywheel/kS", sd_kS);
        double kV = SmartDashboard.getNumber("Flywheel/kV", sd_kV);
        double kA = SmartDashboard.getNumber("Flywheel/kA", sd_kA);

        // Apply only if changed
        if (!sdInit ||
                kP != sd_kP || kI != sd_kI || kD != sd_kD ||
                kS != sd_kS || kV != sd_kV || kA != sd_kA) {


            for (int i = 0; i < flywheelMotors.getTotalMotors(); i++) {
                TalonFXConfiguration cfg = new TalonFXConfiguration();

                // Neutral + inversion
                cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

                if (Constants.FLYWHEEL_MOTOR_CLOCKWISE[i]) {
                    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                } else {
                    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
                }

                // Slot 0 PID
                cfg.Slot0.kP = Constants.FLYWHEEL_kP;
                cfg.Slot0.kI = Constants.FLYWHEEL_kI;
                cfg.Slot0.kD = Constants.FLYWHEEL_kD;

                // Slot 0 Feedforward (Talon internal)
                cfg.Slot0.kS = Constants.FLYWHEEL_kS;
                cfg.Slot0.kV = Constants.FLYWHEEL_kV;
                cfg.Slot0.kA = Constants.FLYWHEEL_kA;

                // Motion Magic acceleration limits
                cfg.MotionMagic.MotionMagicAcceleration =
                    Constants.FLYWHEEL_MOTION_ACCEL_RPMS / 60.0;
                cfg.MotionMagic.MotionMagicJerk =
                    Constants.FLYWHEEL_MOTION_JERK;

                flywheelMotors.getMotorByIndex(i).getConfigurator().apply(cfg);
            }
            
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

            flywheelMotors.applyControl(req, true);
        } else {
            targetRPS = 0;
            CoastOut req =
                    new CoastOut();

            flywheelMotors.applyControl(req, true);
        }

        double percent = currentRPM / maxRpm;

        SmartDashboard.putNumber(
                "Flywheel/MeasuredRPM",
                currentRPM);

        SmartDashboard.putNumber(
                "Flywheel/PercentOutApprox",
                percent);
    }

    public void updateLaunchValues(boolean interpolate){
        // Calculate distance to goal & diffs
        double xDiff = getGoalPose().getX() - drivebase.getPose().getX();
        double yDiff = getGoalPose().getY() - drivebase.getPose().getY();
        double distToGoal = Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));
        
        // Chose how to get the flywheel speed
        if (interpolate){
            getNeededFlywheelSpeed(distToGoal);
        } else {
            calculateLaunchValues(distToGoal);
        }

    }

    public void calculateLaunchValues(double distToGoal){
        double desiredHeight = distToGoal > 6 ? DESIRED_MAX_HEIGHT : distToGoal / 6;

        // Calculate time of flight and velocity vectors
        double verticalVel = Math.sqrt(2 * GRAVITY * (desiredHeight - FLYWHEEL_HEIGHT));
        double estimatedTime = (verticalVel + Math.sqrt(verticalVel - 2 * (GRAVITY) * (GOAL_HEIGHT - FLYWHEEL_HEIGHT))) / GRAVITY;
        double horizonalVel = distToGoal / estimatedTime;

        // Calculate hood angle and angle to face goal
        hoodTargetAngle = Math.atan(verticalVel / horizonalVel);
        //double angleToFaceGoal = Math.atan2(yDiff, xDiff); I think this is done in a different file
        
        // Calculate flywheel RPM needed
        double initialVel = Math.sqrt(Math.pow((verticalVel), 2) * Math.pow((horizonalVel), 2));
        flywheelTargetRPM = initialVel * CONVERSION_FACTOR_MPS_TO_RPM;
        
        // Set the flywheel Velocity & Hood angle
        // setFlywheelSpeed(flywheelTargetRPM);
    }

    public double getNeededFlywheelSpeed(double distToGoal) {
        return interpolateFlywheelSpeedByDistance(distToGoal) * FLYWHEEL_MAX_THEORETICAL_RPM; // Normalize the target velocity by the max theoretical
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
        double xDiff = getGoalPose().getX() - drivebase.getPose().getX();
        double yDiff = getGoalPose().getY() - drivebase.getPose().getY();

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
            currentDistance = FLYWHEEL_SPEED_TABLE[i];
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

    public void startFlywheel() {
        this.flywheelEnabled = true;
    }

    public void stopFlywheel() {
        this.flywheelEnabled = false;
    }

    public double getFlywheelRPM () {
        return flywheelCurrentRPM;
    }
        public double getFlywheelTatgetRPM () {
        return flywheelTargetRPM;
    }
        public double getFlywheelError () {
        return flywheelError;
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
            rollerMotors.setPower(0.6);
        }
    }

    public void startInfeedWithSpeed(double speed) {
        if (canInfeed) {
            rollerMotors.setPower(speed);
        }
    }

    public void stopInfeed() {
        if (canInfeed) {
            rollerMotors.setPower(0);
        }
    }

    public double getTransferRPM() {
        return rollerLeft.getRotorVelocity(true).getValueAsDouble() * 60;
    }

    public double getTransferCurrent() {
        return rollerLeft.getSupplyCurrent(true).getValueAsDouble() + rollerRight.getSupplyCurrent(true).getValueAsDouble();
    }

    public double getTransferLeftMotorCurrent() {
        return rollerLeft.getSupplyCurrent(true).getValueAsDouble();
    }

    public double getTransferRightMotorCurrent() {
        return rollerRight.getSupplyCurrent(true).getValueAsDouble();
    }

    public void setHoodPower(double power){
        if (canHood) {
            this.hoodRollerLeft.set(power);
            this.hoodRollerRight.set(power);
        }
    }

    public void hoodUp() {
        if (canHood) {
            double power = SmartDashboard.getNumber("Hood Power", 0.05);
            this.hoodRollerLeft.set(power);
            this.hoodRollerRight.set(power);
        }
    }

    public void hoodDown() {
        if (canHood) {
            double power = SmartDashboard.getNumber("Hood Power", 0.05);
            this.hoodRollerLeft.set(-power);
            this.hoodRollerRight.set(-power);
        }
    }

    public void stopHood() {
        if (canHood) {
            this.hoodRollerLeft.set(0);
            this.hoodRollerRight.set(0);
        }
    }
    
    // The position input is between 0 and 1 with 0 being up and 1 being down
    public void setHoodMotorPosition(double position) {
        hoodTargetAngleMotorPosition = position;
    }
    public double getHoodCurrent() {
        return hoodRollerLeft.getSupplyCurrent(true).getValueAsDouble() + hoodRollerRight.getSupplyCurrent(true).getValueAsDouble();
    }

    public double getHoodLeftMotorCurrent() {
        return hoodRollerLeft.getSupplyCurrent(true).getValueAsDouble();
    }

    public double getHoodRightMotorCurrent() {
        return hoodRollerRight.getSupplyCurrent(true).getValueAsDouble();
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

        double rpmMath = calculateRPM(robotPose, hubPose, hoodAngle);
        double interpolatedRPM = interpolateRPM(distance);

        return (rpmMath + interpolatedRPM) / 2.0;
    }

    public void setInfeedRPM(double targetRPM) {
        this.infeedTargetRPM = targetRPM;
        double currentRPM = rollerLeft.getRotorVelocity(true).getValueAsDouble() * 60.0;
        double error = targetRPM - currentRPM;
        double adjustment = Constants.INFEED_kP * error; // Adjustment to approach target
        double newRPM = currentRPM + adjustment; // Adjust current RPM towards target
        rollerMotors.setPower(newRPM / Constants.FLYWHEEL_MAX_THEORETICAL_RPM); // Normalize to motor power
    }
}