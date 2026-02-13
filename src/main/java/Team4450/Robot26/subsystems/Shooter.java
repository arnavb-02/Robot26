package Team4450.Robot26.subsystems;

import static Team4450.Robot26.Constants.*;

import com.ctre.phoenix6.hardware.TalonFX;

import Team4450.Robot26.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import com.ctre.phoenix6.CANBus;

public class Shooter extends SubsystemBase {
    // This motor is a Falcon 500
    private final TalonFX flywheelMotorTopLeft = new TalonFX(Constants.FLYWHEEL_MOTOR_TOP_LEFT_CAN_ID, new CANBus(Constants.CANIVORE_NAME));
    // This motor is a Falcon 500
    private final TalonFX flywheelMotorTopRight = new TalonFX(Constants.FLYWHEEL_MOTOR_TOP_RIGHT_CAN_ID, new CANBus(Constants.CANIVORE_NAME));
    // This motor is a Falcon 500
    private final TalonFX flywheelMotorBottomLeft = new TalonFX(Constants.FLYWHEEL_MOTOR_BOTTOM_LEFT_CAN_ID, new CANBus(Constants.CANIVORE_NAME));
    // This motor is Falcon 500
    private final TalonFX flywheelMotorBottomRight = new TalonFX(Constants.FLYWHEEL_MOTOR_BOTTOM_RIGHT_CAN_ID, new CANBus(Constants.CANIVORE_NAME));
    // This motor is a Kraken x60, change CAN ID
    private final TalonFX hoodRollerLeft = new TalonFX(Constants.HOOD_MOTOR_LEFT_CAN_ID, new CANBus(Constants.CANIVORE_NAME));
    // This motor is a Kraken x60, change CAN ID
    private final TalonFX hoodRollerRight = new TalonFX(Constants.HOOD_MOTOR_RIGHT_CAN_ID, new CANBus(Constants.CANIVORE_NAME));
    // This motor is a Kraken x44, change CAN ID
    private final TalonFX rollerLeft = new TalonFX(Constants.ROLLER_MOTOR_LEFT_CAN_ID, new CANBus(Constants.CANIVORE_NAME));
    // This motor is a Kraken x44, change CAN ID
    private final TalonFX rollerRight = new TalonFX(Constants.ROLLER_MOTOR_RIGHT_CAN_ID, new CANBus(Constants.CANIVORE_NAME));

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

    public Shooter(Drivebase drivebase) {
        // initialize commanded angle to whatever a reasonable default is
        this.drivebase = drivebase;

        this.hoodTargetAngle = 0;
        this.hoodTargetAngleMotorPosition = 0;
        this.hoodCurrentAngle = 0;
        this.hoodCurrentAngleMotorPosition = 0;

        this.flywheelCurrentRPM = 0;
        this.flywheelTargetRPM = 0;
        this.flywheelError = 0;

        beamBreak = new DigitalInput(3);
    }

    @Override
    public void periodic() {
        // Flywheel is controlled by TestSubsystem via Constants; no dashboard reads here.
        
        //This line should be all that is needed when the flywheel should be spun up
        //updateLaunchValues(true);

        //Update the beam break sensors
        SmartDashboard.putBoolean("Beam Break", beamBreak.get());

        flywheelCurrentRPM = flywheelMotorBottomLeft.getRotorVelocity(true).getValueAsDouble() * 60;
        flywheelError = flywheelTargetRPM - flywheelCurrentRPM;
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
        setFlywheelSpeed(flywheelTargetRPM);
        setHoodAngle(hoodTargetAngle);
    }

    public void setHoodAngle(double targetAngle){
        this.hoodTargetAngleMotorPosition = this.hoodAngleToMotorPosition(this.hoodTargetAngle);
        // Convert position input to rotations for the motor
        double power = SmartDashboard.getNumber("pivit MotorPower", Constants.INTAKE_PIVIT_MOTOR_POWER);
        if (this.hoodCurrentAngleMotorPosition <= this.hoodTargetAngleMotorPosition - Constants.HOOD_TOLERENCE_MOTOR_ROTATIONS) {
                setHoodPower(power);
            } else if (this.hoodCurrentAngleMotorPosition >= this.hoodTargetAngleMotorPosition + Constants.HOOD_TOLERENCE_MOTOR_ROTATIONS) {
                setHoodPower(-power);
            } else {
                setHoodPower(0);
            }

            this.hoodCurrentAngleMotorPosition = this.getHoodAngle();
            this.hoodCurrentAngle = this.motorPositionToHoodAngle(this.hoodCurrentAngleMotorPosition);
            SmartDashboard.putNumber("Hood current angle", this.hoodCurrentAngle);
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


    public void setFlywheelSpeed(double targetFlywheelSpeed) {
        flywheelMotorBottomLeft.set(targetFlywheelSpeed);
        flywheelMotorBottomRight.set(targetFlywheelSpeed);
        flywheelMotorTopLeft.set(targetFlywheelSpeed);
        flywheelMotorTopRight.set(targetFlywheelSpeed);
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

    public double getFlywheelVoltage() {
        return flywheelMotorBottomLeft.getSupplyVoltage(true).getValueAsDouble() + flywheelMotorBottomRight.getSupplyVoltage(true).getValueAsDouble() + flywheelMotorTopLeft.getSupplyVoltage(true).getValueAsDouble() + flywheelMotorTopRight.getSupplyVoltage(true).getValueAsDouble();
    }

    public double getFlywheelTopLeftMotorVoltage() {
        return flywheelMotorTopLeft.getSupplyVoltage(true).getValueAsDouble();
    }

    public double getFlywheelTopRightMotorVoltage() {
        return flywheelMotorTopRight.getSupplyVoltage(true).getValueAsDouble();
    }

    public double getFlywheelBottomLeftMotorVoltage() {
        return flywheelMotorBottomLeft.getSupplyVoltage(true).getValueAsDouble();
    }

    public double getFlywheelBottomRightMotorVoltage() {
        return flywheelMotorBottomRight.getSupplyVoltage(true).getValueAsDouble();
    }

    public void startTransfer() {
        rollerLeft.set(1);
        rollerRight.set(1);
    }

    public void startTransferWithSpeed(double speed) {
        rollerLeft.set(speed);
        rollerRight.set(speed);
    }

    public void stopTransfer() {
        rollerLeft.set(0);
        rollerRight.set(0);
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

    public double getTransferVoltage() {
        return rollerLeft.getSupplyVoltage(true).getValueAsDouble() + rollerRight.getSupplyVoltage(true).getValueAsDouble();
    }

    public double getTransferLeftMotorVoltage() {
        return rollerLeft.getSupplyVoltage(true).getValueAsDouble();
    }

    public double getTransferRightMotorVoltage() {
        return rollerRight.getSupplyVoltage(true).getValueAsDouble();
    }

    public void setHoodPower(double power){
        this.hoodRollerLeft.set(power);
        this.hoodRollerRight.set(power);
    }
    
    // The position input is between 0 and 1 with 0 being up and 1 being down
    public void setHoodMotorPosition(double position) {
        hoodTargetAngleMotorPosition = position;
    }

    public double getHoodAngle() {
        return hoodCurrentAngle;
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

    public double getHoodVoltage() {
        return hoodRollerLeft.getSupplyVoltage(true).getValueAsDouble() + hoodRollerRight.getSupplyVoltage(true).getValueAsDouble();
    }

    public double getHoodLeftMotorVoltage() {
        return hoodRollerLeft.getSupplyVoltage(true).getValueAsDouble();
    }

    public double getHoodRightMotorVoltage() {
        return hoodRollerRight.getSupplyVoltage(true).getValueAsDouble();
    }
}
