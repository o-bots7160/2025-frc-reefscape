package frc.robot.devices;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;

public class ElevatorMotors implements MotorControl {

    // This was calculated off measuring from encoder to tape measure
    // private static final double conversionFactor = 40.0/81.503/3.0;
    private static final double conversionFactor = 94.60 / 67.0;

    private ElevatorMotor       primaryMotor;

    private ElevatorMotor       secondaryMotor;

    private double              minimumTargetPosition;

    private double              maximumTargetPosition;

    public ElevatorMotors(int primaryMotorCanId, int secondaryMotorCanId, double minimumTargetPosition, double maximumTargetPosition) {
        this.minimumTargetPosition = minimumTargetPosition;
        this.maximumTargetPosition = maximumTargetPosition;
        primaryMotor               = new ElevatorMotor("PrimaryElevatorMotor", primaryMotorCanId, minimumTargetPosition, maximumTargetPosition,
                conversionFactor);
        secondaryMotor             = new ElevatorMotor("SecondaryElevatorMotor", secondaryMotorCanId, minimumTargetPosition, maximumTargetPosition,
                conversionFactor);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // do nothing
    }

    @Override
    public void setVoltage(Voltage voltage) {
        primaryMotor.setVoltage(voltage);
        secondaryMotor.setVoltage(voltage);
    }

    @Override
    public void setVoltage(double voltage) {
        primaryMotor.setVoltage(voltage);
        secondaryMotor.setVoltage(voltage);
    }

    @Override
    public double getMaximumTargetPosition() {
        return maximumTargetPosition;
    }

    @Override
    public double getMinimumTargetPosition() {
        return minimumTargetPosition;
    }

    @Override
    public double getEncoderPosition() {
        return primaryMotor.getEncoderPosition();
    }

    @Override
    public double getEncoderVelocity() {
        return primaryMotor.getEncoderVelocity();
    }

    @Override
    public Voltage getVoltage() {
        return primaryMotor.getVoltage();
    }

    @Override
    public void stop() {
        primaryMotor.stop();
        secondaryMotor.stop();
    }

    @Override
    public SparkMax getMotor() {
        return primaryMotor.getMotor();
    }
}