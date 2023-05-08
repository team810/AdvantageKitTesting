package lib.SparkMax;

import com.revrobotics.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class AdvancedSparkMax extends CANSparkMax {

	public AdvancedSparkMax(int deviceId, MotorType type) {
		super(deviceId, type);
	}

	public REVLibError setIdleMode(AdvancedIdleMode mode, Subsystem required) {
		switch (mode)
		{
			case kBreak:
				return super.setIdleMode(IdleMode.kBrake);
			case kCoast:
				return super.setIdleMode(IdleMode.kCoast);
			case kHardBreak:
				CommandScheduler.getInstance().schedule(new HardBreakCommand(this, required));
				return super.setIdleMode(IdleMode.kBrake);
		}
		return super.setIdleMode(IdleMode.kBrake);
	}

	@Override
	public void close() {
		super.close();
	}

	@Override
	public int getFirmwareVersion() {
		return super.getFirmwareVersion();
	}

	@Override
	public void setControlFramePeriodMs(int periodMs) {
		super.setControlFramePeriodMs(periodMs);
	}

	@Override
	public String getFirmwareString() {
		return super.getFirmwareString();
	}

	@Override
	public byte[] getSerialNumber() {
		return super.getSerialNumber();
	}

	@Override
	public int getDeviceId() {
		return super.getDeviceId();
	}

	@Override
	public MotorType getMotorType() {
		return super.getMotorType();
	}

	@Override
	public REVLibError setPeriodicFramePeriod(PeriodicFrame frame, int periodMs) {
		return super.setPeriodicFramePeriod(frame, periodMs);
	}

	@Override
	public float getSafeFloat(float f) {
		return super.getSafeFloat(f);
	}

	@Override
	public REVLibError restoreFactoryDefaults() {
		return super.restoreFactoryDefaults();
	}

	@Override
	public REVLibError restoreFactoryDefaults(boolean persist) {
		return super.restoreFactoryDefaults(persist);
	}

	@Override
	public void set(double speed) {
		super.set(speed);
	}

	@Override
	public void setVoltage(double outputVolts) {
		super.setVoltage(outputVolts);
	}

	@Override
	public double get() {
		return super.get();
	}

	@Override
	public void setInverted(boolean isInverted) {
		super.setInverted(isInverted);
	}

	@Override
	public boolean getInverted() {
		return super.getInverted();
	}

	@Override
	public void disable() {
		super.disable();
	}

	@Override
	public void stopMotor() {
		super.stopMotor();
	}

	@Override
	public RelativeEncoder getEncoder() {
		return super.getEncoder();
	}

	@Override
	public RelativeEncoder getEncoder(SparkMaxRelativeEncoder.Type encoderType, int countsPerRev) {
		return super.getEncoder(encoderType, countsPerRev);
	}

	@Override
	public RelativeEncoder getAlternateEncoder(int countsPerRev) {
		return super.getAlternateEncoder(countsPerRev);
	}

	@Override
	public RelativeEncoder getAlternateEncoder(SparkMaxAlternateEncoder.Type encoderType, int countsPerRev) {
		return super.getAlternateEncoder(encoderType, countsPerRev);
	}

	@Override
	public SparkMaxAnalogSensor getAnalog(SparkMaxAnalogSensor.Mode mode) {
		return super.getAnalog(mode);
	}

	@Override
	public SparkMaxAbsoluteEncoder getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type encoderType) {
		return super.getAbsoluteEncoder(encoderType);
	}

	@Override
	public SparkMaxPIDController getPIDController() {
		return super.getPIDController();
	}
	@Override
	public SparkMaxLimitSwitch getForwardLimitSwitch(SparkMaxLimitSwitch.Type switchType) {
		return super.getForwardLimitSwitch(switchType);
	}

	@Override
	public SparkMaxLimitSwitch getReverseLimitSwitch(SparkMaxLimitSwitch.Type switchType) {
		return super.getReverseLimitSwitch(switchType);
	}

	@Override
	public REVLibError setSmartCurrentLimit(int limit) {
		return super.setSmartCurrentLimit(limit);
	}

	@Override
	public REVLibError setSmartCurrentLimit(int stallLimit, int freeLimit) {
		return super.setSmartCurrentLimit(stallLimit, freeLimit);
	}

	@Override
	public REVLibError setSmartCurrentLimit(int stallLimit, int freeLimit, int limitRPM) {
		return super.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
	}

	@Override
	public REVLibError setSecondaryCurrentLimit(double limit) {
		return super.setSecondaryCurrentLimit(limit);
	}

	@Override
	public REVLibError setSecondaryCurrentLimit(double limit, int chopCycles) {
		return super.setSecondaryCurrentLimit(limit, chopCycles);
	}

	@Override
	public IdleMode getIdleMode() {
		return super.getIdleMode();
	}

	@Override
	public REVLibError enableVoltageCompensation(double nominalVoltage) {
		return super.enableVoltageCompensation(nominalVoltage);
	}

	@Override
	public REVLibError disableVoltageCompensation() {
		return super.disableVoltageCompensation();
	}

	@Override
	public double getVoltageCompensationNominalVoltage() {
		return super.getVoltageCompensationNominalVoltage();
	}

	@Override
	public REVLibError setOpenLoopRampRate(double rate) {
		return super.setOpenLoopRampRate(rate);
	}

	@Override
	public REVLibError setClosedLoopRampRate(double rate) {
		return super.setClosedLoopRampRate(rate);
	}

	@Override
	public double getOpenLoopRampRate() {
		return super.getOpenLoopRampRate();
	}

	@Override
	public double getClosedLoopRampRate() {
		return super.getClosedLoopRampRate();
	}

	@Override
	public REVLibError follow(CANSparkMax leader) {
		return super.follow(leader);
	}

	@Override
	public REVLibError follow(CANSparkMax leader, boolean invert) {
		return super.follow(leader, invert);
	}

	@Override
	public REVLibError follow(ExternalFollower leader, int deviceID) {
		return super.follow(leader, deviceID);
	}

	@Override
	public REVLibError follow(ExternalFollower leader, int deviceID, boolean invert) {
		return super.follow(leader, deviceID, invert);
	}

	@Override
	public boolean isFollower() {
		return super.isFollower();
	}

	@Override
	public short getFaults() {
		return super.getFaults();
	}

	@Override
	public short getStickyFaults() {
		return super.getStickyFaults();
	}

	@Override
	public boolean getFault(FaultID faultID) {
		return super.getFault(faultID);
	}

	@Override
	public boolean getStickyFault(FaultID faultID) {
		return super.getStickyFault(faultID);
	}

	@Override
	public double getBusVoltage() {
		return super.getBusVoltage();
	}

	@Override
	public double getAppliedOutput() {
		return super.getAppliedOutput();
	}

	@Override
	public double getOutputCurrent() {
		return super.getOutputCurrent();
	}

	@Override
	public double getMotorTemperature() {
		return super.getMotorTemperature();
	}

	@Override
	public REVLibError clearFaults() {
		return super.clearFaults();
	}

	@Override
	public REVLibError burnFlash() {
		return super.burnFlash();
	}

	@Override
	public REVLibError setCANTimeout(int milliseconds) {
		return super.setCANTimeout(milliseconds);
	}

	@Override
	public REVLibError enableSoftLimit(SoftLimitDirection direction, boolean enable) {
		return super.enableSoftLimit(direction, enable);
	}

	@Override
	public REVLibError setSoftLimit(SoftLimitDirection direction, float limit) {
		return super.setSoftLimit(direction, limit);
	}

	@Override
	public double getSoftLimit(SoftLimitDirection direction) {
		return super.getSoftLimit(direction);
	}

	@Override
	public boolean isSoftLimitEnabled(SoftLimitDirection direction) {
		return super.isSoftLimitEnabled(direction);
	}

	@Override
	protected int getFeedbackDeviceID() {
		return super.getFeedbackDeviceID();
	}

	@Override
	public REVLibError getLastError() {
		return super.getLastError();
	}
}
