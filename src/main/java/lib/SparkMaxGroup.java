package lib;

import com.revrobotics.*;

public class SparkMaxGroup {

	private final CANSparkMax follow;
	private final CANSparkMax leader;

	public SparkMaxGroup(CANSparkMax motor1,boolean Motor1Inverted, CANSparkMax motor2, boolean Motor2Inverted)
	{
		if (Motor1Inverted )
		{
			follow = motor2;
			leader = motor1;
			follow.follow(leader, true);
		} else if (Motor2Inverted) {
			follow = motor1;
			leader = motor2;
			follow.follow(leader, true);
		}else{
			leader = motor1;
			follow = motor2;
			follow.follow(leader);
		}
	}
	
	public void set(double speed) {
		leader.set(speed);
	}

	public void setVoltage(double outputVolts) {
		leader.setVoltage(outputVolts);
	}

	public double get() {
		return leader.get();
	}

	public void setInverted(boolean isInverted) {
		leader.setInverted(isInverted);
	}

	public boolean getInverted() {
		return leader.getInverted();
	}

	public void disable() {
		leader.disable();
	}

	public void stopMotor() {
		leader.stopMotor();
	}

	public RelativeEncoder getEncoder() {
		return leader.getEncoder();
	}

	public RelativeEncoder getEncoder(SparkMaxRelativeEncoder.Type encoderType, int countsPerRev) {
		return leader.getEncoder(encoderType, countsPerRev);
	}

	public RelativeEncoder getAlternateEncoder(int countsPerRev) {
		return leader.getAlternateEncoder(countsPerRev);
	}

	public RelativeEncoder getAlternateEncoder(SparkMaxAlternateEncoder.Type encoderType, int countsPerRev) {
		return leader.getAlternateEncoder(encoderType, countsPerRev);
	}

	public SparkMaxAnalogSensor getAnalog(SparkMaxAnalogSensor.Mode mode) {
		return leader.getAnalog(mode);
	}

	public SparkMaxAbsoluteEncoder getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type encoderType) {
		return leader.getAbsoluteEncoder(encoderType);
	}

	public SparkMaxPIDController getPIDController() {
		return leader.getPIDController();
	}

	public SparkMaxLimitSwitch getForwardLimitSwitch(SparkMaxLimitSwitch.Type switchType) {
		return leader.getForwardLimitSwitch(switchType);
	}

	public SparkMaxLimitSwitch getReverseLimitSwitch(SparkMaxLimitSwitch.Type switchType) {
		return leader.getReverseLimitSwitch(switchType);
	}

	public REVLibError setSmartCurrentLimit(int limit) {
		return leader.setSmartCurrentLimit(limit);
	}

	public REVLibError setSmartCurrentLimit(int stallLimit, int freeLimit) {
		return leader.setSmartCurrentLimit(stallLimit, freeLimit);
	}

	public REVLibError setSmartCurrentLimit(int stallLimit, int freeLimit, int limitRPM) {
		return leader.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
	}

	public REVLibError setSecondaryCurrentLimit(double limit) {
		return leader.setSecondaryCurrentLimit(limit);
	}

	public REVLibError setSecondaryCurrentLimit(double limit, int chopCycles) {
		return leader.setSecondaryCurrentLimit(limit, chopCycles);
	}

	public REVLibError setIdleMode(CANSparkMax.IdleMode mode) {
		return leader.setIdleMode(mode);
	}

	public CANSparkMax.IdleMode getIdleMode() {
		return leader.getIdleMode();
	}

	public REVLibError enableVoltageCompensation(double nominalVoltage) {
		return leader.enableVoltageCompensation(nominalVoltage);
	}

	public REVLibError disableVoltageCompensation() {
		return leader.disableVoltageCompensation();
	}

	public double getVoltageCompensationNominalVoltage() {
		return leader.getVoltageCompensationNominalVoltage();
	}

	public REVLibError setOpenLoopRampRate(double rate) {
		return leader.setOpenLoopRampRate(rate);
	}

	public REVLibError setClosedLoopRampRate(double rate) {
		return leader.setClosedLoopRampRate(rate);
	}

	public double getOpenLoopRampRate() {
		return leader.getOpenLoopRampRate();
	}

	public double getClosedLoopRampRate() {
		return leader.getClosedLoopRampRate();
	}

	public REVLibError follow(CANSparkMax leader) {
		return this.leader.follow(leader);
	}

	public REVLibError follow(CANSparkMax leader, boolean invert) {
		return this.leader.follow(leader, invert);
	}

	public REVLibError follow(CANSparkMax.ExternalFollower leader, int deviceID) {
		return this.leader.follow(leader, deviceID);
	}

	public REVLibError follow(CANSparkMax.ExternalFollower leader, int deviceID, boolean invert) {
		return this.leader.follow(leader, deviceID, invert);
	}

	public boolean isFollower() {
		return leader.isFollower();
	}

	public short getFaults() {
		return leader.getFaults();
	}

	public short getStickyFaults() {
		return leader.getStickyFaults();
	}

	public boolean getFault(CANSparkMax.FaultID faultID) {
		return leader.getFault(faultID);
	}

	public boolean getStickyFault(CANSparkMax.FaultID faultID) {
		return leader.getStickyFault(faultID);
	}

	public double getBusVoltage() {
		return leader.getBusVoltage();
	}

	public double getAppliedOutput() {
		return leader.getAppliedOutput();
	}

	public double getOutputCurrent() {
		return leader.getOutputCurrent();
	}

	public double getMotorTemperature() {
		return leader.getMotorTemperature();
	}

	public REVLibError clearFaults() {
		return leader.clearFaults();
	}

	public REVLibError burnFlash() {
		return leader.burnFlash();
	}

	public REVLibError setCANTimeout(int milliseconds) {
		return leader.setCANTimeout(milliseconds);
	}

	public REVLibError enableSoftLimit(CANSparkMax.SoftLimitDirection direction, boolean enable) {
		return leader.enableSoftLimit(direction, enable);
	}

	public REVLibError setSoftLimit(CANSparkMax.SoftLimitDirection direction, float limit) {
		return leader.setSoftLimit(direction, limit);
	}

	public double getSoftLimit(CANSparkMax.SoftLimitDirection direction) {
		return leader.getSoftLimit(direction);
	}

	public boolean isSoftLimitEnabled(CANSparkMax.SoftLimitDirection direction) {
		return leader.isSoftLimitEnabled(direction);
	}

	public REVLibError getLastError() {
		return leader.getLastError();
	}

	public void close() {
		leader.close();
	}

	public int getFirmwareVersion() {
		return leader.getFirmwareVersion();
	}

	public void setControlFramePeriodMs(int periodMs) {
		leader.setControlFramePeriodMs(periodMs);
	}

	public String getFirmwareString() {
		return leader.getFirmwareString();
	}

	public byte[] getSerialNumber() {
		return leader.getSerialNumber();
	}

	public int getDeviceId() {
		return leader.getDeviceId();
	}

	public CANSparkMaxLowLevel.MotorType getMotorType() {
		return leader.getMotorType();
	}

	public REVLibError setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame frame, int periodMs) {
		return leader.setPeriodicFramePeriod(frame, periodMs);
	}

	public float getSafeFloat(float f) {
		return leader.getSafeFloat(f);
	}

	public REVLibError restoreFactoryDefaults() {
		return leader.restoreFactoryDefaults();
	}

	public REVLibError restoreFactoryDefaults(boolean persist) {
		return leader.restoreFactoryDefaults(persist);
	}



}
