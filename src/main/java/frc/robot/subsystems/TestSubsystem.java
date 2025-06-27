// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.                               

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.consts; // Importing constants from consts.java

public class TestSubsystem extends SubsystemBase {
    /** Creates new TalonFX motor */
    TalonFX motorTest = new TalonFX(2);
    TalonFXSimState motorTestSimState = motorTest.getSimState();

    /** Creates a new TestSubsystem. */
    public TestSubsystem() {
        // Configure the motor
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Set motor inversion
        motorConfig.withSlot0(new Slot0Configs()
            .withKP(consts.PID.KP) // Set proportional gain
            .withKD(consts.PID.KD) // Set derivative gain
            .withKI(consts.PID.KI) // Set integral gain)
            );
        motorTest.getConfigurator().apply(motorConfig);

        motorTest.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake); // Set brake mode
    }

    /** Dynamically change PID */
    public void setPID(double kp, double ki, double kd) {
        var config = new Slot0Configs().withKP(kp).withKI(ki).withKD(kd);
        motorTest.getConfigurator().apply(config);
    }    

    /** Run motor according to joystick input values. */
    public void setMotor(double position) {
        motorTest.setControl(new PositionVoltage(position)); // Set motor speed
    }

    /** Stop the motor. */
    public void stopMotor() {
        motorTest.setControl(new DutyCycleOut(0)); // Stop the motor
    }

    public void resetMotor() {
        setMotor(0);
    }

    @Override
    public void periodic() {
        // Logging
        Logger.recordOutput("TestMotor/Output", motorTest.get());
        Logger.recordOutput("TestMotor/Position", motorTest.getPosition().getValueAsDouble());
        Logger.recordOutput("TestMotor/Velocity", motorTest.getVelocity().getValueAsDouble());
        Logger.recordOutput("TestMotor/Acceleraation", motorTest.getAcceleration().getValueAsDouble());
        Logger.recordOutput("TestMotor/Temperature", motorTest.getDeviceTemp().getValueAsDouble());
        Logger.recordOutput("TestMotor/Current", motorTest.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput("TestMotor/Voltage", motorTest.getSupplyVoltage().getValueAsDouble());
        Logger.recordOutput("TestMotor/TorqueCurrent", motorTest.getTorqueCurrent().getValueAsDouble());
    }
}