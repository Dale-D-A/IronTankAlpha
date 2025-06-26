// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.                               

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestSubsystem extends SubsystemBase {
    /** Creates new TalonFX motor */
    TalonFX motorTest = new TalonFX(2);
    TalonFXSimState motorTestSimState = motorTest.getSimState();

    /** Creates a new TestSubsystem. */
    public TestSubsystem() {
        // Configure the motor
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Set motor inversion
        motorTest.getConfigurator().apply(motorConfig);

        motorTest.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake); // Set brake mode
    }

    /** Run motor according to joystick input values. */
    public void runMotor(double speed) {
        motorTest.setControl(new DutyCycleOut(speed)); // Set motor speed
    }

    /** Stop the motor. */
    public void stopMotor() {
        motorTest.setControl(new DutyCycleOut(0)); // Stop the motor
    }
}