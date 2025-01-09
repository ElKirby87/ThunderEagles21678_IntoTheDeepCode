package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.util.Timing;

import java.util.Timer;

public class AutoElevatorSubsystem {
    private ElevatorSubsystem m_elevatorSusbsystem;
    private ClawSubsystem m_clawSubsystem;
    private IntakeSubsystem m_intakeSubsystem;
    private final int initPosition = 10; //centimeters
    private final int takeSamplePosition = 15; //centimeters
    private final int takeSampleTolerance = 2;
    private final int intakeTolerance = 50; //ticks

    public void AutoElevatorInit(ElevatorSubsystem m_elevatorSubsystem, ClawSubsystem m_clawSubsystem, IntakeSubsystem m_intakeSubsystem) {
        this.m_clawSubsystem = m_clawSubsystem;
        this.m_elevatorSusbsystem = m_elevatorSubsystem;
        this.m_intakeSubsystem = m_intakeSubsystem;
    }

    public void InicializePosition() {
        m_clawSubsystem.OpenClaw(1);
        m_elevatorSusbsystem.setDistance(initPosition);
        m_clawSubsystem.ActivateArm(1, -1);
    }

    public void IntakeVerify() {
        if (m_intakeSubsystem.ColorVerify()) {
            if (m_intakeSubsystem.CurrentPosition() <= intakeTolerance) {
                TakeSample();
            }
        }
    }

    public void TakeSample() {
        m_elevatorSusbsystem.setDistance(takeSamplePosition);
        if ((m_elevatorSusbsystem.getCurrentPosition() - takeSamplePosition) <= takeSampleTolerance) {
            m_clawSubsystem.OpenClaw(1);
        }
    }
}
