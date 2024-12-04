package org.firstinspires.ftc.teamcode.hardware.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

public class ClawCommand extends CommandBase {

    ClawSubsystem clawSubsystem;
    boolean isOpen;

    public ClawCommand (ClawSubsystem clawSubsystem, boolean isOpen) {
        this.clawSubsystem = clawSubsystem;
        addRequirements(clawSubsystem);
        this.isOpen = isOpen;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        clawSubsystem.OpenClaw(isOpen);
    }

    @Override
    public void end(boolean interrumped) {
        clawSubsystem.ResetClaw();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
