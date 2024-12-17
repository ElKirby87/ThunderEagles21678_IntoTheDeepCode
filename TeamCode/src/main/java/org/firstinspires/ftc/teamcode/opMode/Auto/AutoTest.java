package org.firstinspires.ftc.teamcode.opMode.Auto;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous (name = "AutoTest")
public class AutoTest extends OpMode {
    private Limelight3A limelight3A;
    private final VisionSubsystem m_visionsubsystem = new VisionSubsystem();
    private final MecanumDrive m_mecanumdrive = new MecanumDrive();

    @Override
    public void init() {
        m_visionsubsystem.VisionInit(hardwareMap, telemetry, m_mecanumdrive);
        m_mecanumdrive.DriveInit(hardwareMap);
    }

    @Override
    public void loop() {
        m_visionsubsystem.VisionActivate(telemetry);
    }
}
