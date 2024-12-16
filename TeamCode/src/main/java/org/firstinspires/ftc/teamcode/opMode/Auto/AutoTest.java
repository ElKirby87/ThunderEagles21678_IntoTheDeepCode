package org.firstinspires.ftc.teamcode.opMode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Autonomous (name = "AutoTest")
public class AutoTest extends OpMode {
    private VisionSubsystem m_visionsubsystem = new VisionSubsystem();

    @Override
    public void init() {
        m_visionsubsystem.VisionInit(hardwareMap);
    }

    @Override
    public void loop() {
        m_visionsubsystem.VisionActivate();
    }
}
