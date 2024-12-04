package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase {

    Servo clawServo = null;
    public void ClawInit(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "claw");
    }

    public void OpenClaw(boolean isOpen) {
        if (isOpen) {
            clawServo.setPosition(1.0);
        } else {
            clawServo.setPosition(0.0);
        }
    }

    public void ResetClaw() {
        clawServo.setPosition(0.5);
    }
}
