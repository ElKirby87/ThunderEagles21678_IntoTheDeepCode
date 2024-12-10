package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase {

    CRServo clawServo;
    CRServo clawArmLeftServo;
    CRServo clawArmRightServo;
    public void ClawInit(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(CRServo.class, "claw");
        clawArmLeftServo = hardwareMap.get(CRServo.class, "clawArmLeft");
        clawArmRightServo = hardwareMap.get(CRServo.class, "clawArmRight");
    }

    public void OpenClaw(double power) {
        clawServo.setPower(1);
    }

    public void ActivateArm(double armLeftPower, double armRightPower) {
        clawArmLeftServo.setPower(armLeftPower);
        clawArmRightServo.setPower(armRightPower);
    }
}
