package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {
    CRServo intakeArmLeftServo;
    CRServo intakeArmRightServo;

    public void IntakeInit(HardwareMap hardwareMap) {
        intakeArmLeftServo = hardwareMap.get(CRServo.class, "intakeArmLeft");
        intakeArmRightServo = hardwareMap.get(CRServo.class, "intakeArmRight");
    }

    public void ActivateArm(double armLeftPower, double armRightPower) {
        intakeArmLeftServo.setPower(armLeftPower);
        intakeArmRightServo.setPower(armRightPower);
    }
}
