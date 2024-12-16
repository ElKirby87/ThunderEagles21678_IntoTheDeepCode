package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {
    CRServo intakeArmLeftServo = null;
    CRServo intakeArmRightServo = null;
    Servo intakeHand = null;
    CRServo intakeHandDoll = null;
    DcMotor intakeTray;

    public void IntakeInit(HardwareMap hardwareMap) {
        intakeArmLeftServo = hardwareMap.get(CRServo.class, "intakeArmLeft");
        intakeArmRightServo = hardwareMap.get(CRServo.class, "intakeArmRight");
        intakeHand = hardwareMap.get(Servo.class, "intakeHand");
        intakeHandDoll = hardwareMap.get(CRServo.class, "intakeHand_Doll");
        intakeTray = hardwareMap.get(DcMotor.class, "deposit");
    }

    public void ActivateArm(double armLeftPower, double armRightPower) {
        intakeArmLeftServo.setPower(armLeftPower);
        intakeArmRightServo.setPower(armRightPower);
    }

    public void OpenHand(double position) {
        intakeHand.setPosition(position);
    }

    public void ActivateHandDoll(double power) {
        intakeHandDoll.setPower(power);
    }

    public void ActivateTray (double power) {
        intakeTray.setPower(power);
    }
}
