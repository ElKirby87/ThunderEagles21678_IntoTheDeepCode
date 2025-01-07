package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {
    CRServo intakeArmLeftServo = null;
    CRServo intakeArmRightServo = null;
    DcMotor intakeTray = null;
    DcMotor intake = null;

    public void IntakeInit(HardwareMap hardwareMap) {
        intakeArmLeftServo = hardwareMap.get(CRServo.class, "intakeArmLeft");
        intakeArmRightServo = hardwareMap.get(CRServo.class, "intakeArmRight");
        intakeTray = hardwareMap.get(DcMotor.class, "deposit");
        intake = hardwareMap.get(DcMotor.class, "intake");
    }

    public void ActivateArm(double armLeftPower, double armRightPower) {
        intakeArmLeftServo.setPower(armLeftPower);
        intakeArmRightServo.setPower(armRightPower);
    }

    public void ActivateIntake(double power) { intake.setPower(power); }

    public void ActivateTray (double power) {
        intakeTray.setPower(power);
    }
}
