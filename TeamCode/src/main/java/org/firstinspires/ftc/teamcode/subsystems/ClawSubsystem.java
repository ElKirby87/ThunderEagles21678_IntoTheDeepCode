package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ClawSubsystem extends SubsystemBase {
    CRServo clawServo = null;
    CRServo clawArmLeftServo = null;
    CRServo clawArmRightServo = null;
    DcMotor elevatorLeft = null;
    DcMotor elevatorRight = null;

    public void ClawInit(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(CRServo.class, "claw");
        clawArmLeftServo = hardwareMap.get(CRServo.class, "clawArmLeft");
        clawArmRightServo = hardwareMap.get(CRServo.class, "clawArmRight");
        elevatorLeft = hardwareMap.get(DcMotor.class, "leftElevator");
        elevatorRight = hardwareMap.get(DcMotor.class, "rightElevator");

        elevatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void OpenClaw(double power) {
        clawServo.setPower(power);
    }

    public void ActivateArm(double armLeftPower, double armRightPower) {
        clawArmLeftServo.setPower(armLeftPower);
        clawArmRightServo.setPower(armRightPower);
    }

    public void ActivateElevator(double leftPower, double rightPower) {
        elevatorLeft.setPower(leftPower);
        elevatorRight.setPower(rightPower);
    }
}
