package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
public class ClawSubsystem extends SubsystemBase {
    private Servo clawServo = null;
    private CRServo clawArmLeftServo = null;
    private CRServo clawArmRightServo = null;

    public void ClawInit(HardwareMap hardwareMap, Telemetry telemetry) {
        clawServo = hardwareMap.get(Servo.class, "claw");
        clawArmLeftServo = hardwareMap.get(CRServo.class, "clawArmLeft");
        clawArmRightServo = hardwareMap.get(CRServo.class, "clawArmRight");
    }

    public void OpenClaw(double position) {
        clawServo.setPosition(position);
    }

    public void ActivateArm(double armLeftPower, double armRightPower) {
        clawArmLeftServo.setPower(armLeftPower);
        clawArmRightServo.setPower(armRightPower);
    }
}
