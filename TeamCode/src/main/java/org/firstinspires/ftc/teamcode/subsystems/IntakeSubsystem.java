package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
    private CRServo intakeArmLeftServo = null;
    private CRServo intakeArmRightServo = null;
    private DcMotor intakeTray = null;
    private DcMotor intake = null;
    private ColorSensor colorSensor;
    private Telemetry telemetry;

    public void IntakeInit(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intakeArmLeftServo = hardwareMap.get(CRServo.class, "intakeArmLeft");
        intakeArmRightServo = hardwareMap.get(CRServo.class, "intakeArmRight");
        intakeTray = hardwareMap.get(DcMotor.class, "deposit");
        intake = hardwareMap.get(DcMotor.class, "intake");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorín");

        intakeTray.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeTray.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void ActivateArm(double armLeftPower, double armRightPower) {
        intakeArmLeftServo.setPower(armLeftPower);
        intakeArmRightServo.setPower(armRightPower);
    }

    public void ActivateIntake(double power) { intake.setPower(power); }

    public void ActivateTray (double power) {
        intakeTray.setPower(power);
    }

    public void ColorTest() {
        String colorDetected = "Unknown";
        if (colorSensor.red() > colorSensor.green() && colorSensor.red() > colorSensor.blue()) {
            colorDetected = "RED";
        } else if (colorSensor.blue() > colorSensor.green() && colorSensor.blue() > colorSensor.red()) {
            colorDetected = "BLUE";
        }else if (colorSensor.green() > colorSensor.blue() && colorSensor.red() > colorSensor.blue()) {
            colorDetected = "YELLOW";
        }
        else {
            colorDetected = "Unknown";
        }
        telemetry.addData("LighDetected", (colorSensor.argb()));
        telemetry.addData("LighDetected", (colorSensor.green()));
        telemetry.addData("LighDetected", (colorSensor.red()));
        telemetry.addData("LighDetected", (colorSensor.blue()));
        telemetry.addData("LighDetected", colorDetected);
        telemetry.update();
    }

    public boolean ColorVerify() {
        if (colorSensor == null) {
            return false;
        } else {
            return true;
        }
    }

    public int CurrentPosition() {
        return intakeTray.getCurrentPosition();
    }
}
