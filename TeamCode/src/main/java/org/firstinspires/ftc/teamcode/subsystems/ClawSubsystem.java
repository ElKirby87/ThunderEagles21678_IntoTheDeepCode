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
    private final double wheelDistance = 3.8 * Math.PI;
    private final int finalTicks = 360;
    private Servo clawServo = null;
    private CRServo clawArmLeftServo = null;
    private CRServo clawArmRightServo = null;
    private DcMotorEx elevatorLeft = null;
    private DcMotorEx elevatorRight = null;
    private int minDistance = 30; //En Cm
    private int maxDistance = 70; //En Cm
    private double elevatorTolerance = 0.5; //En Cm
    private double leftCurrentDistance;
    private double rightCurrentDistance;
    private Telemetry telemetry = null;

    private PIDController controlElevator;
    private double power;

    public void ClawInit(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        clawServo = hardwareMap.get(Servo.class, "claw");
        clawArmLeftServo = hardwareMap.get(CRServo.class, "clawArmLeft");
        clawArmRightServo = hardwareMap.get(CRServo.class, "clawArmRight");
        elevatorLeft = hardwareMap.get(DcMotorEx.class, "leftElevator");
        elevatorRight = hardwareMap.get(DcMotorEx.class, "rightElevator");

        controlElevator = new PController(.01, .0, .0);
        controlElevator.setTolerance(10);

        elevatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        leftCurrentDistance = (elevatorLeft.getCurrentPosition() * wheelDistance) / finalTicks;
        rightCurrentDistance = (elevatorRight.getCurrentPosition() * wheelDistance) / finalTicks;

        setMinDistance();
    }

    public void OpenClaw(double position) {
        clawServo.setPosition(position);
    }

    public void ActivateArm(double armLeftPower, double armRightPower) {
        clawArmLeftServo.setPower(armLeftPower);
        clawArmRightServo.setPower(armRightPower);
    }

    public void ActivateElevator(boolean isOn) {
        leftCurrentDistance = (elevatorLeft.getCurrentPosition() * wheelDistance) / finalTicks;
        rightCurrentDistance = (elevatorRight.getCurrentPosition() * wheelDistance) / finalTicks;

        power = getPIDPower();

        if (isOn) {
            elevatorLeft.setPower(power);
            elevatorRight.setPower(power);
            elevatorLeft.setTargetPosition((int) ((maxDistance * finalTicks) / wheelDistance));
            elevatorRight.setTargetPosition((int) ((maxDistance * finalTicks) / wheelDistance));
        } else {
            elevatorLeft.setPower(-power);
            elevatorRight.setPower(-power);
            elevatorLeft.setTargetPosition((int) ((minDistance * finalTicks) / wheelDistance));
            elevatorRight.setTargetPosition((int) ((minDistance * finalTicks) / wheelDistance));
        }
        telemetry.addData("Position left", leftCurrentDistance);
        telemetry.addData("Position right", rightCurrentDistance);
        telemetry.addData("Position leftencoder", elevatorLeft.getCurrentPosition());
        telemetry.addData("Position rightencoder", elevatorRight.getCurrentPosition());
        telemetry.update();
    }

    public void StopElevator() {
        elevatorLeft.setPower(0);
        elevatorRight.setPower(0);
    }

    public void setMinDistance() {
        power = getPIDPower();

        if ((leftCurrentDistance - elevatorTolerance) <= elevatorTolerance ||
                (leftCurrentDistance - elevatorTolerance) >= elevatorTolerance) {
            elevatorLeft.setPower(-power);
            elevatorRight.setPower(-power);
            elevatorLeft.setTargetPosition((int) ((minDistance * finalTicks) / wheelDistance));
            elevatorRight.setTargetPosition((int) ((minDistance * finalTicks) / wheelDistance));
        } else {
            elevatorLeft.setPower(0);
            elevatorRight.setPower(0);
        }
    }

    public double getCurrentHeight() {
        int currentPosition = (elevatorLeft.getCurrentPosition() + elevatorRight.getCurrentPosition()) / 2;
        return (currentPosition / finalTicks) * wheelDistance;
    }

    public double getPIDPower() {
        int minTargetPosition = (int) (minDistance / wheelDistance * finalTicks);
        controlElevator.setSetPoint(minTargetPosition);
        double currentHeight = getCurrentHeight();
        double output = controlElevator.calculate(currentHeight);

        return Range.clip(output, -1.0, 1.0);
    }
}
