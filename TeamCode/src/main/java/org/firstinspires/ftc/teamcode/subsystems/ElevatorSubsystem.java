package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ElevatorSubsystem extends SubsystemBase {
    private DcMotor elevatorLeft = null;
    private DcMotor elevatorRight = null;
    private int minDistance = 10; //En Cm
    private int maxDistance = 64; //En Cm
    private Telemetry telemetry = null;
    final double wheelDistance = 3.761 * Math.PI; //Centimeters
    final int finalTicks = 560;

    private PIDController controlElevator;

    public void ElevatorInit(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        elevatorLeft = hardwareMap.get(DcMotor.class, "leftElevator");
        elevatorRight = hardwareMap.get(DcMotor.class, "rightElevator");

        controlElevator = new PController(.01, .0, .0);
        controlElevator.setTolerance(1);

        elevatorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void ActivateElevator(boolean isOn) {
        if (isOn) {
            setDistance(maxDistance);
        } else {
            setDistance(minDistance);
        }
    }

    public void StopElevator() {
        elevatorLeft.setPower(0);
        elevatorRight.setPower(0);
    }

    public void setDistance(double desiredDistance) {
        double leftCurrentDistance;
        double rightCurrentDistance;
        double elevatorLeftError;
        double elevatorRightError;
        double elevatorLeftCorrection;
        double elevatorRightCorrection;

        leftCurrentDistance = (elevatorLeft.getCurrentPosition() * wheelDistance) / finalTicks;
        rightCurrentDistance = (elevatorRight.getCurrentPosition() * wheelDistance) / finalTicks;

        elevatorLeftError = desiredDistance - leftCurrentDistance;
        elevatorRightError = desiredDistance - rightCurrentDistance;
        elevatorLeftCorrection = getPIDPower(elevatorLeftError);
        elevatorRightCorrection = getPIDPower(elevatorRightError);

        elevatorLeft.setPower(elevatorLeftCorrection);
        elevatorRight.setPower(elevatorRightCorrection);
        elevatorLeft.setTargetPosition((int) ((desiredDistance * finalTicks) / wheelDistance));
        elevatorRight.setTargetPosition((int) ((desiredDistance * finalTicks) / wheelDistance));
    }

    public double getPIDPower(double desiredDistance) {
        int targetPosition = (int) (desiredDistance / wheelDistance * finalTicks);
        double output = controlElevator.calculate(targetPosition);
        return Range.clip(output, -1.0, 1.0);
    }

    public double getCurrentPosition() {
        double position = (double) (elevatorLeft.getCurrentPosition() + elevatorRight.getCurrentPosition()) / 2;;
        return ((position * wheelDistance) / finalTicks);
    }
}
