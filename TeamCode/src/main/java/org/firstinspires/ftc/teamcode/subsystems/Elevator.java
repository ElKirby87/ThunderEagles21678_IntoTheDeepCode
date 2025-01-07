package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Elevator extends SubsystemBase {
    private DcMotor elevatorLeft ;
    private DcMotor elevatorRight ;
    private static final double TICKS_PER_REVOLUTION = 560;
    private static final double PULLEY_CIRCUMFERENCE = 11.81;

    private double kP = 0.1;
    private double KI = 0.01;
    private double kD = 0.005;

    private PIDController pidController;
    private Telemetry telemetry;

    public void ElevatorInit(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        elevatorLeft = hardwareMap.get(DcMotor.class, "leftElevator");
        elevatorRight = hardwareMap.get(DcMotor.class, "rightElevator");
        elevatorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevatorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pidController = new PIDController(kP,KI,kD);
    }

    public void moveToHeight(double heightCm){
        int targetPosition = (int)(heightCm / PULLEY_CIRCUMFERENCE * TICKS_PER_REVOLUTION);
        pidController.setSetPoint(targetPosition);
        double currentHeight = getCurrentHeight();
        double output = pidController.calculate(currentHeight);

        double power = Range.clip(output,-1.0,1.0);
        elevatorLeft.setPower(power);
        elevatorRight.setPower(power);

        telemetry.addData("currentPosition", getCurrentHeight());

    }

    public double getCurrentHeight(){
        int currentPosition = (elevatorLeft.getCurrentPosition()+elevatorRight.getCurrentPosition())/2;
        return (currentPosition / TICKS_PER_REVOLUTION) * PULLEY_CIRCUMFERENCE;
    }
}
