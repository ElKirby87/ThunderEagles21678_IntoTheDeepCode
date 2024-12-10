package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrive extends SubsystemBase {
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;

    public void DriveInit(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void Drive(double drive, double strafe, double twist) {
        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(twist), 1);
        double frontLeftPower = (drive + strafe + twist) / denominator;
        double backLeftPower = (drive - strafe + twist) / denominator;
        double frontRightPower = (drive - strafe - twist) / denominator;
        double backRightPower = (drive + strafe - twist) / denominator;

        leftFront.setPower(frontLeftPower);
        rightFront.setPower(frontRightPower);
        leftBack.setPower(backLeftPower * 0.5);
        rightBack.setPower(backRightPower * 0.5);
    }
}
