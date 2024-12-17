package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDriveSubsystem {

    // Declaración de los motores
    private DcMotor frontLeft, backLeft, frontRight, backRight;

    // Constructor de la clase
    public MecanumDriveSubsystem(HardwareMap hardwareMap) {
        // Inicialización de los motores
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");

        // Configuración de la dirección de los motores
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }

    // Método para controlar el movimiento del robot
    public void drive(double forwardBack, double strafe, double turn) {
        // Cálculo de la potencia de los motores
        double leftFrontPower = forwardBack + strafe + turn;
        double rightFrontPower = forwardBack - strafe - turn;
        double leftBackPower = forwardBack - strafe + turn;
        double rightBackPower = forwardBack + strafe - turn;

        // Clipping de los valores de potencia
        double max = Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(rightFrontPower),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Establecer la potencia de los motores
        frontLeft.setPower(leftFrontPower);
        backLeft.setPower(leftBackPower);
        frontRight.setPower(rightFrontPower);
        backRight.setPower(rightBackPower);
    }
}
