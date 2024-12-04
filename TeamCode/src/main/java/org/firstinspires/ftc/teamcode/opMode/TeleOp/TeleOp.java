package org.firstinspires.ftc.teamcode.opMode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Commands.ClawCommand;
import org.firstinspires.ftc.teamcode.hardware.Constants;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp (name = "Mecanum")
public class TeleOp extends OpMode {
    private final Robot robot = Robot.getInstance();
    private final MecanumDrive m_mecanumDrive = new MecanumDrive();
    private final ClawSubsystem m_clawSubsystem = new ClawSubsystem();

    @Override
    public void init() {
        String opModeType = String.valueOf(Constants.OpModeType.TELEOP);
        super.resetRuntime();
        robot.init(hardwareMap);
        m_mecanumDrive.DriveInit(hardwareMap);
        m_clawSubsystem.ClawInit(hardwareMap);
    }

    @Override
    public void loop() {
        m_mecanumDrive.Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        m_clawSubsystem.
        if (gamepad1.a) {
            new ClawCommand(m_clawSubsystem, true);
        }
        if (gamepad1.b) {
            new ClawCommand(m_clawSubsystem, false);
        }
    }
}
