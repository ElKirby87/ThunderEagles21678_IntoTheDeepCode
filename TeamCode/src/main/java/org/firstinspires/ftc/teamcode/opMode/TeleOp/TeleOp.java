package org.firstinspires.ftc.teamcode.opMode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Constants;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp (name = "Teleoperated")
public class TeleOp extends OpMode {
    private final Robot robot = Robot.getInstance();
    private final MecanumDrive m_mecanumDrive = new MecanumDrive();
    private final ClawSubsystem m_clawSubsystem = new ClawSubsystem();
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

    @Override
    public void init() {
        String opModeType = String.valueOf(Constants.OpModeType.TELEOP);
        super.resetRuntime();
        robot.init(hardwareMap);
        m_mecanumDrive.DriveInit(hardwareMap);
        m_clawSubsystem.ClawInit(hardwareMap);
        m_intakeSubsystem.IntakeInit(hardwareMap);
    }

    @Override
    public void loop() {
        m_mecanumDrive.Drive(gamepad1.left_stick_y, gamepad1.left_stick_x * 1.1, -gamepad1.right_stick_x);
        if (gamepad1.right_bumper) {
            m_clawSubsystem.OpenClaw(1);
        }
        if (!gamepad1.right_bumper) {
            m_clawSubsystem.OpenClaw(0);
        }
        if (gamepad1.y) {
            m_clawSubsystem.ActivateArm(1, -1);
        }
        if (gamepad1.x) {
            m_clawSubsystem.ActivateArm(-1, 1);
        }
        if (!gamepad1.y && !gamepad1.x) {
            m_clawSubsystem.ActivateArm(0, 0);
        }
        if (gamepad1.a) {
            m_intakeSubsystem.ActivateArm(1, -1);
        }
        if (gamepad1.b) {
            m_intakeSubsystem.ActivateArm(-1, 1);
        }
        if (!gamepad1.a && !gamepad1.b) {
            m_intakeSubsystem.ActivateArm(0, 0);
        }
    }
}
