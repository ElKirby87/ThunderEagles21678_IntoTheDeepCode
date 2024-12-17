package org.firstinspires.ftc.teamcode.opMode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.hardware.Constants;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveFieldCentric;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp (name = "TeleOpSingle")
public class TeleOp extends OpMode {
    private final Robot robot = Robot.getInstance();
    private final MecanumDrive m_mecanumDrive = new MecanumDrive();
    MecanumDriveSubsystem mecanumDriveSubsystem ;
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
        mecanumDriveSubsystem = new MecanumDriveSubsystem(hardwareMap);
    }

    @Override
    public void loop() {
       // m_mecanumDrive.Drive(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
        double forwardBack = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        m_mecanumDrive.Drive(forwardBack, strafe, turn);
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
        if (gamepad1.left_bumper) {
            m_intakeSubsystem.OpenHand(0.25);
        }
        if (!gamepad1.left_bumper) {
            m_intakeSubsystem.OpenHand(0.10);
        }
        if (gamepad1.dpad_left) {
            m_intakeSubsystem.ActivateTray(1);
        }
        if (gamepad1.dpad_right) {
            m_intakeSubsystem.ActivateTray(-1);
        }
        if (!gamepad1.dpad_left && !gamepad1.dpad_right) {
            m_intakeSubsystem.ActivateTray(0);
        }
        if (gamepad1.dpad_up) {
            m_clawSubsystem.ActivateElevator(-1, 1);
        }
        if (gamepad1.dpad_down) {
            m_clawSubsystem.ActivateElevator(1, -1);
        }
        if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
            m_clawSubsystem.ActivateElevator(0, 0);
        }
        if (gamepad1.left_trigger == 0) {
            m_intakeSubsystem.ActivateHandDoll(-gamepad1.right_trigger);
        }
        if (gamepad1.right_trigger == 0) {
            m_intakeSubsystem.ActivateHandDoll(gamepad1.left_trigger);
        }
    }

}
