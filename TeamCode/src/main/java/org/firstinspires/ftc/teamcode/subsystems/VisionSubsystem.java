package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Collection;
import java.util.Collections;
import java.util.List;

public class VisionSubsystem extends SubsystemBase {
    private Limelight3A limelight3A;
    private MecanumDrive m_mecanumDrive = new MecanumDrive();
    private IMU imu;
    final double DESIRED_DISTANCE = 12.0;
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;


    public void VisionInit(HardwareMap hardwareMap, Telemetry telemetry, MecanumDrive m_mecanumDrive) {
        this.m_mecanumDrive = m_mecanumDrive;
        imu = hardwareMap.get(IMU.class, "imu");
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight3a");
        limelight3A.pipelineSwitch(0);
        limelight3A.start();
        telemetry.setMsTransmissionInterval(11);
    }

    public void VisionActivate(Telemetry telemetry) {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        LLResult result = limelight3A.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialR = result.getFiducialResults();
        if (result != null) {
            if (result.isValid()) {
                double targetX = result.getTx();
                double targetY = result.getTy();
                //double targetYaw = result.

                double rangeError = (targetY - DESIRED_DISTANCE);
                double headingError = targetX;
                //double yawError = targetYaw;

                double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                //double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                //m_mecanumDrive.Drive(drive, strafe, turn);
            }
            else {
                telemetry.addData("Unknown", "No tag find");
            }
        } else {
            m_mecanumDrive.Drive(0, 0, 0);
        }
    }
}
