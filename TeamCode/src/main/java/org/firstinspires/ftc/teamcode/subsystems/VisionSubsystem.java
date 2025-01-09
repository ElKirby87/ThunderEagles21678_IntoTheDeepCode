package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
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
    final double DESIRED_DISTANCE = 20;
    final int TOLERANCE = 2;//in
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.01 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.015  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 1;
    final double MAX_AUTO_STRAFE= 1;
    final double MAX_AUTO_TURN  = 1;


    public void VisionInit(HardwareMap hardwareMap, Telemetry telemetry, MecanumDrive m_mecanumDrive) {
        this.m_mecanumDrive = m_mecanumDrive;
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight3a");
        limelight3A.pipelineSwitch(0);
        limelight3A.start();
        telemetry.setMsTransmissionInterval(11);
    }

    public void VisionActivate(Telemetry telemetry) {
        LLResult result = limelight3A.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                LLResultTypes.FiducialResult fr = fiducialResults.get(0);
                if (fr.getFiducialId() == 14 || fr.getFiducialId() == 12) {
                    double targetX = fr.getTargetPoseRobotSpace().getPosition().x;
                    double targetZ = (fr.getTargetPoseRobotSpace().getPosition().z * 65.6167979002);
                    double targetYaw = fr.getTargetPoseRobotSpace().getOrientation().getYaw(AngleUnit.DEGREES);

                    double rangeError = targetZ;
                    double headingError = targetX;
                    double yawError = targetYaw;

                    telemetry.addData("tz", targetZ);
                    telemetry.update();

                    double drive = 0;
                    if (targetZ > DESIRED_DISTANCE) {
                        drive = -Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    } if (targetZ < DESIRED_DISTANCE) {
                        drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);;
                    } if ((targetZ - DESIRED_DISTANCE) <= TOLERANCE &&
                            (targetZ - DESIRED_DISTANCE) >= 0) {
                        m_mecanumDrive.DriveStop();
                    }
                    double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                    double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                    m_mecanumDrive.Drive(drive, strafe, turn);
                }
            } else {
                m_mecanumDrive.DriveStop();
            }
        }
    }


}
