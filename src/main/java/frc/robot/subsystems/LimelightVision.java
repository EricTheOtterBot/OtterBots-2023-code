// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.oi.LimeLight;
import frc.robot.oi.LimeLight.CamMode;
import frc.robot.oi.LimeLight.LedMode;
import frc.robot.oi.LimeLight.StreamType;

public class LimelightVision extends SubsystemBase {
  /** Creates a new LimelightVision. */

  private int numCams = 1;

  public LimeLight cam_tag_15;

  public LimeLight cam_tape_16;

  Rotation2d rr = new Rotation2d(1.57);
  Translation3d tl3 = new Translation3d(1, 2, 3);
  Rotation3d rr3 = new Rotation3d(1.57, 1.00, .44);
  Transform3d tran3d = new Transform3d(tl3, rr3);

  public int[][] targetMatrix = new int[8][3];

  private double imageCaptureTime;

  private int fiducialId;

  private Pose3d camPose = new Pose3d();

  private Pose2d camPose2d = new Pose2d();

  private Transform3d robTran = new Transform3d();

  private Pose2d targetPose2d = new Pose2d();

  public Transform3d camTran = new Transform3d();

  private Pose2d visionPoseEstimatedData;

  public static Map<String, Integer> tapePipelines;

  public static Map<String, Integer> tagPipelines;

  private double movingAverageX;
  private double[] movingArrayX;
  private int movingAverageIndex;

  static {
    tagPipelines = new HashMap<>();
    tagPipelines.put("tag_0", 0);
    tagPipelines.put("PL1", 1);
    tagPipelines.put("PL2", 2);
    tagPipelines.put("PL3", 3);
    tagPipelines.put("tape_4", 4);
    tagPipelines.put("PL5", 5);
    tagPipelines.put("PL6", 6);
    tagPipelines.put("PL7", 7);
    tagPipelines.put("PL8", 8);
    tagPipelines.put("PL0", 9);

  }

  static {
    tapePipelines = new HashMap<>();
    tapePipelines.put("PL)", 0);
    tapePipelines.put("PL1", 1);
    tapePipelines.put("PL2", 2);
    tapePipelines.put("PL3", 3);
    tapePipelines.put("tape_4", 4);
    tapePipelines.put("PL5", 5);
    tapePipelines.put("PL6", 6);
    tapePipelines.put("PL7", 7);
    tapePipelines.put("PL8", 8);
    tapePipelines.put("PL9", 9);

  }

  public LimelightVision() {

    cam_tag_15 = new LimeLight("limelight-otto");
    cam_tag_15.setLEDMode(LedMode.kforceOn);
    cam_tag_15.setCamMode(CamMode.kvision);
    cam_tag_15.setStream(StreamType.kStandard);
    cam_tag_15.setPipeline(0);

    movingAverageX = 0.0;
    movingArrayX = new double[5];
    movingAverageIndex = 0;

    if (numCams > 1) {

      // cam_tape_16 = new LimeLight("limelight-tape_16");

      cam_tape_16.setLEDMode(LedMode.kpipeLine);
      cam_tape_16.setCamMode(CamMode.kvision);
      cam_tape_16.setStream(StreamType.kStandard);

      // ShuffleboardLL cam_tape_16Display = new ShuffleboardLL(cam_tape_16);
    }
  }

  public void setPipeline(int pipelineNumber) {
    cam_tag_15.setPipeline(pipelineNumber);
  }

  public Transform3d getCamTransform(LimeLight cam) {

    imageCaptureTime = cam.getPipelineLatency() / 1000d;

    fiducialId = cam.getAprilTagID();

    if (fiducialId != -1) {

      camTran = cam.getCamTran();
    }

    return camTran;

  }

  public Transform3d getRobotTransform(LimeLight cam) {

    imageCaptureTime = cam.getPipelineLatency() / 1000d;

    fiducialId = cam.getAprilTagID();

    if (fiducialId != -1) {
      robTran = cam.getRobotTransform();

    }

    return robTran;

  }

  public double round2dp(double number) {

    number = Math.round(number * 100);
    number /= 100;
    return number;

  }

  public boolean targetExists() {
    return cam_tag_15.getIsTargetFound();
  }

  public double getXValue() {
    return cam_tag_15.getdegRotationToTarget();
  }

  public double calculateAverage() {
    double sumOfValues = 0;

    for(int i = 0; i < 5; i++) {
      sumOfValues += movingArrayX[i];
    }

    double averageOfValues = sumOfValues / 5;
    return averageOfValues;
  }

  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("Limelight tx", cam_tag_15.getdegRotationToTarget());
    SmartDashboard.putNumber("Limelight tx avg", this.calculateAverage());
    
    movingAverageIndex++;
    if(movingAverageIndex > 4) {
      movingAverageIndex = 0;
    }
    movingArrayX[movingAverageIndex] = cam_tag_15.getdegRotationToTarget();

  }

}
