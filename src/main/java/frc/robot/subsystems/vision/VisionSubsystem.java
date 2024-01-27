package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.io.IOException;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class VisionSubsystem {
   private PhotonCamera camera;
   private AprilTagFieldLayout aprilTagFieldLayout;
   private PhotonPoseEstimator photonPoseEstimator;

   public VisionSubsystem(double cameraFrontToBackInMeters, double cameraSideToSideInMeters, double cameraHeightInMeters) {
      System.out.println("Vision: About to connect to camera");
      this.camera = new PhotonCamera("Camera_Module_v1");
      System.out.println("Vision: got Camera: " + this.camera.getName());

      try {
         this.aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      } catch (IOException e) {
         System.out.println("Unable to load AprilTagFieldLayout: " + e.toString());
         e.printStackTrace(System.err);
         this.aprilTagFieldLayout = null;
      }

      System.out.println("Vision: got AprilTagFieldLayout: " + this.aprilTagFieldLayout);
      Transform3d robotToCam = new Transform3d(new Translation3d(cameraFrontToBackInMeters, cameraSideToSideInMeters, cameraHeightInMeters), new Rotation3d(0.0D, 0.0D, 0.0D));
      this.photonPoseEstimator = new PhotonPoseEstimator(this.aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, this.camera, robotToCam);
      System.out.println("Vision: Setup Pose Estimator: " + this.photonPoseEstimator);
   }

   public void getClosestTarget() {
      PhotonTrackedTarget visionTarget = null;
      PhotonPipelineResult result = this.camera.getLatestResult();
      boolean hasTargets = result.hasTargets();
      System.out.println("vision: Got target: " + hasTargets);
      if (hasTargets) {
         PhotonTrackedTarget target = result.getBestTarget();
         double yaw = target.getYaw();
         double pitch = target.getPitch();
         double area = target.getArea();
         double skew = target.getSkew();
         List<TargetCorner> corners = target.getMinAreaRectCorners();
         System.out.println("Vision: got target data: (yaw/pitch/area/skew) (" + yaw + "/" + pitch + "/" + area + "/" + skew + ")");
         int targetID = target.getFiducialId();
         double poseAmbiguity = target.getPoseAmbiguity();
         System.out.println("vision: got aprilTag id: " + targetID);
         Transform3d bestCameraToTarget = target.getBestCameraToTarget();
         Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
      } else {
         System.out.println("Vision: number of targets: 0");
      }

   }
}