package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem  extends SubsystemBase {
   private PhotonCamera camera;
   private AprilTagFieldLayout aprilTagFieldLayout;
   private PhotonPoseEstimator photonPoseEstimator;
   private PhotonPipelineResult pipelineResult;
   private boolean m_hasCameraEnabled = false;

   /**
    * Empty, no camera - set camera enabled to false, basically return empty for all methods
    */
   public VisionSubsystem()
   {
     m_hasCameraEnabled = false;
   } 

   /**
    * Real constructor, create camera object
    * @param cameraFrontToBackInMeters
    * @param cameraSideToSideInMeters
    * @param cameraHeightInMeters
    * @param cameraName
    */
   public VisionSubsystem(double cameraFrontToBackInMeters, double cameraSideToSideInMeters, double cameraHeightInMeters, Rotation3d cameraRotation, String cameraName) 
   {
      m_hasCameraEnabled = true;

      System.out.println("Vision: About to connect to camera");
      this.camera = new PhotonCamera(cameraName);
      this.camera.setVersionCheckEnabled(false);
      System.out.println("Vision: got Camera: " + this.camera.getName());
      pipelineResult = new PhotonPipelineResult();
      
      try {
         this.aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      } catch (IOException e) {
         System.out.println("Unable to load AprilTagFieldLayout: " + e.toString());
         e.printStackTrace(System.err);
         this.aprilTagFieldLayout = null;
      }
      System.out.println("Vision: got AprilTagFieldLayout: " + this.aprilTagFieldLayout);
      
      //Setup Pipeline - 0 == new pipeline
      //Setup Pipeline - 1 ?= AprilTag pipeline
      camera.setPipelineIndex(1);      

      Transform3d robotToCam = new Transform3d(new Translation3d(cameraFrontToBackInMeters, cameraSideToSideInMeters, cameraHeightInMeters), cameraRotation);
      this.photonPoseEstimator = new PhotonPoseEstimator(this.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.camera, robotToCam); 
      System.out.println("Vision: Setup Pose Estimator: " + this.photonPoseEstimator);
   }

   public Pose2d debugClosestTarget() {
      System.out.println("calling vision debug");
      //quick return if camera is not enabled
      if (m_hasCameraEnabled == false) return null;      

      Pose2d visionEstimatedRobotPose = null;
      PhotonTrackedTarget target = null;
      PhotonPipelineResult result = this.camera.getLatestResult();
      boolean hasTargets = result.hasTargets();
      System.out.println("vision: Got target: " + hasTargets);
      System.out.println("Vision pipeline index: " + camera.getPipelineIndex());
      if (hasTargets) {
         target = result.getBestTarget();
         double yaw = target.getYaw();
         double pitch = target.getPitch();
         double area = target.getArea();
         double skew = target.getSkew();
         //List<TargetCorner> corners = target.getMinAreaRectCorners();
         //System.out.println("Vision: got target data: (yaw/pitch/area/skew) (" + yaw + "/" + pitch + "/" + area + "/" + skew + ")");
         int targetID = target.getFiducialId();
         System.out.println("vision: got aprilTag id: " + targetID);
         double poseAmbiguity = target.getPoseAmbiguity();
         System.out.println("vision: got poseAmbiguity: " + poseAmbiguity);
         Transform3d bestCameraToTarget = target.getBestCameraToTarget();
         System.out.println("bestCameraToTarget X/Y/Rotate: " + bestCameraToTarget.getX() + " / " + bestCameraToTarget.getY() + "/" + bestCameraToTarget.getRotation());
         Optional<Pose3d> aprilPoseOptional = aprilTagFieldLayout.getTagPose(targetID);
         Pose3d appriltagPose3d = (aprilPoseOptional.isPresent() ? aprilPoseOptional.get() : null);
         Pose2d appriltagPose = ((appriltagPose3d != null) ? appriltagPose3d.toPose2d() : null);
         if (appriltagPose != null) {
            System.out.println("Estimated target Yaw: " + target.getYaw());
            System.out.println("AprilTag Position X/Y/Rotate: " + appriltagPose.getX() + " / " + appriltagPose.getY() + "/" + appriltagPose.getRotation());  
            
            Optional<EstimatedRobotPose> estimatedRobotPoseOp = getEstimatedGlobalPose();
            Pose3d estimatedRobotPose = ((estimatedRobotPoseOp != null && estimatedRobotPoseOp.isPresent()) ? estimatedRobotPoseOp.get().estimatedPose : null);
            if (estimatedRobotPose != null) {
               visionEstimatedRobotPose = estimatedRobotPose.toPose2d();
               System.out.println("Estimated Robot Position X/Y/Rotate: " + visionEstimatedRobotPose.getX() + " / " + visionEstimatedRobotPose.getY() + "/" + visionEstimatedRobotPose.getRotation());  
               double distanceToTarget = estimatedRobotPose.getTranslation().getDistance(appriltagPose3d.getTranslation());
               System.out.println("Distance to target: " + distanceToTarget);
            }
         }
      } else {
         System.out.println("Vision: number of targets: 0");
      }
      return visionEstimatedRobotPose;

   }

   public PhotonTrackedTarget getTarget() {
      //quick return if camera is not enabled
      if (m_hasCameraEnabled == false) return null;      

      PhotonTrackedTarget target = null;
      PhotonPipelineResult result = this.camera.getLatestResult();
      boolean hasTargets = result.hasTargets();

      if(hasTargets) {
         target = result.getBestTarget();
      }
      return target;
   }

   public PhotonTrackedTarget getVisibleSpeakerTarget(){
      //quick return if camera is not enabled
      if (m_hasCameraEnabled == false) return null;      

      PhotonTrackedTarget target = null;
      PhotonPipelineResult result = this.camera.getLatestResult();
      boolean hasTargets = result.hasTargets();

      if(hasTargets) {
         for (PhotonTrackedTarget potentialTarget : result.getTargets()){
            if (potentialTarget.getFiducialId() == 7 || potentialTarget.getFiducialId() == 4) {
               target = potentialTarget;
               break;
            }
         }
      }
      return target;
   }

   public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
      //quick return if camera is not enabled
      if (m_hasCameraEnabled == false)
      {
         return null;
      } 

      return photonPoseEstimator.update(pipelineResult);
   }

   public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d previousPose) 
   {
      //quick return if camera is not enabled
      if (m_hasCameraEnabled == false) return null;      

      photonPoseEstimator.setReferencePose(previousPose);
      return photonPoseEstimator.update();
   }

   public Optional<Pose3d> getAprilTagPose(int id) {
      return aprilTagFieldLayout.getTagPose(id);
   }

   @Override
   public void periodic() {
      if (m_hasCameraEnabled)
         pipelineResult = camera.getLatestResult();
   }   
}