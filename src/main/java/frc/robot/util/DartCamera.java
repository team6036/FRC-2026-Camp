package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.*;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** Robot-side helper for Dart cameras. */
public class DartCamera {
  public static final String GLOBAL_TABLE_NAME = "/Dartboard";
  public static final NetworkTable GLOBAL_TABLE =
      NetworkTableInstance.getDefault().getTable(GLOBAL_TABLE_NAME);

  public final String name;
  public final NetworkTable table;
  public final StringSubscriber cameraIdSub;
  public final IntegerSubscriber heartbeatSub;
  public final DoubleSubscriber captureFpsSub;

  private final Map<String, PipelineStream> pipelines = new HashMap<>();

  public DartCamera(String name) {
    this.name = name;
    this.table = GLOBAL_TABLE.getSubTable(this.name);
    this.cameraIdSub = table.getStringTopic("camera_id").subscribe("-1");
    this.heartbeatSub = table.getIntegerTopic("heartbeat").subscribe(-1);
    this.captureFpsSub = table.getDoubleTopic("capture_fps").subscribe(-1.0);
  }

  public PipelineStream getPipeline(String pipelineName) {
    return pipelines.computeIfAbsent(
        pipelineName, p -> new PipelineStream(table.getSubTable("pipelines").getSubTable(p)));
  }

  public PipelineStream aprilTag3d() {
    return getPipeline("apriltag3d");
  }

  public PipelineStream model() {
    return getPipeline("model");
  }

  public PipelineStream colorThreshold() {
    return getPipeline("threshold");
  }

  public String getCameraId() {
    return cameraIdSub.get();
  }

  public long getHeartbeat() {
    return heartbeatSub.get();
  }

  public double getCaptureFps() {
    return captureFpsSub.get();
  }

  public boolean isAlive() {
    return getHeartbeat() >= 0;
  }

  public static Pose3d readPoseRads(double[] data, int offset) {
    if (data == null || data.length < offset + 6) {
      return new Pose3d();
    }

    return new Pose3d(
        new Translation3d(data[offset], data[offset + 1], data[offset + 2]),
        new Rotation3d(data[offset + 3], data[offset + 4], data[offset + 5]));
  }

  public static final class PipelineStream {
    public final NetworkTable table;
    public final DoubleArraySubscriber resultsSub;
    public final DoubleSubscriber fpsSub;

    public PipelineStream(NetworkTable table) {
      this.table = table;
      this.resultsSub = table.getDoubleArrayTopic("results").subscribe(new double[0]);
      this.fpsSub = table.getDoubleTopic("fps").subscribe(-1.0);
    }

    public double getFps() {
      return fpsSub.get();
    }

    public double[] getLatestResults() {
      return resultsSub.get();
    }

    public TimestampedDoubleArray[] readResultsQueue() {
      return resultsSub.readQueue();
    }

    public List<AprilTagResult> readAprilTagResults() {
      java.util.List<AprilTagResult> results = new java.util.ArrayList<>();
      for (TimestampedDoubleArray entry : readResultsQueue()) {
        AprilTagResult result = AprilTagResult.fromPayload(entry.value);
        if (result != null) {
          results.add(result);
        }
      }
      return results;
    }

    public List<ObjectDetectionResult> readObjectDetectionResults() {
      java.util.List<ObjectDetectionResult> results = new java.util.ArrayList<>();
      for (TimestampedDoubleArray entry : readResultsQueue()) {
        ObjectDetectionResult result = ObjectDetectionResult.fromPayload(entry.value);
        if (result != null) {
          results.add(result);
        }
      }
      return results;
    }

    public List<ColorResult> readColorResults() {
      java.util.List<ColorResult> results = new java.util.ArrayList<>();
      for (TimestampedDoubleArray entry : readResultsQueue()) {
        ColorResult result = ColorResult.fromPayload(entry.value);
        if (result != null) {
          results.add(result);
        }
      }
      return results;
    }
  }

  public abstract static class BaseTarget {
    public final int id;
    public final double yaw;
    public final double pitch;

    public BaseTarget(int id, double yaw, double pitch) {
      this.id = id;
      this.yaw = yaw;
      this.pitch = pitch;
    }
  }

  public static final class AprilTagTarget extends BaseTarget {
    public final Transform3d tagToCamera;

    public AprilTagTarget(int id, double yaw, double pitch, Transform3d tagToCamera) {
      super(id, yaw, pitch);
      this.tagToCamera = tagToCamera;
    }

    public static AprilTagTarget fromPayload(double[] payload, int offset) {
      if (payload == null || payload.length < offset + 9) {
        return null;
      }

      int id = (int) payload[offset];
      double yaw = payload[offset + 1];
      double pitch = payload[offset + 2];
      Transform3d tagToCamera =
          new Transform3d(
              new Translation3d(payload[offset + 6], payload[offset + 7], payload[offset + 8]),
              new Rotation3d(payload[offset + 3], payload[offset + 4], payload[offset + 5]));
      return new AprilTagTarget(id, yaw, pitch, tagToCamera);
    }
  }

  public static final class ObjectDetectionTarget extends BaseTarget {
    public final float confidence;

    public ObjectDetectionTarget(int id, double yaw, double pitch, float confidence) {
      super(id, yaw, pitch);
      this.confidence = confidence;
    }

    public static ObjectDetectionTarget fromPayload(double[] payload, int offset) {
      if (payload == null || payload.length < offset + 4) {
        return null;
      }

      int id = (int) payload[offset];
      double yaw = payload[offset + 1];
      double pitch = payload[offset + 2];
      double confidence = payload[offset + 3];
      return new ObjectDetectionTarget(id, yaw, pitch, (float) confidence);
    }
  }

  public static final class ColorTarget extends BaseTarget {
    public final double area;

    public ColorTarget(int id, double yaw, double pitch, double area) {
      super(id, yaw, pitch);
      this.area = area;
    }

    public static ColorTarget fromPayload(double[] payload, int offset) {
      if (payload == null || payload.length < offset + 4) {
        return null;
      }

      int id = (int) payload[offset];
      double yaw = payload[offset + 1];
      double pitch = payload[offset + 2];
      double area = payload[offset + 3];
      return new ColorTarget(id, yaw, pitch, area);
    }
  }

  public static final class PnPResult {
    public final Pose3d pose;
    public final double error;

    public PnPResult(Pose3d pose, double error) {
      this.pose = pose;
      this.error = error;
    }

    public static PnPResult fromPayload(double[] payload, int offset) {
      if (payload == null || payload.length < offset + 7) {
        return null;
      }
      Pose3d pose =
          new Pose3d(
              new Translation3d(payload[offset], payload[offset + 1], payload[offset + 2]),
              new Rotation3d(payload[offset + 3], payload[offset + 4], payload[offset + 5]));
      return new PnPResult(pose, payload[offset + 6]);
    }
  }

  public abstract static class BaseResult {
    public final double captureTimestamp;
    public final int numTargets;
    public final BaseTarget[] targets;
    public final double[] raw;

    public BaseResult(double captureTimestamp, int numTargets, BaseTarget[] targets, double[] raw) {
      this.captureTimestamp = captureTimestamp;
      this.numTargets = numTargets;
      this.targets = targets;
      this.raw = raw;
    }
  }

  public static final class AprilTagResult extends BaseResult {
    public final AprilTagTarget[] targets;
    public final int numPnpResults;
    public final PnPResult[] pnpResults;

    public AprilTagResult(
        double captureTimestamp,
        int numTargets,
        AprilTagTarget[] targets,
        int numPnpResults,
        PnPResult[] pnpResults,
        double[] raw) {
      super(captureTimestamp, numTargets, targets, raw);
      this.targets = targets;
      this.numPnpResults = numPnpResults;
      this.pnpResults = pnpResults;
    }

    public static AprilTagResult fromPayload(double[] payload) {
      if (payload == null || payload.length <= 3) {
        return null;
      }

      double captureTimestamp = payload[0];
      int idx = 1;

      int numTargets = (int) payload[idx++];
      AprilTagTarget[] targets = new AprilTagTarget[Math.max(numTargets, 0)];
      for (int i = 0; i < targets.length; i++) {
        targets[i] = AprilTagTarget.fromPayload(payload, idx);
        idx += 9;
      }

      int numPnpResults = 0;
      PnPResult[] pnpResults = new PnPResult[0];
      if (idx < payload.length) {
        numPnpResults = (int) payload[idx++];
        pnpResults = new PnPResult[Math.max(numPnpResults, 0)];
        for (int i = 0; i < pnpResults.length; i++) {
          pnpResults[i] = PnPResult.fromPayload(payload, idx);
          idx += 7;
        }
      }

      return new AprilTagResult(
          captureTimestamp, numTargets, targets, numPnpResults, pnpResults, payload);
    }
  }

  public static final class ObjectDetectionResult extends BaseResult {
    public final ObjectDetectionTarget[] targets;

    public ObjectDetectionResult(
        double captureTimestamp, int numTargets, ObjectDetectionTarget[] targets, double[] raw) {
      super(captureTimestamp, numTargets, targets, raw);
      this.targets = targets;
    }

    public static ObjectDetectionResult fromPayload(double[] payload) {
      if (payload == null || payload.length <= 3) {
        return null;
      }

      double captureTimestamp = payload[0];
      int idx = 1;

      int numTargets = (int) payload[idx++];
      ObjectDetectionTarget[] targets = new ObjectDetectionTarget[Math.max(numTargets, 0)];
      for (int i = 0; i < targets.length; i++) {
        targets[i] = ObjectDetectionTarget.fromPayload(payload, idx);
        idx += 4;
      }

      return new ObjectDetectionResult(captureTimestamp, numTargets, targets, payload);
    }
  }

  public static final class ColorResult extends BaseResult {
    public final ColorTarget[] targets;

    public ColorResult(
        double captureTimestamp, int numTargets, ColorTarget[] targets, double[] raw) {
      super(captureTimestamp, numTargets, targets, raw);
      this.targets = targets;
    }

    public static ColorResult fromPayload(double[] payload) {
      if (payload == null || payload.length <= 3) {
        return null;
      }

      double captureTimestamp = payload[0];
      int idx = 1;

      int numTargets = (int) payload[idx++];
      ColorTarget[] targets = new ColorTarget[Math.max(numTargets, 0)];
      for (int i = 0; i < targets.length; i++) {
        targets[i] = ColorTarget.fromPayload(payload, idx);
        idx += 4;
      }

      return new ColorResult(captureTimestamp, numTargets, targets, payload);
    }
  }
}
