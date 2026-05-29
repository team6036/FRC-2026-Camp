package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.*;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

/**
 * Robot-side helper for Dart cameras.
 *
 * <p>Wraps NetworkTables topics under {@code /Dartboard/<cameraName>} and provides helpers for
 * reading/decoding pipeline result payloads published as {@code double[]} arrays.
 */
public class DartCamera {
  public static final String GLOBAL_TABLE_NAME = "/Dartboard";
  public static final NetworkTable GLOBAL_TABLE =
      NetworkTableInstance.getDefault().getTable(GLOBAL_TABLE_NAME);

  public final String name;
  public final NetworkTable table;
  public final StringSubscriber cameraIdSub;
  public final IntegerSubscriber heartbeatSub;
  public final DoubleSubscriber captureFpsSub;

  private final PipelineStream<AprilTagResult> aprilTagPipeline;
  private final PipelineStream<ObjectDetectionResult> objectDetectionPipeline;
  private final PipelineStream<ColorResult> colorPipeline;

  public DartCamera(String name) {
    this.name = name;
    this.table = GLOBAL_TABLE.getSubTable(this.name);
    this.cameraIdSub = table.getStringTopic("camera_id").subscribe("-1");
    this.heartbeatSub = table.getIntegerTopic("heartbeat").subscribe(-1);
    this.captureFpsSub = table.getDoubleTopic("capture_fps").subscribe(-1.0);

    NetworkTable pipelinesTable = table.getSubTable("pipelines");
    this.aprilTagPipeline =
        new PipelineStream<>(pipelinesTable.getSubTable("apriltag3d"), AprilTagResult::fromPayload);
    this.objectDetectionPipeline =
        new PipelineStream<>(
            pipelinesTable.getSubTable("model"), ObjectDetectionResult::fromPayload);
    this.colorPipeline =
        new PipelineStream<>(pipelinesTable.getSubTable("threshold"), ColorResult::fromPayload);
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

  public List<AprilTagResult> readAprilTagResults() {
    return aprilTagPipeline.readResults();
  }

  public List<ObjectDetectionResult> readObjectDetectionResults() {
    return objectDetectionPipeline.readResults();
  }

  public List<ColorResult> readColorResults() {
    return colorPipeline.readResults();
  }

  public static Pose3d readPose(double[] data, int offset) {
    if (data == null || data.length < offset + 6) {
      return new Pose3d();
    }

    return new Pose3d(
        new Translation3d(data[offset], data[offset + 1], data[offset + 2]),
        new Rotation3d(data[offset + 3], data[offset + 4], data[offset + 5]));
  }

  /** NetworkTables accessor for a pipeline under {@code /pipelines/<pipelineName>}. */
  private static final class PipelineStream<R extends BaseResult> {
    private final DoubleArraySubscriber resultsSub;
    private final DoubleSubscriber fpsSub;
    private final Function<double[], R> decoder;

    public PipelineStream(NetworkTable table, Function<double[], R> decoder) {
      this.resultsSub = table.getDoubleArrayTopic("results").subscribe(new double[0]);
      this.fpsSub = table.getDoubleTopic("fps").subscribe(-1.0);
      this.decoder = decoder;
    }

    public double getFps() {
      return fpsSub.get();
    }

    public List<R> readResults() {
      var results = new ArrayList<R>();
      for (var entry : resultsSub.readQueue()) {
        var result = decoder.apply(entry.value);
        if (result != null) {
          results.add(result);
        }
      }
      return results;
    }
  }

  /** Base type for a detected target (id, yaw/pitch relative to camera). */
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

  /** A single AprilTag target, including a 3D tag to camera transform. */
  public static final class AprilTagTarget extends BaseTarget {
    public final Transform3d tagToCamera;

    public AprilTagTarget(int id, double yaw, double pitch, Transform3d tagToCamera) {
      super(id, yaw, pitch);
      this.tagToCamera = tagToCamera;
    }

    /**
     * Decodes one AprilTag target starting at {@code offset}.
     *
     * <p>Payload layout (9 doubles):
     *
     * <pre>
     * [0] id
     * [1] yawRad
     * [2] pitchRad
     * [3] rotXRad (tag->camera)
     * [4] rotYRad
     * [5] rotZRad
     * [6] transX_m (tag->camera)
     * [7] transY_m
     * [8] transZ_m
     * </pre>
     */
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

  /** A single ML/object-detection target, including a confidence score. */
  public static final class ObjectDetectionTarget extends BaseTarget {
    public final float confidence;

    public ObjectDetectionTarget(int id, double yaw, double pitch, float confidence) {
      super(id, yaw, pitch);
      this.confidence = confidence;
    }

    /**
     * Decodes one object-detection target starting at {@code offset}.
     *
     * <p>Payload layout (4 doubles):
     *
     * <pre>
     * [0] classId
     * [1] yawRad
     * [2] pitchRad
     * [3] confidence
     * </pre>
     */
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

  /** A single threshold/color target, including an area metric. */
  public static final class ColorTarget extends BaseTarget {
    public final double area;

    public ColorTarget(int id, double yaw, double pitch, double area) {
      super(id, yaw, pitch);
      this.area = area;
    }

    /**
     * Decodes one color target starting at {@code offset}.
     *
     * <p>Payload layout (4 doubles):
     *
     * <pre>
     * [0] id
     * [1] yawRad
     * [2] pitchRad
     * [3] area
     * </pre>
     */
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

  /** A PnP solve output (pose + error). */
  public static final class PnPResult {
    public final Pose3d pose;
    public final double error;

    public PnPResult(Pose3d pose, double error) {
      this.pose = pose;
      this.error = error;
    }

    /**
     * Decodes one PnP result starting at {@code offset}.
     *
     * <p>Payload layout (7 doubles):
     *
     * <pre>
     * [0] poseX_m
     * [1] poseY_m
     * [2] poseZ_m
     * [3] rotXRad
     * [4] rotYRad
     * [5] rotZRad
     * [6] error
     * </pre>
     */
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

  /** Base type for a decoded pipeline result (one frame): timestamp + targets + raw payload. */
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

  /** Decoded result for the AprilTag pipeline. */
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

    /**
     * Decodes an AprilTag payload into an {@link AprilTagResult}.
     *
     * <p>Expected shape:
     *
     * <ul>
     *   <li>{@code [captureTimestamp, numTargets, ...targets..., (optional) numPnP, ...pnp...]}
     *   <li>Each AprilTag target consumes 9 doubles.
     *   <li>Each PnP result consumes 7 doubles.
     * </ul>
     *
     * <p>If the payload is null or too short to contain even the header, returns {@code null}.
     */
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

  /** Decoded result for the object detection pipeline. */
  public static final class ObjectDetectionResult extends BaseResult {
    public final ObjectDetectionTarget[] targets;

    public ObjectDetectionResult(
        double captureTimestamp, int numTargets, ObjectDetectionTarget[] targets, double[] raw) {
      super(captureTimestamp, numTargets, targets, raw);
      this.targets = targets;
    }

    /**
     * Decodes an object detection payload into an {@link ObjectDetectionResult}.
     *
     * <p>Expected shape:
     *
     * <ul>
     *   <li>{@code [captureTimestamp, numTargets, ...targets...]}
     *   <li>Each target consumes 4 doubles.
     * </ul>
     *
     * <p>If the payload is null/too short to contain even the header, returns {@code null}.
     */
    public static ObjectDetectionResult fromPayload(double[] payload) {
      if (payload == null || payload.length <= 2) {
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

  /** Decoded result for the color pipeline. */
  public static final class ColorResult extends BaseResult {
    public final ColorTarget[] targets;

    public ColorResult(
        double captureTimestamp, int numTargets, ColorTarget[] targets, double[] raw) {
      super(captureTimestamp, numTargets, targets, raw);
      this.targets = targets;
    }

    /**
     * Decodes a color payload into a {@link ColorResult}.
     *
     * <p>Expected shape (high level):
     *
     * <ul>
     *   <li>{@code [captureTimestamp, numTargets, ...targets...]}
     *   <li>Each target consumes 4 doubles.
     * </ul>
     *
     * <p>If the payload is null/too short to contain even the header, returns {@code null}.
     */
    public static ColorResult fromPayload(double[] payload) {
      if (payload == null || payload.length <= 2) {
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
