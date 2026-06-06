package frc.robot.util;

import edu.wpi.first.networktables.*;

public class NTDouble {
  private static final String table = "Test/";
  private final DoubleTopic topic;
  private final DoublePublisher publisher;
  private final DoubleSubscriber subscriber;
  private double value;

  public NTDouble(String name, double value) {
    topic = NetworkTableInstance.getDefault().getDoubleTopic(table + name);
    publisher = topic.publish();
    publisher.set(value);
    subscriber = topic.subscribe(value);
    this.value = value;
  }

  public double get() {
    return subscriber.get();
  }

  public boolean shouldUpdate() {
    double currentValue = get();
    boolean ret = value != currentValue;
    value = currentValue;
    return ret;
  }
}
