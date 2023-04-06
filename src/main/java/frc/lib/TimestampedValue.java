package frc.lib;

public class TimestampedValue<T> {
    private final T value;
    private final double timestamp;

    public TimestampedValue(T value, double timestamp) {
        this.value = value;
        this.timestamp = timestamp;
    }

    public T getValue() {
        return value;
    }

    public double getTimestamp() {
        return timestamp;
    }
}
