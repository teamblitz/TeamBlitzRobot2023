package frc.lib;

public class MutableReference<T> {
    private T val = null;

    public MutableReference(T val) {
        set(val);
    }

    public MutableReference() {
        this(null);
    }

    public T get() {
        return val;
    }

    public void set(T val) {
        this.val = val;
    }
}
