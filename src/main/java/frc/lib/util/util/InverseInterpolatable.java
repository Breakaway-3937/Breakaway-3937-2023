package frc.lib.util.util;

public interface InverseInterpolatable<T> {
    double inverseInterpolate(T upper, T query);
}
