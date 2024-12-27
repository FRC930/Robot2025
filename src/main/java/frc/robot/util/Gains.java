package frc.robot.util;

public class Gains {
  public final double kP;
  public final double kI;
  public final double kD;
  public final double kS;
  public final double kG;
  public final double kV;
  public final double kA;

  private Gains(double kP, double kI, double kD, double kS, double kG, double kV, double kA) {
    if (kP < 0 || kI < 0 || kD < 0 || kS < 0 || kV < 0 || kA < 0 || kG < 0) {
      throw new IllegalArgumentException("Gains must be non-negative");
    }
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kS = kS;
    this.kG = kG;
    this.kV = kV;
    this.kA = kA;
  }

  public static Builder builder() {
    return new Builder();
  }

  public static class Builder {
    private double kP;
    private double kI;
    private double kD;
    private double kS;
    private double kG;
    private double kV;
    private double kA;

    public Builder kP(double kP) {
      this.kP = kP;
      return this;
    }

    public Builder kI(double kI) {
      this.kI = kI;
      return this;
    }

    public Builder kD(double kD) {
      this.kD = kD;
      return this;
    }

    public Builder kS(double kS) {
      this.kS = kS;
      return this;
    }

    public Builder kG(double kG) {
      this.kG = kG;
      return this;
    }

    public Builder kV(double kV) {
      this.kV = kV;
      return this;
    }

    public Builder kA(double kA) {
      this.kA = kA;
      return this;
    }

    public Gains build() {
      return new Gains(kP, kI, kD, kS, kG, kV, kA);
    }
  }
}
