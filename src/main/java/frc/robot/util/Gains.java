package frc.robot.util;

public class Gains {
  public final double kP;
  public final double kI;
  public final double kD;
  public final double kS;
  public final double kG;
  public final double kV;
  public final double kA;

  public final double kMMV;
  public final double kMMA;
  public final double kMMJ;
  public final double kMMEV;
  public final double kMMEA;

  private Gains(double kP, double kI, double kD, double kS, double kG, double kV, double kA,double kMMV, double kMMA, double kMMJ, double kMMEV, double kMMEA) {
    if (kP < 0 || kI < 0 || kD < 0 || kS < 0 || kV < 0 || kA < 0 || kG < 0 || kMMV < 0 || kMMA < 0 || kMMJ < 0 || kMMEV < 0 || kMMEA < 0) {
      // throw new IllegalArgumentException("Gains must be non-negative");
    }
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kS = kS;
    this.kG = kG;
    this.kV = kV;
    this.kA = kA;

    this.kMMV = kMMV;
    this.kMMA = kMMA;
    this.kMMJ = kMMJ;
    this.kMMEA = kMMEA;
    this.kMMEV = kMMEV;
  }

  public static Gains getEmpty() {
    return new Gains(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  }

  public static Builder builder() {
    return new Builder();
  }

  public static class Builder {
    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double kS = 0;
    private double kG = 0;
    private double kV = 0;
    private double kA = 0;

    private double kMMV = 0;
    private double kMMA = 0;
    private double kMMJ = 0;
    private double kMMEV = 0;
    private double kMMEA = 0;

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

    public Builder kMMV(double kMMV) {
      this.kMMV = kMMV;
      return this;
    }

    public Builder kMMA(double kMMA) {
      this.kMMA = kMMA;
      return this;
    }

    public Builder kMMJ(double kMMJ) {
      this.kMMJ = kMMJ;
      return this;
    }

    public Builder kMMEV(double kMMEV) {
      this.kMMEV = kMMEV;
      return this;
    }

    public Builder kMMEA(double kMMEA) {
      this.kMMEA = kMMEA;
      return this;
    }


    public Gains build() {
      return new Gains(kP, kI, kD, kS, kG, kV, kA, kMMV, kMMA, kMMJ, kMMEV, kMMEA);
    }
  }
}
