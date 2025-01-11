package frc.robot.utilities.controlloop;

import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class PIDConfig implements Sendable{
    
    private static int instances = 0;
    private double p,i,d,iLimit,max,min, errorBand;
    private DoubleSupplier f;
    private boolean continuous;

    public PIDConfig (double p, double i, double d){
        setP(p).setI(i).setD(d).setF(0);
        instances++;
        SendableRegistry.addLW(this, "PIDConfig", instances);
    }

    public PIDConfig setP(double p){
        this.p = p;
        return this;
    }

    public PIDConfig setI(double i){
        this.i = i;
        return this;
    }

    public PIDConfig setD(double d){
        this.d = d;
        return this;
    }

    public PIDConfig setF(double f){
        return setF(() -> f);
    }

    public PIDConfig setF(DoubleSupplier f){
        this.f = f;
        return this;
    }

    public PIDConfig setILimit(double iLimit){
        this.iLimit = iLimit;
        return this;
    }

    public PIDConfig setContinuous(boolean continuous) {
        this.continuous = continuous;
        return this;
    }

    public PIDConfig setContinuous(double min, double max) {
        this.continuous = true;
        this.max = max;
        this.min = min;
        errorBand = (max-min)/2d;
        return this;
    }

    public double getP() {
        return p;
    }

    public double getI() {
        return i;
    }

    public double getD() {
        return d;
    }

    public double getF(){
        return f.getAsDouble();
    }
    
    public double getILimit() {
        return iLimit;
    }

    public double getMax() {
        return max;
    }

    public double getMin() {
        return min;
    }

    public boolean getContinuous(){
        return continuous;
    }

    public double getErrorBand() {
        return errorBand;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // builder.setSmartDashboardType("PIDConfig");
        // builder.addDoubleProperty("p", this::getP, this::setP);
        // builder.addDoubleProperty("i", this::getI, this::setI);
        // builder.addDoubleProperty("d", this::getD, this::setD);
        // builder.addDoubleProperty("iLimit", this::getILimit, this::setILimit);
        // builder.addBooleanProperty("continuous", this::getContinuous, this::setContinuous);
    }



}
