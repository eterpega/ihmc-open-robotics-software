package us.ihmc.graphicsDescription.graphInterfaces;

import us.ihmc.graphicsDescription.dataBuffer.DataEntry;

public class BackwardFiniteDifferenceFilter extends TimeFilter {

    public BackwardFiniteDifferenceFilter(DataEntry fromEntry) {
        super(fromEntry);
    }

    @Override
    public String getFilterShort() {
        return "fd-b";
    }

    @Override
    public String getFilterName() {
        return "Backward Finite Difference";
    }

    @Override
    public String getFilterDescription() {
        return "Computes the finite difference between each current point and the one before it; f(x)-f(x-1)";
    }

    @Override
    public String getFilterIconPath() {
        return "icons/BackwardFiniteDifference.png";
    }

    @Override
    public double[] applyFilter(double[] data) {
        double[] r = new double[data.length];

        r[0] = 0.0d;

        for (int step = 1; step < data.length; ++step) {
            r[step] = data[step] - data[step - 1];
        }

        return r;
    }
}
