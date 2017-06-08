package us.ihmc.graphicsDescription.graphInterfaces;

import us.ihmc.graphicsDescription.dataBuffer.DataEntry;

public class ForwardFiniteDifferenceFilter extends TimeFilter {

    public ForwardFiniteDifferenceFilter(DataEntry fromEntry) {
        super(fromEntry);
    }

    @Override
    public String getFilterShort() {
        return "fd-f";
    }

    @Override
    public String getFilterName() {
        return "Forward Finite Difference";
    }

    @Override
    public String getFilterDescription() {
        return "Computes the finite difference between each next point and the current; f(x+1)-f(x)";
    }

    @Override
    public String getFilterIconPath() {
        return "icons/ForwardFiniteDifference.png";
    }

    @Override
    public double[] applyFilter(double[] data) {
        double[] r = new double[data.length];

        r[data.length - 1] = 0;

        for (int step = 0; step < data.length - 1; ++step) {
            r[step] = data[step + 1] - data[step];
        }

        return r;
    }
}
