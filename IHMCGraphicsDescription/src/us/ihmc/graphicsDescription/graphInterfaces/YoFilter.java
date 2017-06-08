package us.ihmc.graphicsDescription.graphInterfaces;

import us.ihmc.graphicsDescription.dataBuffer.DataEntry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

public abstract class YoFilter implements DataEntry {
    final DataEntry entry;
    private boolean autoScaleEnabled;
    private boolean minMaxChanged;
    private boolean inverted;
    private double manualMaxScaling;
    private double manualMinScaling;
    private double min, max;

    abstract public double[] applyFilter(double[] data);

    abstract public String getFilterShort();

    abstract public String getFilterName();

    abstract public String getFilterDescription();

    abstract public String getFilterIconPath();

    YoFilter(DataEntry fromEntry) {
        if (fromEntry.getVariable() instanceof DoubleYoVariable) {
            this.entry = fromEntry;
            this.autoScaleEnabled = true;
            this.minMaxChanged = false;
            this.inverted = false;
        } else {
            throw new RuntimeException("cannot create filter for non-double");
        }
    }

    @Override
    final public double[] getData() {
        return this.applyFilter(entry.getData());
    }

    @Override
    final public String getVariableName() {
        return String.format("f/%s: %s", this.getFilterShort(), entry.getVariableName());
    }

    @Override
    final public String getFullVariableNameWithNameSpace() {
        return String.format("f/%s: %s", this.getFilterShort(), entry.getFullVariableNameWithNameSpace());
    }

    @Override
    final public void getVariableNameAndValue(StringBuffer stringBuffer) {
        stringBuffer.append(
                String.format(
                        "f/%s: %s: %.6f",
                        this.getFilterShort(),
                        entry.getVariableName(),
                        entry.getVariable().getValueAsDouble()
                )
        );
    }

    @Override
    final public void getVariableNameAndValueAtIndex(StringBuffer stringBuffer, int index) {
        stringBuffer.append(
                String.format(
                        "f/%s: %s: %.6f",
                        this.getFilterShort(),
                        entry.getVariableName(),
                        this.getData()[index]
                )
        );
    }

    @Override
    final public double getMax() {
        double[] data = this.getData();

        double max;

        if (data.length < 2) {
            max = data[0] > data[1] ? data[0] : data[1];
        } else {
            max = data[0];

            for (int i = 1; i < data.length; ++i) {
                if (data[i] > max) {
                    max = data[i];
                }
            }
        }

        return max;
    }

    @Override
    final public double getMin() {
        double[] data = this.getData();

        double min;

        if (data.length < 2) {
            min = data[0] > data[1] ? data[1] : data[0];
        } else {
            min = data[0];

            for (int i = 1; i < data.length; ++i) {
                if (data[i] < min) {
                    min = data[i];
                }
            }
        }

        return min;
    }

    @Override
    final public boolean isAutoScaleEnabled() {
        return this.autoScaleEnabled;
    }

    @Override
    public double getManualMinScaling() {
        return this.manualMinScaling;
    }

    @Override
    public double getManualMaxScaling() {
        return this.manualMaxScaling;
    }

    @Override
    final public void resetMinMaxChanged() {
        this.minMaxChanged = false;
    }

    @Override
    final public boolean minMaxChanged() {
        return this.minMaxChanged;
    }

    private void reCalcMinMax(int leftIndex, int rightIndex, int leftPlotIndex, int rightPlotIndex) {
        // Left Index is the Set IN Point
        // Right Index is the Set OUT Point

        double[] data = this.getData();

        if (data == null) {
            return;
        }

        minMaxChanged = true;

        double newMin = Double.POSITIVE_INFINITY;
        double newMax = Double.NEGATIVE_INFINITY;

        if (leftIndex < rightIndex) {
            for (int i = leftIndex; i < rightIndex; i++) {
                if (!Double.isNaN(data[i]) && data[i] < newMin)
                    newMin = data[i];
                if (!Double.isNaN(data[i]) && data[i] > newMax)
                    newMax = data[i];
            }
        } else {
            for (int i = leftIndex; i < rightPlotIndex; i++) {
                if (!Double.isNaN(data[i]) && data[i] < newMin)
                    newMin = data[i];
                if (!Double.isNaN(data[i]) && data[i] > newMax)
                    newMax = data[i];
            }

            for (int i = leftPlotIndex; i < rightIndex; i++) {
                if (!Double.isNaN(data[i]) && data[i] < newMin)
                    newMin = data[i];
                if (!Double.isNaN(data[i]) && data[i] > newMax)
                    newMax = data[i];
            }
        }

        if (newMin > newMax) {
            newMin = 0.0;
            newMax = 0.0;
        }

        this.min = newMin;
        this.max = newMax;
    }

    @Override
    public double getMax(int leftIndex, int rightIndex, int leftPlotIndex, int rightPlotIndex) {
        reCalcMinMax(leftIndex, rightIndex, leftPlotIndex, rightPlotIndex);

        return this.max;
    }

    @Override
    public double getMin(int leftIndex, int rightIndex, int leftPlotIndex, int rightPlotIndex) {
        reCalcMinMax(leftIndex, rightIndex, leftPlotIndex, rightPlotIndex);

        return this.min;
    }

    @Override
    public void setManualScaling(double newMinVal, double newMaxVal) {
        this.manualMinScaling = newMinVal;
        this.manualMaxScaling = newMaxVal;
        this.autoScaleEnabled = false;
    }

    @Override
    public void enableAutoScale(boolean b) {
        this.autoScaleEnabled = b;
    }

    @Override
    public YoVariable<?> getVariable() {
        YoVariable real = this.entry.getVariable();
        return new DoubleYoVariable(
                this.getFilterShort() + "-" + real.getName(),
                this.getFilterDescription() + " (original description: " + real.getDescription() + ")",
                new YoVariableRegistry("fakistry"));
    }

    @Override
    public void setInverted(boolean selected) {
        this.inverted = selected;
    }

    @Override
    public boolean getInverted() {
        return this.inverted;
    }
}
