package us.ihmc.graphicsDescription.graphInterfaces;

import us.ihmc.graphicsDescription.dataBuffer.DataEntry;

abstract public class TimeFilter extends YoFilter {
    public TimeFilter(DataEntry fromEntry) {
        super(fromEntry);
    }
}
