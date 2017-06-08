package us.ihmc.graphicsDescription.graphInterfaces;

import us.ihmc.graphicsDescription.dataBuffer.DataEntry;

abstract public class PhaseFilter extends YoFilter {
    public PhaseFilter(DataEntry fromEntry) {
        super(fromEntry);
    }
}