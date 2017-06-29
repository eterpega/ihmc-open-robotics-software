package us.ihmc.simulationconstructionset;

public interface GraphConfigurationChangeListener
{
   public abstract void notifyOfGraphTypeChange();

   public abstract void notifyOfBaselineChange();

   public abstract void notifyOfScaleChange();

   public abstract void notifyOfDisplayChange();
}
