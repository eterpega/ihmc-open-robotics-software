package us.ihmc.simulationconstructionset;

import java.util.ArrayList;

import us.ihmc.tools.gui.GraphicsUpdatable;

public class GraphicsUpdatablePlaybackListener implements PlaybackListener
{
   private final ArrayList<GraphicsUpdatable> graphicsUpdatableList;

   public GraphicsUpdatablePlaybackListener(ArrayList<GraphicsUpdatable> graphicsUpdatableList)
   {
      this.graphicsUpdatableList = graphicsUpdatableList;
   }

   @Override
   public void notifyOfIndexChange(int newIndex, double newTime)
   {
      if (graphicsUpdatableList != null)
      {
         for (GraphicsUpdatable graphicsUpdatable : graphicsUpdatableList)
         {
            graphicsUpdatable.update();
         }
      }
   }

   @Override public void notifyOfManualEndChange(int inPoint, int outPoint)
   {
      if (graphicsUpdatableList != null)
      {
         for (GraphicsUpdatable graphicsUpdatable : graphicsUpdatableList)
         {
            graphicsUpdatable.update();
         }
      }
   }

   @Override
   public void play(double realTimeRate)
   {
   }

   @Override
   public void stop()
   {
   }
}
