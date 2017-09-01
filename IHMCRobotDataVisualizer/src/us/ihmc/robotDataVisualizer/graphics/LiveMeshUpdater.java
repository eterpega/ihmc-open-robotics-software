package us.ihmc.robotDataVisualizer.graphics;

import java.util.concurrent.atomic.AtomicBoolean;

public class LiveMeshUpdater extends Thread {
   private LiveMeshDisplay display;

   private MeshProvider provider;

   private AtomicBoolean running = new AtomicBoolean(false);

   public LiveMeshUpdater(LiveMeshDisplay display, MeshProvider provider) {
      if (display == null || provider == null) {
         throw new NullPointerException("Must have non-null display and provider for LiveMeshUpdater");
      }

      this.display = display;

      this.provider = provider;
   }

   @Override public void run() {
      running.set(true);

      while (running.get()) {
         if (provider.hasMeshes()) {
            display.update(provider.getMeshes());
         } else {
            display.update(null);
         }
      }
   }

   public void halt() {
      running.set(false);
   }
}
