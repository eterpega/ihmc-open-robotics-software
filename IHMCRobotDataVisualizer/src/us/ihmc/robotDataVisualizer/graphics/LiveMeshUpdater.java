package us.ihmc.robotDataVisualizer.graphics;

import javafx.scene.shape.MeshView;

import java.util.List;

public class LiveMeshUpdater extends Thread {
   private LiveMeshDisplay display;

   private MeshProvider provider;

   public LiveMeshUpdater(LiveMeshDisplay display, MeshProvider provider) {
      if (display == null || provider == null) {
         throw new NullPointerException("Must have non-null display and provider for LiveMeshUpdater");
      }

      this.display = display;

      this.provider = provider;
   }

   @Override public void run() {
      MeshStream stream = new MeshStream(this.provider);

      for (List<MeshView> meshViews : stream)
      {
         this.display.update(meshViews);
      }

      this.display.update(null);
   }
}
