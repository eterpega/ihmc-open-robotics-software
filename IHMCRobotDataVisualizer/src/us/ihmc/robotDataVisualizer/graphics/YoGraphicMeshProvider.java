package us.ihmc.robotDataVisualizer.graphics;

import javafx.scene.shape.MeshView;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class YoGraphicMeshProvider extends MeshProvider
{
   private AtomicReference<List<MeshView>> asyncMeshes = new AtomicReference<>();

   public void setLater(List<MeshView> meshes) {
      this.asyncMeshes.set(meshes);
   }

   @Override protected List<MeshView> provideMeshes()
   {
      return this.asyncMeshes.getAndSet(null);
   }
}
