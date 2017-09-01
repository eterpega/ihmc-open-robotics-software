package us.ihmc.robotDataVisualizer.graphics;

import javafx.scene.shape.MeshView;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public abstract class CachedMeshProvider extends MeshProvider
{
   private List<MeshView> cache = null;

   @Override protected void updateMeshes() {
      List<MeshView> meshes;

      if ((meshes = provideMeshes()) != null)
      {
         setMeshes(meshes);
         cache = meshes;
      } else {
         setMeshes(cache);
      }
   }
}
