package us.ihmc.robotDataVisualizer.graphics.meshProvider;

import javafx.scene.shape.MeshView;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

abstract public class MeshProvider
{
   private AtomicReference<List<MeshView>> meshes = new AtomicReference<>();

   final public boolean hasMeshes()
   {
      if (meshes.get() == null)
      {
         updateMeshes();
      }

      return meshes.get() != null;
   }

   final public List<MeshView> getMeshes()
   {
      return meshes.getAndSet(null);
   }

   final void setMeshes(List<MeshView> meshes)
   {
      this.meshes.set(meshes);
   }

   protected void updateMeshes()
   {
      List<MeshView> meshes;

      if ((meshes = provideMeshes()) != null)
      {
         setMeshes(meshes);
      }
   }

   protected abstract List<MeshView> provideMeshes();
}
