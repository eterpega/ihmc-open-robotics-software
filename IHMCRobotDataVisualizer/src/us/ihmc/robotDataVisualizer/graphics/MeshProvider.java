package us.ihmc.robotDataVisualizer.graphics;

import javafx.scene.shape.MeshView;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

abstract public class MeshProvider {
   private AtomicReference<List<MeshView>> meshes = new AtomicReference<>();

   final boolean hasMeshes() {
      updateMeshes();

      return meshes.get() != null;
   }

   final void setMeshes(List<MeshView> meshes) {
      this.meshes.set(meshes);
   }

   final List<MeshView> getMeshes() {
      return meshes.getAndSet(null);
   }

   public void updateMeshes() {
      List<MeshView> meshes;

      if ((meshes = provideMeshes()) != null) {
         setMeshes(meshes);
      }
   }

   protected abstract List<MeshView> provideMeshes();
}
