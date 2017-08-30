package us.ihmc.robotDataVisualizer.graphics;

import javafx.scene.shape.MeshView;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

abstract public class MeshProvider extends Thread {
   private AtomicReference<List<MeshView>> meshes = new AtomicReference<>();

   private AtomicBoolean generating = new AtomicBoolean(false);

   final boolean hasMeshes() {
      return meshes.get() != null;
   }

   final void setMeshes(List<MeshView> meshes) {
      this.meshes.set(meshes);
   }

   final List<MeshView> getMeshes() {
      return meshes.getAndSet(null);
   }

   final synchronized boolean isGenerating() {
      return generating.get();
   }

   final synchronized void setGenerating(boolean b) {
      generating.set(b);
   }

   @Override final public void run() {
      do {
         setGenerating(true);
      } while(provideMeshes());

      setGenerating(false);
   }

   abstract boolean provideMeshes();
}
