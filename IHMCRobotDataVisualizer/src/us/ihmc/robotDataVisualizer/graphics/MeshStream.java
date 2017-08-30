package us.ihmc.robotDataVisualizer.graphics;

import javafx.scene.shape.MeshView;

import java.util.Iterator;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class MeshStream implements Iterable<List<MeshView>> {
   private AtomicReference<List<MeshView>> current = new AtomicReference<>();

   private MeshProvider provider;

   MeshStream(MeshProvider provider) {
      this.provider = provider;

      new Thread(this.provider).start();
   }

   synchronized private List<MeshView> next() {
      if (current.get() != null)
      {
         return current.getAndSet(provider.getMeshes());
      } else if (provider.hasMeshes()) {
         current.set(provider.getMeshes());

         return current.get();
      }

      throw new NullPointerException("Provider failed to provide meshes");
   }

   synchronized private boolean checkProvider() {
      boolean canProvide = current.get() != null || provider.hasMeshes();

      while (!canProvide && provider.isGenerating()) {
         canProvide = provider.hasMeshes();
      }

      return canProvide;
   }

   @Override public Iterator<List<MeshView>> iterator()
   {
      return new Iterator<List<MeshView>>()
      {
         @Override public boolean hasNext()
         {
            return MeshStream.this.checkProvider();
         }

         @Override public List<MeshView> next()
         {
            List<MeshView> opt = MeshStream.this.next();

            if (opt != null) {
               return opt;
            }

            throw new NullPointerException("Did not check hasNext first");
         }
      };
   }
}