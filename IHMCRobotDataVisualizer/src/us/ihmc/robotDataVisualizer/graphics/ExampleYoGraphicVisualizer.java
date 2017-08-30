package us.ihmc.robotDataVisualizer.graphics;

import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class ExampleYoGraphicVisualizer extends Application
{
   private LiveMeshDisplay display;

   @Override public void start(Stage stage) throws Exception
   {
      stage.setScene(new Scene(display = new LiveMeshDisplay()));

      stage.show();

      new LiveMeshUpdater(display).start();
   }

   private class LiveMeshUpdater extends Thread {
      LiveMeshDisplay display;

      public LiveMeshUpdater(LiveMeshDisplay display) {
         this.display = display;
      }

      @Override public void run() {
         for (List<MeshView> meshViews : new MeshStream(new SimpleConeMeshProvider()))
         {
            this.display.update(meshViews);
         }
      }
   }

   private class SimpleConeMeshProvider extends MeshProvider {
      @Override public void run() {
         setGenerating(true);

         while (isGenerating())
         {
            ArrayList<MeshView> newMeshes = new ArrayList<>();

            newMeshes.add(new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.Cone(50.0, 50.0, 100))));

            meshes.set(newMeshes);

            // no stop condition
         }

         setGenerating(false);
      }
   }

   private class MeshProvider extends Thread {
      protected AtomicReference<List<MeshView>> meshes = new AtomicReference<>();

      private AtomicBoolean generating = new AtomicBoolean(false);

      boolean hasMeshes() {
         return meshes.get() != null;
      }

      List<MeshView> getMeshes() {
         return meshes.getAndSet(null);
      }

      synchronized boolean isGenerating() {
         return generating.get();
      }

      synchronized protected void setGenerating(boolean b) {
         generating.set(b);
      }
   }

   private class MeshStream implements Iterable<List<MeshView>> {
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

   public static void main(String[] args) {
      launch(args);
   }
}
