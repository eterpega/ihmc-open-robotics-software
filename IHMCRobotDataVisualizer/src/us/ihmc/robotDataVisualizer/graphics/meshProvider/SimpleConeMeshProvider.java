package us.ihmc.robotDataVisualizer.graphics.meshProvider;

import javafx.scene.shape.MeshView;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;

import java.util.ArrayList;
import java.util.List;

public class SimpleConeMeshProvider extends CachedMeshProvider
{
   private boolean first = true;

   @Override protected List<MeshView> provideMeshes()
   {
      List<MeshView> toProvide = null;

      if (first) {
         first = false;

         toProvide = new ArrayList<>();

         toProvide.add(new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.Cone(50.0, 50.0, 100))));
      }

      return toProvide;
   }
}
