package us.ihmc.robotDataVisualizer.graphics;

import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;

import java.util.ArrayList;

public class ExampleYoGraphicVisualizer extends Application
{
   private LiveMeshDisplay display;

   @Override public void start(Stage stage) throws Exception
   {
      stage.setScene(new Scene(display = new LiveMeshDisplay()));

      stage.show();

      new LiveMeshUpdater(display, new SimpleConeMeshProvider()).start();
   }

   private class SimpleConeMeshProvider extends MeshProvider {
      @Override boolean provideMeshes()
      {
         ArrayList<MeshView> newMeshes = new ArrayList<>();

         newMeshes.add(new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.Cone(50.0, 50.0, 100))));

         setMeshes(newMeshes);

         return false;
      }
   }

   public static void main(String[] args) {
      launch(args);
   }
}
