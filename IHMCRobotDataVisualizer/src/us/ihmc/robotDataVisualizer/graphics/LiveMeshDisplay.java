package us.ihmc.robotDataVisualizer.graphics;

import javafx.application.Platform;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.geometry.Insets;
import javafx.scene.layout.Background;
import javafx.scene.layout.BackgroundFill;
import javafx.scene.layout.CornerRadii;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.paint.Paint;
import javafx.scene.shape.MeshView;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public class LiveMeshDisplay extends Pane
{

   private Property<List<MeshView>> updateMeshes = new SimpleObjectProperty<>();

   private AtomicBoolean updating = new AtomicBoolean(false);

   public void update(List<MeshView> meshes) {
      updateMeshes.setValue(meshes);
   }

   public LiveMeshDisplay(Paint backgroundColor) {
      this.setBackground(new Background(new BackgroundFill(backgroundColor, CornerRadii.EMPTY, Insets.EMPTY)));

      updateMeshes.addListener((prop, old, nw) -> {
         if (!updating.get())
         {
            updating.set(true);

            Platform.runLater(() ->
            {
               getChildren().clear();

               getChildren().addAll(nw);

               updating.set(false);
            });
         }
      });
   }

   public LiveMeshDisplay() {
      this(Color.BLACK);
   }
}
