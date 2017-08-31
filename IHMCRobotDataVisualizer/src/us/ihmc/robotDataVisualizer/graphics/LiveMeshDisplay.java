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

public class LiveMeshDisplay extends Pane
{
   private Property<List<MeshView>> updateMeshes = new SimpleObjectProperty<>();

   public void update(List<MeshView> meshes) {
      updateMeshes.setValue(meshes);
   }

   public LiveMeshDisplay(Paint backgroundColor, MeshProvider provider) {
      if (backgroundColor == null || provider == null) {
         throw new NullPointerException("Background color and provider cannot be null for LiveMeshDisplay");
      }

      this.setBackground(new Background(new BackgroundFill(backgroundColor, CornerRadii.EMPTY, Insets.EMPTY)));

      updateMeshes.addListener((prop, old, nw) -> {
         Platform.runLater(() -> {
            getChildren().clear();
            getChildren().addAll(nw);
         });
      });

      new LiveMeshUpdater(this, provider).start();
   }

   public LiveMeshDisplay(MeshProvider provider) {
      this(Color.BLACK, provider);
   }
}
