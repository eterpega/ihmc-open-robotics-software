package us.ihmc.robotDataVisualizer.graphics;

import javafx.application.Platform;
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
import java.util.concurrent.atomic.AtomicReference;

public class LiveMeshDisplay extends Pane
{
   private AtomicBoolean canUpdate = new AtomicBoolean(false);

   private AtomicReference<List<MeshView>> updateMeshes = new AtomicReference<>();

   public void update(List<MeshView> meshes) {
      updateMeshes.set(meshes);

      canUpdate.set(true);
   }

   public LiveMeshDisplay(Paint backgroundColor) {
      this.setBackground(new Background(new BackgroundFill(backgroundColor, CornerRadii.EMPTY, Insets.EMPTY)));

      Platform.runLater(() -> {

      });
   }

   public LiveMeshDisplay() {
      this(Color.BLACK);
   }
}
