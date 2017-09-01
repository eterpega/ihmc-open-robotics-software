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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public class LiveMeshDisplay extends Pane
{
   private Property<List<MeshView>> updateMeshes = new SimpleObjectProperty<>();

   private LiveMeshUpdater updater;

   private ArrayList<LiveMeshDisplayUpdateListener> updateListeners = new ArrayList<>();

   private AtomicBoolean updating = new AtomicBoolean(false);

   public void update(List<MeshView> meshes)
   {
      updateMeshes.setValue(meshes);

      if (meshes == null) {
         Platform.runLater(getChildren()::clear);
      }
   }

   public void close()
   {
      updater.halt();

      Platform.runLater(this.getChildren()::clear);
   }

   public void attachUpdateListener(LiveMeshDisplayUpdateListener listener)
   {
      updateListeners.add(listener);
   }

   public void detachUpdateListener(LiveMeshDisplayUpdateListener listener)
   {
      updateListeners.remove(listener);
   }

   private void notifyUpdateListeners()
   {
      for (LiveMeshDisplayUpdateListener listener : updateListeners)
      {
         listener.notifyOfLiveMeshDisplayUpdate(this);
      }
   }

   public LiveMeshDisplay(Paint backgroundColor, MeshProvider provider, LiveMeshDisplayUpdateListener... listeners)
   {
      if (backgroundColor == null || provider == null)
      {
         throw new NullPointerException("Background color and provider cannot be null for LiveMeshDisplay");
      }

      this.setBackground(new Background(new BackgroundFill(backgroundColor, CornerRadii.EMPTY, Insets.EMPTY)));

      updateListeners.addAll(Arrays.asList(listeners));

      updateMeshes.addListener((prop, old, nw) ->
      {
         if (!updating.get())
         {
            updating.set(true);

            Platform.runLater(() ->
            {
               getChildren().clear();

               if (nw != null)
               {
                  getChildren().addAll(nw);
               }

               notifyUpdateListeners();

               updating.set(false);
            });
         }
      });

      (updater = new LiveMeshUpdater(this, provider)).start();
   }

   public LiveMeshDisplay(MeshProvider provider, LiveMeshDisplayUpdateListener... listeners)
   {
      this(Color.BLACK, provider, listeners);
   }
}
