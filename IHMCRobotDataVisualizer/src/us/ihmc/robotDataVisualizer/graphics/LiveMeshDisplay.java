package us.ihmc.robotDataVisualizer.graphics;

import javafx.animation.AnimationTimer;
import javafx.beans.property.Property;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.geometry.Insets;
import javafx.scene.layout.Background;
import javafx.scene.layout.BackgroundFill;
import javafx.scene.layout.CornerRadii;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.paint.Paint;
import javafx.scene.shape.MeshView;
import us.ihmc.robotDataVisualizer.graphics.meshProvider.MeshProvider;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class LiveMeshDisplay extends Pane
{
   private Property<List<MeshView>> updateMeshes = new SimpleObjectProperty<>();

   private Property<Boolean> running = new SimpleBooleanProperty(false);

   private ArrayList<LiveMeshDisplayUpdateListener> updateListeners = new ArrayList<>();

   private MeshProvider provider;

   private LiveMeshAnimator animator;

   public void halt() {
      running.setValue(false);
   }

   public void resume() {
      if (running.getValue()) {
         throw new UnsupportedOperationException("Cannot resume LiveMeshDisplay animating while already running");
      }

      running.setValue(true);
   }

   public void update(List<MeshView> meshes)
   {
      updateMeshes.setValue(meshes);
   }

   public void attachUpdateListener(LiveMeshDisplayUpdateListener listener) {
      updateListeners.add(listener);
   }

   public void detachUpdateListener(LiveMeshDisplayUpdateListener listener) {
      updateListeners.remove(listener);
   }

   public void notifyUpdateListeners() {
      for (LiveMeshDisplayUpdateListener listener : updateListeners) {
         listener.notifyOfLiveMeshDisplayUpdate(this);
      }
   }

   private class LiveMeshAnimator extends AnimationTimer
   {
      @Override public void handle(long l)
      {
         if (running.getValue())
         {
            getChildren().clear();

            if (provider.hasMeshes()) {
               update(provider.getMeshes());
            }

            List<MeshView> updateWith;

            if ((updateWith = updateMeshes.getValue()) != null)
            {
               getChildren().addAll(updateWith);

               notifyUpdateListeners();
            }
         }
      }
   }

   public LiveMeshDisplay(Paint backgroundColor, MeshProvider provider, LiveMeshDisplayUpdateListener... listeners)
   {
      if (backgroundColor == null || provider == null)
      {
         throw new NullPointerException("Background color and provider cannot be null for LiveMeshDisplay");
      }

      this.provider = provider;

      this.setBackground(new Background(new BackgroundFill(backgroundColor, CornerRadii.EMPTY, Insets.EMPTY)));

      updateListeners.addAll(Arrays.asList(listeners));

      running.setValue(true);

      (animator = new LiveMeshAnimator()).start();
   }

   public LiveMeshDisplay(MeshProvider provider, LiveMeshDisplayUpdateListener... listeners)
   {
      this(Color.BLACK, provider, listeners);
   }
}
