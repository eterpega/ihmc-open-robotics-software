package us.ihmc.robotDataVisualizer.graphics.meshProvider;

import javafx.scene.shape.MeshView;

import java.util.ArrayList;
import java.util.List;

public class EmptyMeshProvider extends MeshProvider
{
   @Override protected List<MeshView> provideMeshes()
   {
      return new ArrayList<>();
   }
}
