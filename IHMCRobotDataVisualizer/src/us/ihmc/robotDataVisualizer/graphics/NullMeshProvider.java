package us.ihmc.robotDataVisualizer.graphics;

import javafx.scene.shape.MeshView;

import java.util.List;

public class NullMeshProvider extends MeshProvider
{
   @Override protected List<MeshView> provideMeshes()
   {
      return null;
   }
}
