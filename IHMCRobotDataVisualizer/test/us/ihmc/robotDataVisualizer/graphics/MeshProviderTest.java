package us.ihmc.robotDataVisualizer.graphics;

import org.junit.Test;
import static org.junit.Assert.*;

import java.util.ArrayList;
import java.util.List;

import javafx.scene.shape.MeshView;

public class MeshProviderTest
{
   @Test public void TestAsyncMeshProviderProvidesLater() {
      List<MeshView> toProvide = new ArrayList<>();

      AsyncMeshProvider async = new AsyncMeshProvider();

      async.provideLater(toProvide);

      assertTrue(async.hasMeshes());
      assertEquals(toProvide, async.getMeshes());
   }

   @Test public void TestNullMeshProviderProvidesNull() {
      NullMeshProvider provider = new NullMeshProvider();

      assertFalse(provider.hasMeshes());
      assertNull(provider.getMeshes());
      assertNull(provider.provideMeshes());
   }

   @Test public void TestCachedMeshProviderProvidesCache() {
      List<MeshView> toCache = new ArrayList<>();

      CachedMeshProvider provider = new CachedMeshProvider()
      {
         private boolean first = true;

         @Override protected List<MeshView> provideMeshes()
         {
            if (first) {
               first = false;

               return toCache;
            }

            return null;
         }
      };

      assertTrue(provider.hasMeshes());
      assertEquals(toCache, provider.getMeshes());
      assertTrue(provider.hasMeshes());
      assertEquals(toCache, provider.getMeshes());
   }
}
