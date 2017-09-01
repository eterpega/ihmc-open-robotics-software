package us.ihmc.robotDataVisualizer.graphics;

import org.junit.Test;

import static org.junit.Assert.fail;

public class LiveMeshDisplayTest
{
   @Test public void TestLiveMeshDisplayRequiresNonNullProvider() {
      try {
         new LiveMeshDisplay(null);
         fail();
      } catch (NullPointerException npe) {
         // pass
      }
   }

   @Test public void TestLiveMeshDisplayRequiresNonNullPaint() {
      try {
         new LiveMeshDisplay(null, new NullMeshProvider());
         fail();
      } catch (NullPointerException npe) {
         // pass
      }
   }
}
