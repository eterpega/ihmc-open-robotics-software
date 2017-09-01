package us.ihmc.robotDataVisualizer.graphics;

import org.junit.Test;

import static org.junit.Assert.fail;

public class LiveMeshDisplayTest
{
   @Test public void TestLiveMeshUpdaterRequiresNonNullDisplay() {
      try {
         new LiveMeshUpdater(null, new NullMeshProvider());
         fail();
      } catch (NullPointerException npe) {
         // pass
      }
   }

   @Test public void TestLiveMeshUpdaterRequiresNonNullProvider() {
      try {
         new LiveMeshUpdater(new LiveMeshDisplay(new NullMeshProvider()), null);
         fail();
      } catch (NullPointerException npe) {
         // pass
      }
   }

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
