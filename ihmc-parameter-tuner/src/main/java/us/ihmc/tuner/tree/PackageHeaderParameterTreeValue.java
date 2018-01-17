package us.ihmc.tuner.tree;

/**
 * A tree item that represents a package header, which contains many other parameters.
 */
public class PackageHeaderParameterTreeValue extends ParameterTreeValue
{
   private final String packageName;

   public PackageHeaderParameterTreeValue(String packageName)
   {
      this.packageName = packageName;
   }

   @Override
   public String getDraggableParameterPath()
   {
      // package header not draggable
      return null;
   }

   @Override
   public String toString()
   {
      return packageName;
   }
}
