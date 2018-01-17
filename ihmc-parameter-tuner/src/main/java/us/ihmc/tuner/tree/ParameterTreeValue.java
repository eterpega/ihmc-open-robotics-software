package us.ihmc.tuner.tree;

/**
 * An element in the parameter tree view.
 */
public abstract class ParameterTreeValue
{
   /**
    * @return a correctly formatted parameter reference string (see {@link us.ihmc.tuner.util.ParameterNameUtil#getDraggableParameterPath(String, int)}), or
    * {@code null}.
    */
   public abstract String getDraggableParameterPath();

   /**
    * @return the string representation to be displayed in the tree.
    */
   @Override
   public abstract String toString();
}
