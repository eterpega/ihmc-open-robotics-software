package us.ihmc.tuner.tree;

import us.ihmc.robotics.dataStructures.parameter.Parameter;
import us.ihmc.tuner.util.ParameterNameUtil;

/**
 * A tree item that represents a specific index of an array parameter.
 */
public class ArrayElementParameterTreeValue extends ParameterTreeValue
{
   private final Parameter parameter;
   private final int index;

   public ArrayElementParameterTreeValue(Parameter parameter, int index)
   {
      this.parameter = parameter;
      this.index = index;
   }

   @Override
   public String getDraggableParameterPath()
   {
      // make sure to drag the correct index
      return ParameterNameUtil.getDraggableParameterPath(parameter.getPath(), index);
   }

   @Override
   public String toString()
   {
      return Integer.toString(index);
   }
}
