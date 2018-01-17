package us.ihmc.tuner.tree;

import us.ihmc.robotics.dataStructures.parameter.Parameter;

/**
 * A tree item that represents the header of an array parameter.
 */
public class ArrayHeaderParameterTreeValue extends ParameterTreeValue
{
   private final Parameter parameter;

   public ArrayHeaderParameterTreeValue(Parameter parameter)
   {
      this.parameter = parameter;
   }

   @Override
   public String getDraggableParameterPath()
   {
      // header is not draggable
      return null;
   }

   @Override
   public String toString()
   {
      return parameter.getShortPath();
   }
}
