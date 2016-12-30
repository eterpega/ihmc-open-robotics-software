package us.ihmc.tuner.tree;

import us.ihmc.robotics.dataStructures.parameter.Parameter;
import us.ihmc.tuner.util.ParameterNameUtil;

public class SingleParameterTreeValue extends ParameterTreeValue
{
   private final Parameter parameter;

   public SingleParameterTreeValue(Parameter parameter)
   {
      this.parameter = parameter;
   }

   @Override
   public String getDraggableParameterPath()
   {
      // drag the only element
      return ParameterNameUtil.getDraggableParameterPath(parameter.getPath(), 0);
   }

   @Override
   public String toString()
   {
      return parameter.getShortPath();
   }
}
