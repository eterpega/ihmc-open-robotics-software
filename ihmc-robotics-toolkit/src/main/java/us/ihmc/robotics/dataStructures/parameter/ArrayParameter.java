package us.ihmc.robotics.dataStructures.parameter;

public abstract class ArrayParameter extends Parameter
{
   public ArrayParameter(String path)
   {
      super(path);
   }

   public abstract int length();
}
