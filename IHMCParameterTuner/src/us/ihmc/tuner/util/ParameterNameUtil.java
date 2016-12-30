package us.ihmc.tuner.util;

public class ParameterNameUtil
{
   public final static String getPackageNameFromQualifiedName(String qualified)
   {
      return qualified.substring(0, qualified.lastIndexOf("."));
   }

   public final static String getDraggableParameterPath(String parameterName, int index)
   {
      return parameterName + "#" + index;
   }

   public final static String getParameterNameFromDraggablePath(String path)
   {
      return path.split("#")[0];
   }

   public final static int getParameterIndexFromDraggablePath(String path)
   {
      return Integer.parseInt(path.split("#")[1]);
   }
}
