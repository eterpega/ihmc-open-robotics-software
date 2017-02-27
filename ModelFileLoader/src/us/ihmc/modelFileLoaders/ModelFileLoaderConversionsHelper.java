package us.ihmc.modelFileLoaders;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFInertia;
import us.ihmc.modelFileLoaders.urdfLoader.xmlDescription.*;
import us.ihmc.robotics.dataStructures.MutableColor;
import us.ihmc.tools.io.printing.PrintTools;

import java.util.HashMap;
import java.util.List;

public class ModelFileLoaderConversionsHelper
{

   public static AppearanceDefinition getAppearanceFromURDFMaterial(List<String> resourceDirectories, URDFMaterial material)
   {
      String name = material.getName();
      URDFTexture texture = material.getTexture();
      URDFColor color = material.getColor();
      return getAppearanceDefinition(resourceDirectories, texture, color, name);
   }

   public static AppearanceDefinition getAppearanceFromURDFMaterialGlobal(List<String> resourceDirectories, URDFMaterialGlobal material)
   {
      String name = material.getName();
      URDFTexture texture = material.getTexture();
      URDFColor color = material.getColor();
      return getAppearanceDefinition(resourceDirectories, texture, color, name);
   }

   public static Pair<Vector3D, QuaternionReadOnly> getPoseFromURDFPose(String containingElementName, URDFPose pose)
   {
      String xyzString = pose.getXyz();
      String rpyString = pose.getRpy();
      String[] xyzSplit = xyzString.split(" ");
      String[] rpySplit = rpyString.split(" ");

      Vector3D position = new Vector3D();
      Quaternion orientation = new Quaternion();

      if (xyzSplit.length != 3)
      {
         reportPoseXYZError(containingElementName);
         position.set(0.0, 0.0, 0.0);
      }
      else
      {
         try
         {
            double x = Double.parseDouble(xyzSplit[0]);
            double y = Double.parseDouble(xyzSplit[1]);
            double z = Double.parseDouble(xyzSplit[2]);

            position.set(x, y, z);
         }
         catch (Throwable e)
         {
            reportPoseXYZError(containingElementName);
            position.set(0.0, 0.0, 0.0);
         }
      }

      if (rpySplit.length != 3)
      {
         reportRPYError(containingElementName);
      }
      else
      {
         try
         {
            double roll = Double.parseDouble(rpySplit[0]);
            double pitch = Double.parseDouble(rpySplit[1]);
            double yaw = Double.parseDouble(rpySplit[2]);

            orientation.setYawPitchRoll(yaw, pitch, roll);
         }
         catch (Throwable e)
         {
            reportRPYError(containingElementName);
         }
      }

      return new ImmutablePair<>(position, orientation);
   }

   public static Tuple3DReadOnly getDimensionsFromURDFBoxGeometry(URDFBox boxGeometry)
   {
      Vector3D dimensions = new Vector3D();
      String size = boxGeometry.getSize();
      String[] sizeSplit = size.split(" ");

      if (sizeSplit.length != 3)
      {
         PrintTools.warn(ModelFileLoaderConversionsHelper.class, "Improperly formatted size string for Box geometry: [" + size + "], using unit cube");
         dimensions.set(1.0, 1.0, 1.0);
      }
      else
      {
         try
         {
            double x = Double.parseDouble(sizeSplit[0]);
            double y = Double.parseDouble(sizeSplit[1]);
            double z = Double.parseDouble(sizeSplit[2]);

            dimensions.set(x, y, z);
         }
         catch (Throwable e)
         {
            PrintTools.warn(ModelFileLoaderConversionsHelper.class, "Improperly formatted size string for Box geometry: [" + size + "], using unit cube");
            dimensions.set(1.0, 1.0, 1.0);
         }
      }

      return dimensions;
   }

   public static Pair<Vector3D, QuaternionReadOnly> getPoseFromURDFPose(URDFPose pose)
   {
      return getPoseFromURDFPose(null, pose);
   }

   /**
    * <p>Helper method to accomodate the oddities of the JAXB/XJC generated code for URDFRobot.</p>
    *
    * <p>The implementation of the XSD schema leads to all top-level {@link URDFLink}, {@link URDFJoint}, {@link URDFMaterialGlobal}, and {@link URDFTransmission}
    * to be contained in a single list of objects. This method iterates over that list and separates everything out in to a single pass.</p>
    *
    * @param urdfRobot The {@link URDFRobot} instance to parse.
    * @param urdfJointsToPack The map to insert joint information in to. Maps from joint name to joint object.
    * @param urdfLinksToPack The map to insert link information in to. Maps from link name to link object.
    * @param urdfGlobalMaterialsToPack The map to insert global material definitions in to. Maps from material name to material object.
    * @param urdfTransmissionToPack The map to insert transmission information in to. Maps from transmission name to transmission object.
    */
   public static void getGlobalLinksJointsMaterialsAndTransmissionFromURDFRobot(URDFRobot urdfRobot, HashMap<String, URDFJoint> urdfJointsToPack,
                                                                                HashMap<String, URDFLink> urdfLinksToPack,
                                                                                HashMap<String, URDFMaterialGlobal> urdfGlobalMaterialsToPack,
                                                                                HashMap<String, URDFTransmission> urdfTransmissionToPack)
   {
      for (Object o : urdfRobot.getJointAndLinkAndMaterial())
      {
         if (o instanceof URDFJoint)
         {
            URDFJoint urdfJoint = (URDFJoint) o;

            urdfJointsToPack.put(urdfJoint.getName(), urdfJoint);
         }

         if (o instanceof URDFLink)
         {
            URDFLink urdfLink = (URDFLink) o;

            urdfLinksToPack.put(urdfLink.getName(), urdfLink);
         }

         if (o instanceof URDFMaterialGlobal)
         {
            URDFMaterialGlobal urdfMaterial = (URDFMaterialGlobal) o;

            urdfGlobalMaterialsToPack.put(urdfMaterial.getName(), urdfMaterial);
         }

         if (o instanceof URDFTransmission)
         {
            URDFTransmission urdfTransmission = (URDFTransmission) o;

            urdfTransmissionToPack.put(urdfTransmission.getName(), urdfTransmission);
         }
      }
   }

   /**
    * <p>Similarly to {@link ModelFileLoaderConversionsHelper#getGlobalLinksJointsMaterialsAndTransmissionFromURDFRobot(URDFRobot, HashMap, HashMap, HashMap, HashMap)},
    * this method eases parsing a compount list of Objects that can be one of several types. This method returns the singular instance of {@link URDFInertial} while packing
    * the {@link URDFVisual}s and {@link URDFCollision}s in to the supplied out-args.
    *
    * @param urdfLink The link to process
    * @param urdfVisualsToPack The collection of visual definitions to pack
    * @param urdfCollisionsToPack The colleciton of collision defintions to pack
    * @return The Intertial information for the joint, or null if it is not present
    */
   public static URDFInertial getInertialInformationAndPackVisualsAndCollisionsForURDFLink(URDFLink urdfLink, List<URDFVisual> urdfVisualsToPack,
                                                                                           List<URDFCollision> urdfCollisionsToPack)
   {
      URDFInertial ret = null;
      for (Object o : urdfLink.getInertialOrVisualOrCollision())
      {
         if (o instanceof URDFInertial)
         {
            ret = (URDFInertial) o;
         }

         if (o instanceof URDFVisual)
         {
            urdfVisualsToPack.add((URDFVisual) o);
         }

         if (o instanceof URDFCollision)
         {
            urdfCollisionsToPack.add((URDFCollision) o);
         }
      }

      return ret;
   }

   private static AppearanceDefinition getAppearanceDefinition(List<String> resourceDirectories, URDFTexture texture, URDFColor color, String materialName)
   {
      if (texture != null)
      {
         //TODO look up the actual buffered image using the resource directores
         return new YoAppearanceTexture(texture.getFilename());
      }

      if (color != null)
      {
         String rgbaString = color.getRgba();
         String[] rgbaSplit = rgbaString.split(" ");
         if(rgbaSplit.length != 4)
         {
            reportRGBAColorError(materialName);
         }
         else
         {
            try
            {
               {
                  double r = Double.parseDouble(rgbaSplit[0]);
                  double g = Double.parseDouble(rgbaSplit[1]);
                  double b = Double.parseDouble(rgbaSplit[2]);
                  double a = Double.parseDouble(rgbaSplit[3]);

                  return new YoAppearanceRGBColor(r, g, b, a);
               }
            }
            catch(Throwable e)
            {
               reportRGBAColorError(materialName);
               return YoAppearance.Gold();
            }
         }
         return YoAppearance.RGBColor(0.0, 0.0, 0.0, 0.0);
      }

      return YoAppearance.Gold();
   }

   public static String sanitizeJointName(String dirtyName)
   {
      return dirtyName.trim().replaceAll("[//[//]///]", "").replace(".", "_");
   }

   public static Vector3D stringToNormalizedVector3d(String vector)
   {
      Vector3D vector3d = stringToVector3d(vector);
      vector3d.normalize();
      return vector3d;

   }

   public static Vector3D stringToVector3d(String vector)
   {
      String[] vecString = vector.split("\\s+");
      Vector3D vector3d = new Vector3D(Double.parseDouble(vecString[0]), Double.parseDouble(vecString[1]), Double.parseDouble(vecString[2]));
      return vector3d;
   }

   public static MutableColor stringToColor(String color)
   {
      String[] vecString = color.split("\\s+");
      MutableColor color3f = new MutableColor(Float.parseFloat(vecString[0]), Float.parseFloat(vecString[1]), Float.parseFloat(vecString[2]));
      return color3f;
   }

   public static Vector2D stringToVector2d(String xy)
   {
      String[] vecString = xy.split("\\s+");

      Vector2D vector = new Vector2D(Double.parseDouble(vecString[0]), Double.parseDouble(vecString[1]));
      return vector;

   }

   public static RigidBodyTransform poseToTransform(String pose)
   {
      RigidBodyTransform ret = new RigidBodyTransform();
      if (pose == null)
      {
         return ret;
      }
      pose = pose.trim();
      String[] data = pose.split("\\s+");

      RigidBodyTransform translation = new RigidBodyTransform();
      Vector3D translationVector = new Vector3D();
      translationVector.setX(Double.parseDouble(data[0]));
      translationVector.setY(Double.parseDouble(data[1]));
      translationVector.setZ(Double.parseDouble(data[2]));
      translation.setTranslationAndIdentityRotation(translationVector);

      RigidBodyTransform rotation = new RigidBodyTransform();
      Vector3D eulerAngels = new Vector3D();
      eulerAngels.setX(Double.parseDouble(data[3]));
      eulerAngels.setY(Double.parseDouble(data[4]));
      eulerAngels.setZ(Double.parseDouble(data[5]));
      rotation.setRotationEulerAndZeroTranslation(eulerAngels);

      ret.set(translation);
      ret.multiply(rotation);

      return ret;
   }

   public static Matrix3D sdfInertiaToMatrix3d(SDFInertia sdfInertia)
   {

      Matrix3D inertia = new Matrix3D();
      if (sdfInertia != null)
      {
         double ixx = Double.parseDouble(sdfInertia.getIxx());
         double ixy = Double.parseDouble(sdfInertia.getIxy());
         double ixz = Double.parseDouble(sdfInertia.getIxz());
         double iyy = Double.parseDouble(sdfInertia.getIyy());
         double iyz = Double.parseDouble(sdfInertia.getIyz());
         double izz = Double.parseDouble(sdfInertia.getIzz());
         inertia.setM00(ixx);
         inertia.setM01(ixy);
         inertia.setM02(ixz);
         inertia.setM10(ixy);
         inertia.setM11(iyy);
         inertia.setM12(iyz);
         inertia.setM20(ixz);
         inertia.setM21(iyz);
         inertia.setM22(izz);
      }
      return inertia;
   }

   private static void reportRGBAColorError(String materialName)
   {
      StringBuilder errorString = new StringBuilder();

      errorString.append("Improperly formatted RGBA component for color material, using YoAppearance.Gold().");
      if(materialName != null)
      {
         errorString.append(" URDFMaterial Name: " ).append(materialName);
      }

      PrintTools.warn(ModelFileLoaderConversionsHelper.class, errorString.toString());
   }

   private static void reportRPYError(String containingElementName)
   {
      StringBuilder errorString = new StringBuilder();

      errorString.append("Improperly formatted RPY component for pose, using identity.");
      if (containingElementName != null)
      {
         errorString.append(" Containing Element: ").append(containingElementName);
      }

      PrintTools.warn(ModelFileLoaderConversionsHelper.class, errorString.toString());
   }

   private static void reportPoseXYZError(String containingElementName)
   {
      StringBuilder errorString = new StringBuilder();

      errorString.append("Improperly formatted XYZ component for pose, using (0, 0, 0).");
      if (containingElementName != null)
      {
         errorString.append(" Containing Element: ").append(containingElementName);
      }

      PrintTools.warn(ModelFileLoaderConversionsHelper.class, errorString.toString());
   }
}
