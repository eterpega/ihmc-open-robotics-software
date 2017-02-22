package us.ihmc.modelFileLoaders;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFInertia;
import us.ihmc.modelFileLoaders.urdfLoader.xmlDescription.*;
import us.ihmc.robotics.dataStructures.MutableColor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class ModelFileLoaderConversionsHelper
{

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
   public static URDFInertial getInertialInformationAndPackVisualsAndCollisionsForURDFLink(URDFLink urdfLink, List<URDFVisual> urdfVisualsToPack, List<URDFCollision> urdfCollisionsToPack)
   {
      URDFInertial ret = null;
      for (Object o : urdfLink.getInertialOrVisualOrCollision())
      {
         if (o instanceof URDFInertial)
         {
            ret = (URDFInertial) o;
         }

         if(o instanceof URDFVisual)
         {
            urdfVisualsToPack.add((URDFVisual) o);
         }

         if(o instanceof URDFCollision)
         {
            urdfCollisionsToPack.add((URDFCollision) o);
         }
      }

      return ret;
   }

   public static List<URDFLink> getRootLinksFromURDFLinks(List<URDFLink> urdfLinks)
   {
      ArrayList<URDFLink> ret = new ArrayList<>();

      for (URDFLink urdfLink : urdfLinks)
      {
         //         System.out.println(urdfLink.ge);
      }

      return ret;
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
}
