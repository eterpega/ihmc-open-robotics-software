package us.ihmc.robotics.robotDescription;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.simulationconstructionset.collisionMeshDefinition.BoxCollisionMeshDefinitionData;
import us.ihmc.simulationconstructionset.collisionMeshDefinition.CollisionMeshDefinitionData;
import us.ihmc.simulationconstructionset.collisionMeshDefinition.CollisionMeshDefinitionDataHolder;
import us.ihmc.simulationconstructionset.collisionMeshDefinition.CylinderCollisionMeshDefinitionData;
import us.ihmc.simulationconstructionset.collisionMeshDefinition.SphereCollisionMeshDefinitionData;

public class RobotDescription implements RobotDescriptionNode, GraphicsObjectsHolder
{
   private String name;
   private final ArrayList<JointDescription> rootJoints = new ArrayList<>();

   public RobotDescription(String name)
   {
      this.setName(name);
   }

   public void addRootJoint(JointDescription rootJoint)
   {
      this.rootJoints.add(rootJoint);
   }

   @Override
   public String getName()
   {
      return name;
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public ArrayList<JointDescription> getRootJoints()
   {
      return rootJoints;
   }

   @Override
   public ArrayList<JointDescription> getChildrenJoints()
   {
      return getRootJoints();
   }

   public JointDescription getJointDescription(String name)
   {
      for (JointDescription rootJoint : rootJoints)
      {
         JointDescription jointDescription = getJointDescriptionRecursively(name, rootJoint);
         if (jointDescription != null)
            return jointDescription;
      }

      return null;
   }

   private JointDescription getJointDescriptionRecursively(String name, JointDescription jointDescription)
   {
      if (jointDescription.getName().equals(name))
         return jointDescription;

      ArrayList<JointDescription> childJointDescriptions = jointDescription.getChildrenJoints();
      for (JointDescription childJointDescription : childJointDescriptions)
      {
         JointDescription jointDescriptionRecursively = getJointDescriptionRecursively(name, childJointDescription);
         if (jointDescriptionRecursively != null)
            return jointDescriptionRecursively;
      }
      return null;
   }

   @Override
   public ArrayList<CollisionMeshDescription> getCollisionObjects(String name)
   {
      JointDescription jointDescription = getJointDescription(name);
      if (jointDescription == null)
         return null;

      return jointDescription.getLink().getCollisionMeshes();
   }

   @Override
   public Graphics3DObject getGraphicsObject(String name)
   {
      JointDescription jointDescription = getJointDescription(name);
      if (jointDescription == null)
         return null;

      return jointDescription.getLink().getLinkGraphics();
   }

   public LinkDescription getLinkDescription(String name)
   {
      JointDescription jointDescription = getJointDescription(name);
      if (jointDescription == null)
         return null;

      return jointDescription.getLink();
   }

   @Override
   public void scale(double factor, double massScalePower, List<String> ignoreInertiaScaleJointList)
   {
      JointDescription.scaleChildrenJoint(getChildrenJoints(), factor, massScalePower, ignoreInertiaScaleJointList);
   }

   public void addCollisionMeshDefinitionData(CollisionMeshDefinitionDataHolder collisionMeshDefinitionDataHolder)
   {
      List<CollisionMeshDefinitionData> collisionMeshDefinitionDataList = collisionMeshDefinitionDataHolder.getCollisionMeshDefinitionData();
      int numberOfDefinitionData = collisionMeshDefinitionDataList.size();

      for (int i = 0; i < numberOfDefinitionData; i++)
      {
         // TODO : get rid of the field, type.
         switch (collisionMeshDefinitionDataList.get(i).getCollisionMeshType())
         {
         case SPHERE:
            addSphereCollisionMeshDefinitionData((SphereCollisionMeshDefinitionData) collisionMeshDefinitionDataList.get(i));
            break;
         case CYLINDER:
            addCylinderCollisionMeshDefinitionData((CylinderCollisionMeshDefinitionData) collisionMeshDefinitionDataList.get(i));
            break;
         case BOX:
            addBoxCollisionMeshDefinitionData((BoxCollisionMeshDefinitionData) collisionMeshDefinitionDataList.get(i));
            break;
         default:
            // TODO throw exception.
            break;
         }
      }
   }

   private void addBoxCollisionMeshDefinitionData(BoxCollisionMeshDefinitionData collisionMeshDefinitionData)
   {
      LinkDescription linkDescription = getLinkDescription(collisionMeshDefinitionData.getParentJointName());

      CollisionMeshDescription collisionMesh = new CollisionMeshDescription();
      collisionMesh.identity();
      collisionMesh.transform(collisionMeshDefinitionData.getTransformToParentJoint());
      collisionMesh.addCubeReferencedAtCenter(collisionMeshDefinitionData.getLength(), collisionMeshDefinitionData.getWidth(),
                                              collisionMeshDefinitionData.getHeight());
      collisionMesh.setCollisionGroup(collisionMeshDefinitionData.getCollisionGroup());
      collisionMesh.setCollisionMask(collisionMeshDefinitionData.getCollisionMask());
      linkDescription.addCollisionMesh(collisionMesh);

      LinkGraphicsDescription linkGraphics;
      if (linkDescription.getLinkGraphics() != null)
      {
         linkGraphics = linkDescription.getLinkGraphics();
         linkGraphics.identity();
         linkGraphics.transform(collisionMeshDefinitionData.getTransformToParentJoint());
         linkGraphics.translate(0, 0, -0.5 * collisionMeshDefinitionData.getHeight());
         linkGraphics.addCube(collisionMeshDefinitionData.getLength(), collisionMeshDefinitionData.getWidth(), collisionMeshDefinitionData.getHeight(),
                              collisionMeshDefinitionData.getYoAppearance());
      }
      else
      {
         linkGraphics = new LinkGraphicsDescription();
         linkGraphics.identity();
         linkGraphics.transform(collisionMeshDefinitionData.getTransformToParentJoint());
         linkGraphics.translate(0, 0, -0.5 * collisionMeshDefinitionData.getHeight());
         linkGraphics.addCube(collisionMeshDefinitionData.getLength(), collisionMeshDefinitionData.getWidth(), collisionMeshDefinitionData.getHeight(),
                              collisionMeshDefinitionData.getYoAppearance());
         linkDescription.setLinkGraphics(linkGraphics);
      }
   }

   private void addSphereCollisionMeshDefinitionData(SphereCollisionMeshDefinitionData collisionMeshDefinitionData)
   {
      LinkDescription linkDescription = getLinkDescription(collisionMeshDefinitionData.getParentJointName());

      CollisionMeshDescription collisionMesh = new CollisionMeshDescription();
      collisionMesh.identity();
      collisionMesh.transform(collisionMeshDefinitionData.getTransformToParentJoint());
      collisionMesh.addSphere(collisionMeshDefinitionData.getRadius());
      collisionMesh.setCollisionGroup(collisionMeshDefinitionData.getCollisionGroup());
      collisionMesh.setCollisionMask(collisionMeshDefinitionData.getCollisionMask());
      linkDescription.addCollisionMesh(collisionMesh);

      LinkGraphicsDescription linkGraphics;
      if (linkDescription.getLinkGraphics() != null)
      {
         linkGraphics = linkDescription.getLinkGraphics();
         linkGraphics.identity();
         linkGraphics.transform(collisionMeshDefinitionData.getTransformToParentJoint());
         linkGraphics.addSphere(collisionMeshDefinitionData.getRadius(), collisionMeshDefinitionData.getYoAppearance());
      }
      else
      {
         linkGraphics = new LinkGraphicsDescription();
         linkGraphics.identity();
         linkGraphics.transform(collisionMeshDefinitionData.getTransformToParentJoint());
         linkGraphics.addSphere(collisionMeshDefinitionData.getRadius(), collisionMeshDefinitionData.getYoAppearance());
         linkDescription.setLinkGraphics(linkGraphics);
      }
   }

   private void addCylinderCollisionMeshDefinitionData(CylinderCollisionMeshDefinitionData collisionMeshDefinitionData)
   {
      LinkDescription linkDescription = getLinkDescription(collisionMeshDefinitionData.getParentJointName());

      CollisionMeshDescription collisionMesh = new CollisionMeshDescription();
      collisionMesh.identity();
      collisionMesh.transform(collisionMeshDefinitionData.getTransformToParentJoint());
      collisionMesh.addCylinderReferencedAtBottomMiddle(collisionMeshDefinitionData.getRadius(), collisionMeshDefinitionData.getHeight());
      collisionMesh.setCollisionGroup(collisionMeshDefinitionData.getCollisionGroup());
      collisionMesh.setCollisionMask(collisionMeshDefinitionData.getCollisionMask());
      linkDescription.addCollisionMesh(collisionMesh);

      LinkGraphicsDescription linkGraphics;
      if (linkDescription.getLinkGraphics() != null)
      {
         linkGraphics = linkDescription.getLinkGraphics();
         linkGraphics.identity();
         linkGraphics.transform(collisionMeshDefinitionData.getTransformToParentJoint());
         linkGraphics.addCylinder(collisionMeshDefinitionData.getHeight(), collisionMeshDefinitionData.getRadius(),
                                  collisionMeshDefinitionData.getYoAppearance());
      }
      else
      {
         linkGraphics = new LinkGraphicsDescription();
         linkGraphics.identity();
         linkGraphics.transform(collisionMeshDefinitionData.getTransformToParentJoint());
         linkGraphics.addCylinder(collisionMeshDefinitionData.getHeight(), collisionMeshDefinitionData.getRadius(),
                                  collisionMeshDefinitionData.getYoAppearance());
         linkDescription.setLinkGraphics(linkGraphics);
      }
   }
}
