package us.ihmc.urdfLoader;

import us.ihmc.modelFileLoaders.urdfLoader.xmlDescription.URDFJoint;
import us.ihmc.modelFileLoaders.urdfLoader.xmlDescription.URDFLink;
import us.ihmc.modelFileLoaders.urdfLoader.xmlDescription.URDFRobot;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Unmarshaller;
import java.util.List;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class URDFLoadingTest
{
   public static void main(String[] args) throws JAXBException
   {
      JAXBContext jaxbContext = JAXBContext.newInstance("us.ihmc.urdfLoader.xmlDescription");
      Unmarshaller unmarshaller = jaxbContext.createUnmarshaller();

      URDFRobot robot = (URDFRobot) unmarshaller.unmarshal(URDFLoadingTest.class.getClassLoader().getResource("urdfRobotTest.urdf"));

      System.out.println("Robot name: " + robot.getName());

      List<Object> jointAndLinkAndMaterial = robot.getJointAndLinkAndMaterial();

      for (Object obj : jointAndLinkAndMaterial)
      {
         if(obj instanceof URDFLink)
         {
            URDFLink link = (URDFLink) obj;

            System.out.println("Link: " + link.getName());
         }

         if(obj instanceof URDFJoint)
         {
            URDFJoint joint = (URDFJoint) obj;

            System.out.println("Joint: " + joint.getName());
            System.out.println("Parent link: " + joint.getParent().getLink());
         }
      }
   }
}
