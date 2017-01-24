package us.ihmc.urdfLoader;

import us.ihmc.urdfLoader.xmlDescription.Joint;
import us.ihmc.urdfLoader.xmlDescription.Link;
import us.ihmc.urdfLoader.xmlDescription.Robot;

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

      Robot robot = (Robot) unmarshaller.unmarshal(URDFLoadingTest.class.getClassLoader().getResource("urdfRobotTest.urdf"));

      System.out.println("Robot name: " + robot.getName());

      List<Object> jointAndLinkAndMaterial = robot.getJointAndLinkAndMaterial();

      for (Object obj : jointAndLinkAndMaterial)
      {
         if(obj instanceof Link)
         {
            Link link = (Link) obj;

            System.out.println("Link: " + link.getName());
         }

         if(obj instanceof Joint)
         {
            Joint joint = (Joint) obj;

            System.out.println("Joint: " + joint.getName());
            System.out.println("Parent link: " + joint.getParent().getLink());
         }
      }
   }
}
