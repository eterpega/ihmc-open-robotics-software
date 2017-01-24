
package us.ihmc.urdfLoader.xmlDescription;

import javax.xml.bind.JAXBElement;
import javax.xml.bind.annotation.XmlElementDecl;
import javax.xml.bind.annotation.XmlRegistry;
import javax.xml.namespace.QName;


/**
 * This object contains factory methods for each 
 * Java content interface and Java element interface 
 * generated in the us.ihmc.urdfLoader.xmlDescription package. 
 * <p>An ObjectFactory allows you to programatically 
 * construct new instances of the Java representation 
 * for XML content. The Java representation of XML 
 * content can consist of schema derived interfaces 
 * and classes representing the binding of schema 
 * type definitions, element declarations and model 
 * groups.  Factory methods for each of these are 
 * provided in this class.
 * 
 */
@XmlRegistry
public class ObjectFactory {

    private final static QName _URDFTransmissionUseSimulatedGripperJoint_QNAME = new QName("http://www.ros.org", "use_simulated_gripper_joint");
    private final static QName _URDFTransmissionJoint_QNAME = new QName("http://www.ros.org", "joint");
    private final static QName _URDFTransmissionRightActuator_QNAME = new QName("http://www.ros.org", "rightActuator");
    private final static QName _URDFTransmissionRollJoint_QNAME = new QName("http://www.ros.org", "rollJoint");
    private final static QName _URDFTransmissionActuator_QNAME = new QName("http://www.ros.org", "actuator");
    private final static QName _URDFTransmissionFlexJoint_QNAME = new QName("http://www.ros.org", "flexJoint");
    private final static QName _URDFTransmissionLeftActuator_QNAME = new QName("http://www.ros.org", "leftActuator");
    private final static QName _URDFTransmissionPassiveJoint_QNAME = new QName("http://www.ros.org", "passive_joint");
    private final static QName _URDFTransmissionMechanicalReduction_QNAME = new QName("http://www.ros.org", "mechanicalReduction");
    private final static QName _URDFTransmissionGapJoint_QNAME = new QName("http://www.ros.org", "gap_joint");

    /**
     * Create a new ObjectFactory that can be used to create new instances of schema derived classes for package: us.ihmc.urdfLoader.xmlDescription
     * 
     */
    public ObjectFactory() {
    }

    /**
     * Create an instance of {@link URDFTransmission }
     * 
     */
    public URDFTransmission createURDFTransmission() {
        return new URDFTransmission();
    }

    /**
     * Create an instance of {@link URDFRobot }
     * 
     */
    public URDFRobot createURDFRobot() {
        return new URDFRobot();
    }

    /**
     * Create an instance of {@link URDFJoint }
     * 
     */
    public URDFJoint createURDFJoint() {
        return new URDFJoint();
    }

    /**
     * Create an instance of {@link URDFLink }
     * 
     */
    public URDFLink createURDFLink() {
        return new URDFLink();
    }

    /**
     * Create an instance of {@link URDFMaterialGlobal }
     * 
     */
    public URDFMaterialGlobal createURDFMaterialGlobal() {
        return new URDFMaterialGlobal();
    }

    /**
     * Create an instance of {@link URDFParent }
     * 
     */
    public URDFParent createURDFParent() {
        return new URDFParent();
    }

    /**
     * Create an instance of {@link URDFColor }
     * 
     */
    public URDFColor createURDFColor() {
        return new URDFColor();
    }

    /**
     * Create an instance of {@link URDFMass }
     * 
     */
    public URDFMass createURDFMass() {
        return new URDFMass();
    }

    /**
     * Create an instance of {@link URDFBox }
     * 
     */
    public URDFBox createURDFBox() {
        return new URDFBox();
    }

    /**
     * Create an instance of {@link URDFAxis }
     * 
     */
    public URDFAxis createURDFAxis() {
        return new URDFAxis();
    }

    /**
     * Create an instance of {@link URDFGapJointTransmission }
     * 
     */
    public URDFGapJointTransmission createURDFGapJointTransmission() {
        return new URDFGapJointTransmission();
    }

    /**
     * Create an instance of {@link URDFInertia }
     * 
     */
    public URDFInertia createURDFInertia() {
        return new URDFInertia();
    }

    /**
     * Create an instance of {@link URDFDynamics }
     * 
     */
    public URDFDynamics createURDFDynamics() {
        return new URDFDynamics();
    }

    /**
     * Create an instance of {@link URDFSphere }
     * 
     */
    public URDFSphere createURDFSphere() {
        return new URDFSphere();
    }

    /**
     * Create an instance of {@link URDFLimit }
     * 
     */
    public URDFLimit createURDFLimit() {
        return new URDFLimit();
    }

    /**
     * Create an instance of {@link URDFVisual }
     * 
     */
    public URDFVisual createURDFVisual() {
        return new URDFVisual();
    }

    /**
     * Create an instance of {@link URDFSafetyController }
     * 
     */
    public URDFSafetyController createURDFSafetyController() {
        return new URDFSafetyController();
    }

    /**
     * Create an instance of {@link URDFMimic }
     * 
     */
    public URDFMimic createURDFMimic() {
        return new URDFMimic();
    }

    /**
     * Create an instance of {@link URDFCalibration }
     * 
     */
    public URDFCalibration createURDFCalibration() {
        return new URDFCalibration();
    }

    /**
     * Create an instance of {@link URDFPassiveJointTransmission }
     * 
     */
    public URDFPassiveJointTransmission createURDFPassiveJointTransmission() {
        return new URDFPassiveJointTransmission();
    }

    /**
     * Create an instance of {@link URDFMesh }
     * 
     */
    public URDFMesh createURDFMesh() {
        return new URDFMesh();
    }

    /**
     * Create an instance of {@link URDFPose }
     * 
     */
    public URDFPose createURDFPose() {
        return new URDFPose();
    }

    /**
     * Create an instance of {@link URDFTexture }
     * 
     */
    public URDFTexture createURDFTexture() {
        return new URDFTexture();
    }

    /**
     * Create an instance of {@link URDFInertial }
     * 
     */
    public URDFInertial createURDFInertial() {
        return new URDFInertial();
    }

    /**
     * Create an instance of {@link URDFVerbose }
     * 
     */
    public URDFVerbose createURDFVerbose() {
        return new URDFVerbose();
    }

    /**
     * Create an instance of {@link URDFCollision }
     * 
     */
    public URDFCollision createURDFCollision() {
        return new URDFCollision();
    }

    /**
     * Create an instance of {@link URDFMaterial }
     * 
     */
    public URDFMaterial createURDFMaterial() {
        return new URDFMaterial();
    }

    /**
     * Create an instance of {@link URDFActuatorTransmission }
     * 
     */
    public URDFActuatorTransmission createURDFActuatorTransmission() {
        return new URDFActuatorTransmission();
    }

    /**
     * Create an instance of {@link URDFName }
     * 
     */
    public URDFName createURDFName() {
        return new URDFName();
    }

    /**
     * Create an instance of {@link URDFCylinder }
     * 
     */
    public URDFCylinder createURDFCylinder() {
        return new URDFCylinder();
    }

    /**
     * Create an instance of {@link URDFGeometry }
     * 
     */
    public URDFGeometry createURDFGeometry() {
        return new URDFGeometry();
    }

    /**
     * Create an instance of {@link URDFChild }
     * 
     */
    public URDFChild createURDFChild() {
        return new URDFChild();
    }

    /**
     * Create an instance of {@link URDFTransmission.URDFUseSimulatedGripperJoint }
     * 
     */
    public URDFTransmission.URDFUseSimulatedGripperJoint createURDFTransmissionURDFUseSimulatedGripperJoint() {
        return new URDFTransmission.URDFUseSimulatedGripperJoint();
    }

    /**
     * Create an instance of {@link JAXBElement }{@code <}{@link URDFTransmission.URDFUseSimulatedGripperJoint }{@code >}}
     * 
     */
    @XmlElementDecl(namespace = "http://www.ros.org", name = "use_simulated_gripper_joint", scope = URDFTransmission.class)
    public JAXBElement<URDFTransmission.URDFUseSimulatedGripperJoint> createURDFTransmissionUseSimulatedGripperJoint(URDFTransmission.URDFUseSimulatedGripperJoint value) {
        return new JAXBElement<URDFTransmission.URDFUseSimulatedGripperJoint>(_URDFTransmissionUseSimulatedGripperJoint_QNAME, URDFTransmission.URDFUseSimulatedGripperJoint.class, URDFTransmission.class, value);
    }

    /**
     * Create an instance of {@link JAXBElement }{@code <}{@link URDFName }{@code >}}
     * 
     */
    @XmlElementDecl(namespace = "http://www.ros.org", name = "joint", scope = URDFTransmission.class)
    public JAXBElement<URDFName> createURDFTransmissionJoint(URDFName value) {
        return new JAXBElement<URDFName>(_URDFTransmissionJoint_QNAME, URDFName.class, URDFTransmission.class, value);
    }

    /**
     * Create an instance of {@link JAXBElement }{@code <}{@link URDFActuatorTransmission }{@code >}}
     * 
     */
    @XmlElementDecl(namespace = "http://www.ros.org", name = "rightActuator", scope = URDFTransmission.class)
    public JAXBElement<URDFActuatorTransmission> createURDFTransmissionRightActuator(URDFActuatorTransmission value) {
        return new JAXBElement<URDFActuatorTransmission>(_URDFTransmissionRightActuator_QNAME, URDFActuatorTransmission.class, URDFTransmission.class, value);
    }

    /**
     * Create an instance of {@link JAXBElement }{@code <}{@link URDFActuatorTransmission }{@code >}}
     * 
     */
    @XmlElementDecl(namespace = "http://www.ros.org", name = "rollJoint", scope = URDFTransmission.class)
    public JAXBElement<URDFActuatorTransmission> createURDFTransmissionRollJoint(URDFActuatorTransmission value) {
        return new JAXBElement<URDFActuatorTransmission>(_URDFTransmissionRollJoint_QNAME, URDFActuatorTransmission.class, URDFTransmission.class, value);
    }

    /**
     * Create an instance of {@link JAXBElement }{@code <}{@link URDFName }{@code >}}
     * 
     */
    @XmlElementDecl(namespace = "http://www.ros.org", name = "actuator", scope = URDFTransmission.class)
    public JAXBElement<URDFName> createURDFTransmissionActuator(URDFName value) {
        return new JAXBElement<URDFName>(_URDFTransmissionActuator_QNAME, URDFName.class, URDFTransmission.class, value);
    }

    /**
     * Create an instance of {@link JAXBElement }{@code <}{@link URDFActuatorTransmission }{@code >}}
     * 
     */
    @XmlElementDecl(namespace = "http://www.ros.org", name = "flexJoint", scope = URDFTransmission.class)
    public JAXBElement<URDFActuatorTransmission> createURDFTransmissionFlexJoint(URDFActuatorTransmission value) {
        return new JAXBElement<URDFActuatorTransmission>(_URDFTransmissionFlexJoint_QNAME, URDFActuatorTransmission.class, URDFTransmission.class, value);
    }

    /**
     * Create an instance of {@link JAXBElement }{@code <}{@link URDFActuatorTransmission }{@code >}}
     * 
     */
    @XmlElementDecl(namespace = "http://www.ros.org", name = "leftActuator", scope = URDFTransmission.class)
    public JAXBElement<URDFActuatorTransmission> createURDFTransmissionLeftActuator(URDFActuatorTransmission value) {
        return new JAXBElement<URDFActuatorTransmission>(_URDFTransmissionLeftActuator_QNAME, URDFActuatorTransmission.class, URDFTransmission.class, value);
    }

    /**
     * Create an instance of {@link JAXBElement }{@code <}{@link URDFPassiveJointTransmission }{@code >}}
     * 
     */
    @XmlElementDecl(namespace = "http://www.ros.org", name = "passive_joint", scope = URDFTransmission.class)
    public JAXBElement<URDFPassiveJointTransmission> createURDFTransmissionPassiveJoint(URDFPassiveJointTransmission value) {
        return new JAXBElement<URDFPassiveJointTransmission>(_URDFTransmissionPassiveJoint_QNAME, URDFPassiveJointTransmission.class, URDFTransmission.class, value);
    }

    /**
     * Create an instance of {@link JAXBElement }{@code <}{@link Double }{@code >}}
     * 
     */
    @XmlElementDecl(namespace = "http://www.ros.org", name = "mechanicalReduction", scope = URDFTransmission.class)
    public JAXBElement<Double> createURDFTransmissionMechanicalReduction(Double value) {
        return new JAXBElement<Double>(_URDFTransmissionMechanicalReduction_QNAME, Double.class, URDFTransmission.class, value);
    }

    /**
     * Create an instance of {@link JAXBElement }{@code <}{@link URDFGapJointTransmission }{@code >}}
     * 
     */
    @XmlElementDecl(namespace = "http://www.ros.org", name = "gap_joint", scope = URDFTransmission.class)
    public JAXBElement<URDFGapJointTransmission> createURDFTransmissionGapJoint(URDFGapJointTransmission value) {
        return new JAXBElement<URDFGapJointTransmission>(_URDFTransmissionGapJoint_QNAME, URDFGapJointTransmission.class, URDFTransmission.class, value);
    }

}
