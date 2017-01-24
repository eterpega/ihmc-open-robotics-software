
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

    private final static QName _TransmissionUseSimulatedGripperJoint_QNAME = new QName("http://www.ros.org", "use_simulated_gripper_joint");
    private final static QName _TransmissionJoint_QNAME = new QName("http://www.ros.org", "joint");
    private final static QName _TransmissionRightActuator_QNAME = new QName("http://www.ros.org", "rightActuator");
    private final static QName _TransmissionRollJoint_QNAME = new QName("http://www.ros.org", "rollJoint");
    private final static QName _TransmissionActuator_QNAME = new QName("http://www.ros.org", "actuator");
    private final static QName _TransmissionFlexJoint_QNAME = new QName("http://www.ros.org", "flexJoint");
    private final static QName _TransmissionLeftActuator_QNAME = new QName("http://www.ros.org", "leftActuator");
    private final static QName _TransmissionPassiveJoint_QNAME = new QName("http://www.ros.org", "passive_joint");
    private final static QName _TransmissionMechanicalReduction_QNAME = new QName("http://www.ros.org", "mechanicalReduction");
    private final static QName _TransmissionGapJoint_QNAME = new QName("http://www.ros.org", "gap_joint");

    /**
     * Create a new ObjectFactory that can be used to create new instances of schema derived classes for package: us.ihmc.urdfLoader.xmlDescription
     * 
     */
    public ObjectFactory() {
    }

    /**
     * Create an instance of {@link Transmission }
     * 
     */
    public Transmission createTransmission() {
        return new Transmission();
    }

    /**
     * Create an instance of {@link Robot }
     * 
     */
    public Robot createRobot() {
        return new Robot();
    }

    /**
     * Create an instance of {@link Joint }
     * 
     */
    public Joint createJoint() {
        return new Joint();
    }

    /**
     * Create an instance of {@link Link }
     * 
     */
    public Link createLink() {
        return new Link();
    }

    /**
     * Create an instance of {@link MaterialGlobal }
     * 
     */
    public MaterialGlobal createMaterialGlobal() {
        return new MaterialGlobal();
    }

    /**
     * Create an instance of {@link Parent }
     * 
     */
    public Parent createParent() {
        return new Parent();
    }

    /**
     * Create an instance of {@link Color }
     * 
     */
    public Color createColor() {
        return new Color();
    }

    /**
     * Create an instance of {@link Mass }
     * 
     */
    public Mass createMass() {
        return new Mass();
    }

    /**
     * Create an instance of {@link Box }
     * 
     */
    public Box createBox() {
        return new Box();
    }

    /**
     * Create an instance of {@link Axis }
     * 
     */
    public Axis createAxis() {
        return new Axis();
    }

    /**
     * Create an instance of {@link GapJointTransmission }
     * 
     */
    public GapJointTransmission createGapJointTransmission() {
        return new GapJointTransmission();
    }

    /**
     * Create an instance of {@link Inertia }
     * 
     */
    public Inertia createInertia() {
        return new Inertia();
    }

    /**
     * Create an instance of {@link Dynamics }
     * 
     */
    public Dynamics createDynamics() {
        return new Dynamics();
    }

    /**
     * Create an instance of {@link Sphere }
     * 
     */
    public Sphere createSphere() {
        return new Sphere();
    }

    /**
     * Create an instance of {@link Limit }
     * 
     */
    public Limit createLimit() {
        return new Limit();
    }

    /**
     * Create an instance of {@link Visual }
     * 
     */
    public Visual createVisual() {
        return new Visual();
    }

    /**
     * Create an instance of {@link SafetyController }
     * 
     */
    public SafetyController createSafetyController() {
        return new SafetyController();
    }

    /**
     * Create an instance of {@link Mimic }
     * 
     */
    public Mimic createMimic() {
        return new Mimic();
    }

    /**
     * Create an instance of {@link Calibration }
     * 
     */
    public Calibration createCalibration() {
        return new Calibration();
    }

    /**
     * Create an instance of {@link PassiveJointTransmission }
     * 
     */
    public PassiveJointTransmission createPassiveJointTransmission() {
        return new PassiveJointTransmission();
    }

    /**
     * Create an instance of {@link Mesh }
     * 
     */
    public Mesh createMesh() {
        return new Mesh();
    }

    /**
     * Create an instance of {@link Pose }
     * 
     */
    public Pose createPose() {
        return new Pose();
    }

    /**
     * Create an instance of {@link Texture }
     * 
     */
    public Texture createTexture() {
        return new Texture();
    }

    /**
     * Create an instance of {@link Inertial }
     * 
     */
    public Inertial createInertial() {
        return new Inertial();
    }

    /**
     * Create an instance of {@link Verbose }
     * 
     */
    public Verbose createVerbose() {
        return new Verbose();
    }

    /**
     * Create an instance of {@link Collision }
     * 
     */
    public Collision createCollision() {
        return new Collision();
    }

    /**
     * Create an instance of {@link Material }
     * 
     */
    public Material createMaterial() {
        return new Material();
    }

    /**
     * Create an instance of {@link ActuatorTransmission }
     * 
     */
    public ActuatorTransmission createActuatorTransmission() {
        return new ActuatorTransmission();
    }

    /**
     * Create an instance of {@link Name }
     * 
     */
    public Name createName() {
        return new Name();
    }

    /**
     * Create an instance of {@link Cylinder }
     * 
     */
    public Cylinder createCylinder() {
        return new Cylinder();
    }

    /**
     * Create an instance of {@link Geometry }
     * 
     */
    public Geometry createGeometry() {
        return new Geometry();
    }

    /**
     * Create an instance of {@link Child }
     * 
     */
    public Child createChild() {
        return new Child();
    }

    /**
     * Create an instance of {@link Transmission.UseSimulatedGripperJoint }
     * 
     */
    public Transmission.UseSimulatedGripperJoint createTransmissionUseSimulatedGripperJoint() {
        return new Transmission.UseSimulatedGripperJoint();
    }

    /**
     * Create an instance of {@link JAXBElement }{@code <}{@link Transmission.UseSimulatedGripperJoint }{@code >}}
     * 
     */
    @XmlElementDecl(namespace = "http://www.ros.org", name = "use_simulated_gripper_joint", scope = Transmission.class)
    public JAXBElement<Transmission.UseSimulatedGripperJoint> createTransmissionUseSimulatedGripperJoint(Transmission.UseSimulatedGripperJoint value) {
        return new JAXBElement<Transmission.UseSimulatedGripperJoint>(_TransmissionUseSimulatedGripperJoint_QNAME, Transmission.UseSimulatedGripperJoint.class, Transmission.class, value);
    }

    /**
     * Create an instance of {@link JAXBElement }{@code <}{@link Name }{@code >}}
     * 
     */
    @XmlElementDecl(namespace = "http://www.ros.org", name = "joint", scope = Transmission.class)
    public JAXBElement<Name> createTransmissionJoint(Name value) {
        return new JAXBElement<Name>(_TransmissionJoint_QNAME, Name.class, Transmission.class, value);
    }

    /**
     * Create an instance of {@link JAXBElement }{@code <}{@link ActuatorTransmission }{@code >}}
     * 
     */
    @XmlElementDecl(namespace = "http://www.ros.org", name = "rightActuator", scope = Transmission.class)
    public JAXBElement<ActuatorTransmission> createTransmissionRightActuator(ActuatorTransmission value) {
        return new JAXBElement<ActuatorTransmission>(_TransmissionRightActuator_QNAME, ActuatorTransmission.class, Transmission.class, value);
    }

    /**
     * Create an instance of {@link JAXBElement }{@code <}{@link ActuatorTransmission }{@code >}}
     * 
     */
    @XmlElementDecl(namespace = "http://www.ros.org", name = "rollJoint", scope = Transmission.class)
    public JAXBElement<ActuatorTransmission> createTransmissionRollJoint(ActuatorTransmission value) {
        return new JAXBElement<ActuatorTransmission>(_TransmissionRollJoint_QNAME, ActuatorTransmission.class, Transmission.class, value);
    }

    /**
     * Create an instance of {@link JAXBElement }{@code <}{@link Name }{@code >}}
     * 
     */
    @XmlElementDecl(namespace = "http://www.ros.org", name = "actuator", scope = Transmission.class)
    public JAXBElement<Name> createTransmissionActuator(Name value) {
        return new JAXBElement<Name>(_TransmissionActuator_QNAME, Name.class, Transmission.class, value);
    }

    /**
     * Create an instance of {@link JAXBElement }{@code <}{@link ActuatorTransmission }{@code >}}
     * 
     */
    @XmlElementDecl(namespace = "http://www.ros.org", name = "flexJoint", scope = Transmission.class)
    public JAXBElement<ActuatorTransmission> createTransmissionFlexJoint(ActuatorTransmission value) {
        return new JAXBElement<ActuatorTransmission>(_TransmissionFlexJoint_QNAME, ActuatorTransmission.class, Transmission.class, value);
    }

    /**
     * Create an instance of {@link JAXBElement }{@code <}{@link ActuatorTransmission }{@code >}}
     * 
     */
    @XmlElementDecl(namespace = "http://www.ros.org", name = "leftActuator", scope = Transmission.class)
    public JAXBElement<ActuatorTransmission> createTransmissionLeftActuator(ActuatorTransmission value) {
        return new JAXBElement<ActuatorTransmission>(_TransmissionLeftActuator_QNAME, ActuatorTransmission.class, Transmission.class, value);
    }

    /**
     * Create an instance of {@link JAXBElement }{@code <}{@link PassiveJointTransmission }{@code >}}
     * 
     */
    @XmlElementDecl(namespace = "http://www.ros.org", name = "passive_joint", scope = Transmission.class)
    public JAXBElement<PassiveJointTransmission> createTransmissionPassiveJoint(PassiveJointTransmission value) {
        return new JAXBElement<PassiveJointTransmission>(_TransmissionPassiveJoint_QNAME, PassiveJointTransmission.class, Transmission.class, value);
    }

    /**
     * Create an instance of {@link JAXBElement }{@code <}{@link Double }{@code >}}
     * 
     */
    @XmlElementDecl(namespace = "http://www.ros.org", name = "mechanicalReduction", scope = Transmission.class)
    public JAXBElement<Double> createTransmissionMechanicalReduction(Double value) {
        return new JAXBElement<Double>(_TransmissionMechanicalReduction_QNAME, Double.class, Transmission.class, value);
    }

    /**
     * Create an instance of {@link JAXBElement }{@code <}{@link GapJointTransmission }{@code >}}
     * 
     */
    @XmlElementDecl(namespace = "http://www.ros.org", name = "gap_joint", scope = Transmission.class)
    public JAXBElement<GapJointTransmission> createTransmissionGapJoint(GapJointTransmission value) {
        return new JAXBElement<GapJointTransmission>(_TransmissionGapJoint_QNAME, GapJointTransmission.class, Transmission.class, value);
    }

}
