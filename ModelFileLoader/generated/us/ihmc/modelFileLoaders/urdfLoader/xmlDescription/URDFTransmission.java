
package us.ihmc.modelFileLoaders.urdfLoader.xmlDescription;

import java.util.ArrayList;
import java.util.List;
import javax.annotation.Generated;
import javax.xml.bind.JAXBElement;
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElementRef;
import javax.xml.bind.annotation.XmlElementRefs;
import javax.xml.bind.annotation.XmlType;


/**
 * <p>Java class for transmission complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="transmission">
 *   &lt;complexContent>
 *     &lt;restriction base="{http://www.w3.org/2001/XMLSchema}anyType">
 *       &lt;sequence maxOccurs="unbounded" minOccurs="0">
 *         &lt;element name="leftActuator" type="{http://www.ros.org}actuator_transmission" minOccurs="0"/>
 *         &lt;element name="rightActuator" type="{http://www.ros.org}actuator_transmission" minOccurs="0"/>
 *         &lt;element name="flexJoint" type="{http://www.ros.org}actuator_transmission" minOccurs="0"/>
 *         &lt;element name="rollJoint" type="{http://www.ros.org}actuator_transmission" minOccurs="0"/>
 *         &lt;element name="gap_joint" type="{http://www.ros.org}gap_joint_transmission" minOccurs="0"/>
 *         &lt;element name="passive_joint" type="{http://www.ros.org}passive_joint_transmission" maxOccurs="unbounded" minOccurs="0"/>
 *         &lt;element name="use_simulated_gripper_joint" minOccurs="0">
 *           &lt;complexType>
 *             &lt;complexContent>
 *               &lt;restriction base="{http://www.w3.org/2001/XMLSchema}anyType">
 *               &lt;/restriction>
 *             &lt;/complexContent>
 *           &lt;/complexType>
 *         &lt;/element>
 *         &lt;element name="mechanicalReduction" type="{http://www.w3.org/2001/XMLSchema}double" minOccurs="0"/>
 *         &lt;element name="actuator" type="{http://www.ros.org}name" minOccurs="0"/>
 *         &lt;element name="joint" type="{http://www.ros.org}name" minOccurs="0"/>
 *       &lt;/sequence>
 *       &lt;attribute name="name" use="required" type="{http://www.w3.org/2001/XMLSchema}string" />
 *       &lt;attribute name="type" use="required" type="{http://www.w3.org/2001/XMLSchema}string" />
 *     &lt;/restriction>
 *   &lt;/complexContent>
 * &lt;/complexType>
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "transmission", namespace = "http://www.ros.org", propOrder = {
    "leftActuatorAndRightActuatorAndFlexJoint"
})
@Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
public class URDFTransmission {

    @XmlElementRefs({
        @XmlElementRef(name = "use_simulated_gripper_joint", namespace = "http://www.ros.org", type = JAXBElement.class, required = false),
        @XmlElementRef(name = "flexJoint", namespace = "http://www.ros.org", type = JAXBElement.class, required = false),
        @XmlElementRef(name = "passive_joint", namespace = "http://www.ros.org", type = JAXBElement.class, required = false),
        @XmlElementRef(name = "joint", namespace = "http://www.ros.org", type = JAXBElement.class, required = false),
        @XmlElementRef(name = "gap_joint", namespace = "http://www.ros.org", type = JAXBElement.class, required = false),
        @XmlElementRef(name = "rightActuator", namespace = "http://www.ros.org", type = JAXBElement.class, required = false),
        @XmlElementRef(name = "mechanicalReduction", namespace = "http://www.ros.org", type = JAXBElement.class, required = false),
        @XmlElementRef(name = "actuator", namespace = "http://www.ros.org", type = JAXBElement.class, required = false),
        @XmlElementRef(name = "rollJoint", namespace = "http://www.ros.org", type = JAXBElement.class, required = false),
        @XmlElementRef(name = "leftActuator", namespace = "http://www.ros.org", type = JAXBElement.class, required = false)
    })
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected List<JAXBElement<?>> leftActuatorAndRightActuatorAndFlexJoint;
    @XmlAttribute(name = "name", required = true)
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected String name;
    @XmlAttribute(name = "type", required = true)
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected String type;

    /**
     * Gets the value of the leftActuatorAndRightActuatorAndFlexJoint property.
     * 
     * <p>
     * This accessor method returns a reference to the live list,
     * not a snapshot. Therefore any modification you make to the
     * returned list will be present inside the JAXB object.
     * This is why there is not a <CODE>set</CODE> method for the leftActuatorAndRightActuatorAndFlexJoint property.
     * 
     * <p>
     * For example, to add a new item, do as follows:
     * <pre>
     *    getLeftActuatorAndRightActuatorAndFlexJoint().add(newItem);
     * </pre>
     * 
     * 
     * <p>
     * Objects of the following type(s) are allowed in the list
     * {@link JAXBElement }{@code <}{@link URDFTransmission.URDFUseSimulatedGripperJoint }{@code >}
     * {@link JAXBElement }{@code <}{@link URDFActuatorTransmission }{@code >}
     * {@link JAXBElement }{@code <}{@link URDFPassiveJointTransmission }{@code >}
     * {@link JAXBElement }{@code <}{@link URDFName }{@code >}
     * {@link JAXBElement }{@code <}{@link URDFGapJointTransmission }{@code >}
     * {@link JAXBElement }{@code <}{@link URDFActuatorTransmission }{@code >}
     * {@link JAXBElement }{@code <}{@link Double }{@code >}
     * {@link JAXBElement }{@code <}{@link URDFName }{@code >}
     * {@link JAXBElement }{@code <}{@link URDFActuatorTransmission }{@code >}
     * {@link JAXBElement }{@code <}{@link URDFActuatorTransmission }{@code >}
     * 
     * 
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public List<JAXBElement<?>> getLeftActuatorAndRightActuatorAndFlexJoint() {
        if (leftActuatorAndRightActuatorAndFlexJoint == null) {
            leftActuatorAndRightActuatorAndFlexJoint = new ArrayList<JAXBElement<?>>();
        }
        return this.leftActuatorAndRightActuatorAndFlexJoint;
    }

    /**
     * Gets the value of the name property.
     * 
     * @return
     *     possible object is
     *     {@link String }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public String getName() {
        return name;
    }

    /**
     * Sets the value of the name property.
     * 
     * @param value
     *     allowed object is
     *     {@link String }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setName(String value) {
        this.name = value;
    }

    /**
     * Gets the value of the type property.
     * 
     * @return
     *     possible object is
     *     {@link String }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public String getType() {
        return type;
    }

    /**
     * Sets the value of the type property.
     * 
     * @param value
     *     allowed object is
     *     {@link String }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setType(String value) {
        this.type = value;
    }


    /**
     * <p>Java class for anonymous complex type.
     * 
     * <p>The following schema fragment specifies the expected content contained within this class.
     * 
     * <pre>
     * &lt;complexType>
     *   &lt;complexContent>
     *     &lt;restriction base="{http://www.w3.org/2001/XMLSchema}anyType">
     *     &lt;/restriction>
     *   &lt;/complexContent>
     * &lt;/complexType>
     * </pre>
     * 
     * 
     */
    @XmlAccessorType(XmlAccessType.FIELD)
    @XmlType(name = "")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T04:55:00-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public static class URDFUseSimulatedGripperJoint {


    }

}
