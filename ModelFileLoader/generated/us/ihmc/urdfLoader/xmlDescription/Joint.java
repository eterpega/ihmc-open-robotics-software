
package us.ihmc.urdfLoader.xmlDescription;

import javax.annotation.Generated;
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlType;


/**
 * <p>Java class for joint complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="joint">
 *   &lt;complexContent>
 *     &lt;restriction base="{http://www.w3.org/2001/XMLSchema}anyType">
 *       &lt;all>
 *         &lt;element name="origin" type="{http://www.ros.org}pose" minOccurs="0"/>
 *         &lt;element name="parent" type="{http://www.ros.org}parent"/>
 *         &lt;element name="child" type="{http://www.ros.org}child"/>
 *         &lt;element name="axis" type="{http://www.ros.org}axis" minOccurs="0"/>
 *         &lt;element name="calibration" type="{http://www.ros.org}calibration" minOccurs="0"/>
 *         &lt;element name="dynamics" type="{http://www.ros.org}dynamics" minOccurs="0"/>
 *         &lt;element name="limit" type="{http://www.ros.org}limit" minOccurs="0"/>
 *         &lt;element name="safety_controller" type="{http://www.ros.org}safety_controller" minOccurs="0"/>
 *         &lt;element name="mimic" type="{http://www.ros.org}mimic" minOccurs="0"/>
 *       &lt;/all>
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
@XmlType(name = "joint", namespace = "http://www.ros.org", propOrder = {

})
@Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
public class Joint {

    @XmlElement(namespace = "http://www.ros.org")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Pose origin;
    @XmlElement(namespace = "http://www.ros.org", required = true)
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Parent parent;
    @XmlElement(namespace = "http://www.ros.org", required = true)
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Child child;
    @XmlElement(namespace = "http://www.ros.org")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Axis axis;
    @XmlElement(namespace = "http://www.ros.org")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Calibration calibration;
    @XmlElement(namespace = "http://www.ros.org")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Dynamics dynamics;
    @XmlElement(namespace = "http://www.ros.org")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Limit limit;
    @XmlElement(name = "safety_controller", namespace = "http://www.ros.org")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected SafetyController safetyController;
    @XmlElement(namespace = "http://www.ros.org")
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected Mimic mimic;
    @XmlAttribute(name = "name", required = true)
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected String name;
    @XmlAttribute(name = "type", required = true)
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected String type;

    /**
     * Gets the value of the origin property.
     * 
     * @return
     *     possible object is
     *     {@link Pose }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public Pose getOrigin() {
        return origin;
    }

    /**
     * Sets the value of the origin property.
     * 
     * @param value
     *     allowed object is
     *     {@link Pose }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setOrigin(Pose value) {
        this.origin = value;
    }

    /**
     * Gets the value of the parent property.
     * 
     * @return
     *     possible object is
     *     {@link Parent }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public Parent getParent() {
        return parent;
    }

    /**
     * Sets the value of the parent property.
     * 
     * @param value
     *     allowed object is
     *     {@link Parent }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setParent(Parent value) {
        this.parent = value;
    }

    /**
     * Gets the value of the child property.
     * 
     * @return
     *     possible object is
     *     {@link Child }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public Child getChild() {
        return child;
    }

    /**
     * Sets the value of the child property.
     * 
     * @param value
     *     allowed object is
     *     {@link Child }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setChild(Child value) {
        this.child = value;
    }

    /**
     * Gets the value of the axis property.
     * 
     * @return
     *     possible object is
     *     {@link Axis }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public Axis getAxis() {
        return axis;
    }

    /**
     * Sets the value of the axis property.
     * 
     * @param value
     *     allowed object is
     *     {@link Axis }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setAxis(Axis value) {
        this.axis = value;
    }

    /**
     * Gets the value of the calibration property.
     * 
     * @return
     *     possible object is
     *     {@link Calibration }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public Calibration getCalibration() {
        return calibration;
    }

    /**
     * Sets the value of the calibration property.
     * 
     * @param value
     *     allowed object is
     *     {@link Calibration }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setCalibration(Calibration value) {
        this.calibration = value;
    }

    /**
     * Gets the value of the dynamics property.
     * 
     * @return
     *     possible object is
     *     {@link Dynamics }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public Dynamics getDynamics() {
        return dynamics;
    }

    /**
     * Sets the value of the dynamics property.
     * 
     * @param value
     *     allowed object is
     *     {@link Dynamics }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setDynamics(Dynamics value) {
        this.dynamics = value;
    }

    /**
     * Gets the value of the limit property.
     * 
     * @return
     *     possible object is
     *     {@link Limit }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public Limit getLimit() {
        return limit;
    }

    /**
     * Sets the value of the limit property.
     * 
     * @param value
     *     allowed object is
     *     {@link Limit }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setLimit(Limit value) {
        this.limit = value;
    }

    /**
     * Gets the value of the safetyController property.
     * 
     * @return
     *     possible object is
     *     {@link SafetyController }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public SafetyController getSafetyController() {
        return safetyController;
    }

    /**
     * Sets the value of the safetyController property.
     * 
     * @param value
     *     allowed object is
     *     {@link SafetyController }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setSafetyController(SafetyController value) {
        this.safetyController = value;
    }

    /**
     * Gets the value of the mimic property.
     * 
     * @return
     *     possible object is
     *     {@link Mimic }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public Mimic getMimic() {
        return mimic;
    }

    /**
     * Sets the value of the mimic property.
     * 
     * @param value
     *     allowed object is
     *     {@link Mimic }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setMimic(Mimic value) {
        this.mimic = value;
    }

    /**
     * Gets the value of the name property.
     * 
     * @return
     *     possible object is
     *     {@link String }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
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
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
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
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
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
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:35:46-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setType(String value) {
        this.type = value;
    }

}
