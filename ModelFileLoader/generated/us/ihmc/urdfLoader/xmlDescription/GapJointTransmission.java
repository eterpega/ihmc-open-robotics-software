
package us.ihmc.urdfLoader.xmlDescription;

import javax.annotation.Generated;
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlAttribute;
import javax.xml.bind.annotation.XmlType;


/**
 * <p>Java class for gap_joint_transmission complex type.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * 
 * <pre>
 * &lt;complexType name="gap_joint_transmission">
 *   &lt;complexContent>
 *     &lt;restriction base="{http://www.w3.org/2001/XMLSchema}anyType">
 *       &lt;attribute name="L0" use="required" type="{http://www.w3.org/2001/XMLSchema}double" />
 *       &lt;attribute name="a" use="required" type="{http://www.w3.org/2001/XMLSchema}double" />
 *       &lt;attribute name="b" use="required" type="{http://www.w3.org/2001/XMLSchema}double" />
 *       &lt;attribute name="gear_ratio" use="required" type="{http://www.w3.org/2001/XMLSchema}double" />
 *       &lt;attribute name="h" use="required" type="{http://www.w3.org/2001/XMLSchema}double" />
 *       &lt;attribute name="mechanical_reduction" use="required" type="{http://www.w3.org/2001/XMLSchema}double" />
 *       &lt;attribute name="name" use="required" type="{http://www.w3.org/2001/XMLSchema}string" />
 *       &lt;attribute name="phi0" use="required" type="{http://www.w3.org/2001/XMLSchema}double" />
 *       &lt;attribute name="r" use="required" type="{http://www.w3.org/2001/XMLSchema}double" />
 *       &lt;attribute name="screw_reduction" use="required" type="{http://www.w3.org/2001/XMLSchema}double" />
 *       &lt;attribute name="t0" use="required" type="{http://www.w3.org/2001/XMLSchema}double" />
 *       &lt;attribute name="theta0" use="required" type="{http://www.w3.org/2001/XMLSchema}double" />
 *     &lt;/restriction>
 *   &lt;/complexContent>
 * &lt;/complexType>
 * </pre>
 * 
 * 
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "gap_joint_transmission", namespace = "http://www.ros.org")
@Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
public class GapJointTransmission {

    @XmlAttribute(name = "L0", required = true)
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected double l0;
    @XmlAttribute(name = "a", required = true)
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected double a;
    @XmlAttribute(name = "b", required = true)
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected double b;
    @XmlAttribute(name = "gear_ratio", required = true)
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected double gearRatio;
    @XmlAttribute(name = "h", required = true)
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected double h;
    @XmlAttribute(name = "mechanical_reduction", required = true)
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected double mechanicalReduction;
    @XmlAttribute(name = "name", required = true)
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected String name;
    @XmlAttribute(name = "phi0", required = true)
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected double phi0;
    @XmlAttribute(name = "r", required = true)
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected double r;
    @XmlAttribute(name = "screw_reduction", required = true)
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected double screwReduction;
    @XmlAttribute(name = "t0", required = true)
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected double t0;
    @XmlAttribute(name = "theta0", required = true)
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    protected double theta0;

    /**
     * Gets the value of the l0 property.
     * 
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public double getL0() {
        return l0;
    }

    /**
     * Sets the value of the l0 property.
     * 
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setL0(double value) {
        this.l0 = value;
    }

    /**
     * Gets the value of the a property.
     * 
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public double getA() {
        return a;
    }

    /**
     * Sets the value of the a property.
     * 
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setA(double value) {
        this.a = value;
    }

    /**
     * Gets the value of the b property.
     * 
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public double getB() {
        return b;
    }

    /**
     * Sets the value of the b property.
     * 
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setB(double value) {
        this.b = value;
    }

    /**
     * Gets the value of the gearRatio property.
     * 
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public double getGearRatio() {
        return gearRatio;
    }

    /**
     * Sets the value of the gearRatio property.
     * 
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setGearRatio(double value) {
        this.gearRatio = value;
    }

    /**
     * Gets the value of the h property.
     * 
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public double getH() {
        return h;
    }

    /**
     * Sets the value of the h property.
     * 
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setH(double value) {
        this.h = value;
    }

    /**
     * Gets the value of the mechanicalReduction property.
     * 
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public double getMechanicalReduction() {
        return mechanicalReduction;
    }

    /**
     * Sets the value of the mechanicalReduction property.
     * 
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setMechanicalReduction(double value) {
        this.mechanicalReduction = value;
    }

    /**
     * Gets the value of the name property.
     * 
     * @return
     *     possible object is
     *     {@link String }
     *     
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
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
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setName(String value) {
        this.name = value;
    }

    /**
     * Gets the value of the phi0 property.
     * 
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public double getPhi0() {
        return phi0;
    }

    /**
     * Sets the value of the phi0 property.
     * 
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setPhi0(double value) {
        this.phi0 = value;
    }

    /**
     * Gets the value of the r property.
     * 
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public double getR() {
        return r;
    }

    /**
     * Sets the value of the r property.
     * 
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setR(double value) {
        this.r = value;
    }

    /**
     * Gets the value of the screwReduction property.
     * 
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public double getScrewReduction() {
        return screwReduction;
    }

    /**
     * Sets the value of the screwReduction property.
     * 
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setScrewReduction(double value) {
        this.screwReduction = value;
    }

    /**
     * Gets the value of the t0 property.
     * 
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public double getT0() {
        return t0;
    }

    /**
     * Sets the value of the t0 property.
     * 
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setT0(double value) {
        this.t0 = value;
    }

    /**
     * Gets the value of the theta0 property.
     * 
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public double getTheta0() {
        return theta0;
    }

    /**
     * Sets the value of the theta0 property.
     * 
     */
    @Generated(value = "com.sun.tools.internal.xjc.Driver", date = "2017-01-24T03:26:12-06:00", comments = "JAXB RI v2.2.8-b130911.1802")
    public void setTheta0(double value) {
        this.theta0 = value;
    }

}
