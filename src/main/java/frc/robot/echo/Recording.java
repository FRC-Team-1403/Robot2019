package frc.robot.echo;


import java.io.Serializable;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Recording implements Serializable{
	
	private int index;
	private Map<String, Double> Data;
	
	/**
	  * Recording Constructor, Initialize "index" variable as -1
	  */
	public Recording() {
		this.Data = new HashMap<String, Double>();
		this.index = -1;
	}
	
	/**
	  * Method to label the Recording with an index(Chronologically Decided based on place in Recording[])
	  */
	public void addIndex(int index) {
		this.index = index;
	}
	
	/**
	  * Returns the index of this Recording object
	  */
	public int returnIndex() {
		return this.index;
	}
	
	/**
	  * Adds a subsystem reading and its corresponding value
	  */
	public void addReading(String key, double value) {
		this.Data.put(key, ReferenceToPrimitive(value));
	}
	
	/**
	  * Return the value of the subsystem reading specified
	  */
	public double returnReading(String key) {
		return this.Data.get(key);
	}
	
	/**
	  * Return a referenced HashMap<String, Double> of all stored subsystem readings
	  */
	public HashMap<String, Double> returnMap() {
		return (HashMap<String, Double>)Data;	
	}
	
	/**
	  * Dummy method for dereferencing a double variable
	  */
	public double ReferenceToPrimitive(double value) {
		Double d = new Double(value);
		return (double)d;
	}
}