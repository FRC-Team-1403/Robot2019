
package frc.robot.echo;


import java.io.FileInputStream;
import java.io.IOException;
import java.io.ObjectInputStream;

import frc.robot.echo.Recording;

//basic wrapper
public class EchoReader {
	private ObjectInputStream fileReader; //import these later
	private Recording[] recordingArr;
	
	public EchoReader() {
	}
	
	/**
	  * Initialize the Reader Stream (For the Serialized Array)
	  * Parameters: FileInputStream from the Recorder Class
	  */
	public void initReader(FileInputStream file) {
		try {
			fileReader = new ObjectInputStream(file);
		} catch (IOException e) {
			e.printStackTrace();
		}
		System.out.println("ObjectInputStream obj finished init");
	}
	
	/**
	  * Unserialize array from previously recorded file and convert it back into an array of Recording(s)
	  * Return Type: Recording[]
	  */
	public Recording[] getArrayFromFile() { //return null if nonexistent
		try {
			this.recordingArr = (Recording[]) fileReader.readObject();
		} catch (ClassNotFoundException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		return this.recordingArr;
	}
	
	/**
	  * Return the temporarily stored array from the EchoReader class
	  */
	public Recording[] returnArrayFromFile() {
		return this.recordingArr;
	}
	
	/**
	  * Reset the Reader Stream object
	  */
	public void destroy(){
		try {
			fileReader.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}