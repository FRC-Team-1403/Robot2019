
package frc.robot.echo;


import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;

import frc.robot.echo.Recording;

//basic wrapper
public class EchoWriter {
	private ObjectOutputStream fileWriter; //import these later
	private boolean isActive;
	
	public EchoWriter() {
		this.isActive = false;
	}
	public boolean isActive() {
		return this.isActive;
	}
	public void initWriter(FileOutputStream file){	
		try {
			fileWriter = new ObjectOutputStream(file);
			this.isActive = true;
		} catch (IOException e) {
			e.printStackTrace();
		}
		System.out.println("Object Output Init");
	}
	public void serializeToFile(Recording[] sequencedReadings) {
		try {
			fileWriter.writeObject(sequencedReadings);
			this.destroy();
		} catch (IOException e1) {
			e1.printStackTrace();
		}
		 
	}

	public void destroy(){
		try {
			fileWriter.flush();
			fileWriter.close();
			this.isActive = false;
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}