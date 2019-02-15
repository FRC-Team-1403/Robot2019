package frc.robot.echo;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Recorder {
	private final int RMAX; //20 ms delay * 50 ms in 1 second * 15 seconds
	private int iteration;
	private Recording interpRecording;
	//private long timeStamp; 
	private long prevTime;
	private Recording[] sequencedReadings;
	public static boolean isRecording;
	public static boolean isStoring;
	
	public Map<Integer, String> FileSelect;
	public static EchoWriter writer;
	public static EchoReader reader;
	public static FileOutputStream writerFile;
	public static FileInputStream readerFile;
	
	/**
	 * Construct Recorder object, Parameter denotes the maximum size of Recording[]
	 **/
	public Recorder(int arrsize) {
		this.RMAX = arrsize;
		//this.timeStamp = 0;
		this.iteration = 0;
		this.prevTime = 0;
		Recorder.isRecording = false;
		Recorder.isStoring = false;
		this.sequencedReadings = new Recording[RMAX];
		this.sequencedReadings[iteration] = new Recording();
		this.FileSelect = new HashMap<Integer, String>();
		Recorder.writer = new EchoWriter();
		Recorder.reader = new EchoReader();
	}
	
	/**
	 * Returns the condition of whether or not Recorder is recording
	 **/
	public static boolean isStoring() {
		if(Recorder.isStoring) {
			Recorder.isStoring = false;
			System.out.println("Recorder Storing Status: " + Recorder.isStoring);
			return true;
		}
		return false;
	}
	
	/**
	 * Begin Recorder and Reinitialize the Writer object (If it exists)
	 **/
	public static void startRecording() {
		Recorder.isRecording = true;
		if(!Recorder.writer.isActive()) {
			initWriter();
		}
	}
	
	/**
	 * Return a Recording object from a SPECIFIED time index in the Recording[]
	 **/
	public Recording getRecording(int index) {
		return sequencedReadings[index];
	}
	
	/**
	 * Return a subsystem reading from the CURRENT time index
	 **/
	public double getReading(String key) {
		return sequencedReadings[iteration].returnReading(key);
	}

	
	/** Return the interpolated reading from the CURRENT time index
	 * 
	 */
	public double getInterpReading(String key) {
		return interpRecording.returnReading(key);
	}
	/**
	 * TELEOPERATED MODE: Specify the subsystem key and its value and add it to the current Recording object from the Recording[] 
	 **/
	public void addReading(String key, double value) {
		this.sequencedReadings[iteration].addIndex(iteration);
		this.sequencedReadings[iteration].addReading(key, value);
	}
	
	/**
	 * Call when TELEOPERATED recording is finished. [Prevents a java.io.IOException]
	 **/
	public void doneRecording() {
		this.sequencedReadings[iteration] = new Recording();
		this.sequencedReadings[iteration].addIndex(-1);
	}
	
	/**
	 * Autonomous: Call after all subsystems from the current timeframe have finished playing back (Iterates to next Timeframe)
	 **/
	public int nextReading() {
		return ++this.iteration;
	}
	
	/**
	 * Teleoperated: Call when all subsystems from the CURRENT timeframe have finished recording (Iterates to next Timeframe)
	 **/
	public int initNextReading() {
		this.sequencedReadings[++this.iteration] = new Recording();
		return this.iteration;
	}
	
	/**
	 * Add previously existing file path to the Robot's list of recordings
	 **/
	public void addFileSelect(int index, String file) {
		this.FileSelect.put(index, file);
	}
	
	/**
	 * Based on the list of file paths, choose one of them to write to
	 **/
	public FileOutputStream setCurrentWritefile(int index) {
		try {
			Recorder.writerFile = new FileOutputStream(this.FileSelect.get(index), false);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
		return writerFile;
	}
	
	/**
	 * Based on the list of file paths, choose one of them to read from
	 **/
	public FileInputStream setCurrentReadfile(int index) {
		try {
			Recorder.readerFile = new FileInputStream(this.FileSelect.get(index));
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
		return readerFile;
	}
	
	/**
	 * Call when ready to read from a file and store its values
	 **/
	public void storeReadings() {
		Recorder.reader.getArrayFromFile();
		sequencedReadings = Arrays.copyOf(Recorder.reader.returnArrayFromFile(), Recorder.reader.returnArrayFromFile().length);
	}
	
	/**
	 * Call when ready to store Recording[] to the specified file
	 **/
	public void storeWritings() {
		this.doneRecording();
		for(Recording r : sequencedReadings) {
			if(r.returnIndex() == -1)
				break;
		}
		writer.serializeToFile(sequencedReadings);
	}
	
	/**
	 * Initialize Writer Object
	 **/
	public static void initWriter() {
		System.out.println("Writer Init Recorder");
		writer.initWriter(writerFile);
	}
	
	/**
	 * Initialize Reader Object
	 **/
	public static void initReader() {
		System.out.println("Reader Init Recorder");
		reader.initReader(readerFile);
	}
	
	/**
	 * Autonomous: Check if Recording[]
	 **/
	public boolean hasNextLine() {
		boolean hasNextLine = getRecording(iteration+1).returnIndex() != -1;
		if(!hasNextLine) return hasNextLine;
		if(checkTime()) 
			nextReading();
		return hasNextLine;
	}
	
	/**
	 * Based on the list of file paths, choose one of them to write to
	 **/
	public void resetReadings() {
		this.iteration = 0;
		this.sequencedReadings = new Recording[RMAX];
		this.sequencedReadings[0] = new Recording();
	}
	
	/**
	 * Check if 20 ms has passed
	 **/
	public boolean checkTime() {
		long currTime = System.currentTimeMillis();
		System.out.println(currTime);
		if(currTime - this.prevTime < 20) {
			interpolateReadings((int)(currTime - this.prevTime));
			return false;
		} else {
			this.prevTime += 20;//= currTime;
			return true;
		}
	}
	
	/**
	 * Linear Interpolation - Corresponds to getInterpReading()
	 */
	public void interpolateReadings(int timediff) {
		//20 ms
		if(this.iteration < 1 || getRecording(iteration+1).returnIndex() == -1) 
			return;
		interpRecording = new Recording();
		for(String key : sequencedReadings[this.iteration].returnMap().keySet()) {
			double value = sequencedReadings[this.iteration+1].returnMap().get(key) - sequencedReadings[this.iteration].returnMap().get(key);
			value *= (double)timediff/20.0;
			interpRecording.addReading(key, sequencedReadings[this.iteration].returnMap().get(key)+value);
		}
	}
}
