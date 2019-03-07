package frc.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.BufferedReader;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.FileReader;
/**
 * Add your docs here.
 */
public class IOWithRIO {

    String path;
    public IOWithRIO(){
    
        boolean newFile = false;
        try{
            path = "/home/lvuser/CallibrationConstants.txt";
            File f = new File(path);
            if(!f.exists()){
             f.createNewFile();
             newFile = true;
            }
        } catch(IOException e){
            e.printStackTrace();
     }

        if(newFile){
            writeToRIO(3.6632863, 2.917286317, -.9538999999, -.95389999999994);
     }

    }

    public void writeToRIO(double armPotReading, double wristPotReading, double armConversion, double wristConversion){
        try{
            FileWriter fw = new FileWriter(new File(path));
            BufferedWriter bw = new BufferedWriter(fw);
            bw.write(Double.toString(armPotReading));
            bw.newLine();
            bw.write(Double.toString(wristPotReading));
            bw.newLine();
            bw.write(Double.toString(armConversion));
            bw.newLine();
            bw.write(Double.toString(wristConversion));
            bw.close();
            fw.close();
        } catch(IOException e){
            e.printStackTrace();
        }
        
    }
    

    public void readFromRIO(){
        try{
            BufferedReader br = new BufferedReader(new FileReader(path));
            String currentLine = br.readLine();
            double armPotReading = Double.parseDouble(currentLine);
            currentLine = br.readLine();
            double wristPotReading = Double.parseDouble(currentLine);
            currentLine = br.readLine();
            double armConversion = Double.parseDouble(currentLine);
            currentLine = br.readLine();
            double wristConversion = Double.parseDouble(currentLine);
            Robot.arm.flat = armPotReading;
            Robot.w.flat = wristPotReading;
            Robot.arm.conversion = armConversion;
            Robot.w.conversion = wristConversion;

            br.close();
        } catch(IOException e){
            e.printStackTrace();
        }
    }
}