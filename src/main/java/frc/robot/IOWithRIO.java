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
        path = "/home/lvuser/Output.txt";
        File f = new File(path);
        if(!f.exists()){
            f.createNewFile();
            newFile = true;
        }
    } catch(IOException e){
        e.printStackTrace();
    }

    if(newFile){
        writeToRIO(3.6632863, 2.917286317);
    }

    }
    public void writeToRIO(double armPotReading, double wristPotReading){
        try{
            FileWriter fw = new FileWriter(new File(path));
            BufferedWriter bw = new BufferedWriter(fw);
            bw.write(Double.toString(armPotReading));
            bw.newLine();
            bw.write(Double.toString(wristPotReading));
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
            Robot.arm.flat = armPotReading;
            Robot.w.flat = wristPotReading;
            br.close();
        } catch(IOException e){
            e.printStackTrace();
        }
    }
}