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
    File f;
    BufferedWriter bw;
    FileWriter fw;
    BufferedReader br;

    public IOWithRIO(){
    boolean newFile = false;
    try{
        f = new File("/home/lvuser/Output.txt");
        if(!f.exists()){
            f.createNewFile();
            newFile = true;
        }
        fw = new FileWriter(f);
    } catch(IOException e){
        e.printStackTrace();
    }
    bw = new BufferedWriter(fw);
    if(newFile) {
      writeToRIO(3.663286317999746, 2.917286317999828);
       
        }
    }
    public void writeToRIO(double armPotReading, double wristPotReading){
        try{
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
            br = new BufferedReader(new FileReader(f));
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