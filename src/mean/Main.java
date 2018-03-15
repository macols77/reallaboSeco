package mean;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;

public class Main {
  
  private static int TIMES = 10;
  private static int VOLTAGE = 12;
  private static int SAMPLES = 1200;

  public static void main(String[] args) {
    BufferedReader br = null;
    FileReader fr = null;
    PrintWriter writer = null;
    
    try {

      for (int i = 1; i <= VOLTAGE; i++) {
        fr = new FileReader("voltaje_"+i+".txt");
        br = new BufferedReader(fr);

        double[] mean = new double[SAMPLES+1];
        
        String sCurrentLine;
        
        int currentSample = 0;
        int p = 0;
        while ((sCurrentLine = br.readLine()) != null) {
          
          
          
          String[] values = sCurrentLine.split(" ");
          String valueToSave = values[1];
          double value = Double.parseDouble(valueToSave);
          
          mean[currentSample] = mean[currentSample] + value;
          
          if (currentSample == 1200) {
            currentSample = 0;
            p++;
          } else
            currentSample++;
          
          if (p == TIMES)
            break;
        }
        
        
        writer = new PrintWriter("trap"+i+"V_0ms600ms600ms_T1ms_ST.mean", "UTF-8");
        for (int j = 0; j < mean.length; j++) {
          writer.println(j + " " + (double)(mean[j]/TIMES));
        }
        writer.close();
      }
      
    } catch (IOException e) {

      e.printStackTrace();

    } finally {

      try {

        if (br != null)
          br.close();

        if (fr != null)
          fr.close();
        
        if (writer != null)
          writer.close();
      } catch (IOException ex) {

        ex.printStackTrace();
      }
    }
  }

}
