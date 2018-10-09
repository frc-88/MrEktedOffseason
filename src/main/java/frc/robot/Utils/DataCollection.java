package frc.robot.Utils;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.io.*;
public class DataCollection{
private static Writer print;
private static ArrayList<BooleanSupplier> booleanSuppliers;

private static ArrayList<DoubleSupplier> doubleSuppliers;

private static ArrayList<IntSupplier> intSuppliers;

public static void initialize()throws IOException{
    booleanSuppliers=new ArrayList<>();
    doubleSuppliers=new ArrayList<>();
    intSuppliers=new ArrayList<>();

        print = new FileWriter("dataSheet.txt");
}

    public static void recordData()throws IOException{
for(BooleanSupplier supplier:booleanSuppliers){
    print.write(Boolean.toString(supplier.getAsBoolean()) +",");
}
    for(IntSupplier supplier:intSuppliers){
        print.write(Integer.toString(supplier.getAsInt()) +",");
}
for(DoubleSupplier supplier:doubleSuppliers){
    print.write(Double.toString(supplier.getAsDouble()) +",");
}
print.write("\n");
    }



public static void addBoolean(BooleanSupplier dataSupplier){
booleanSuppliers.add(dataSupplier);

}

public static void addInt(IntSupplier dataSupplier){
intSuppliers.add(dataSupplier);
    
}
public static void addDouble(DoubleSupplier dataSupplier){
doubleSuppliers.add(dataSupplier);
    
}





}