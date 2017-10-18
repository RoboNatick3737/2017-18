package hankextensions.logging;

import java.util.ArrayList;

public class ProcessConsole
{
    public final String processName;
    public String[] processData;

    private ArrayList<ProcessConsole> parentList;

    public ProcessConsole (String processName, ArrayList<ProcessConsole> parentList)
    {
        this.processName = processName;
        processData = new String[0];

        this.parentList = parentList;
        this.parentList.add (this);
    }

    public void write(String... data)
    {
        this.processData = data;
    }

    public void destroy ()
    {
        parentList.remove (this);
    }
    public void revive ()
    {
        parentList.add (this);
    }
}