package org.firstinspires.ftc.teamcode.sdkextensions.logging;

import java.util.ArrayList;

public class ProcessConsole
{
    public final String processName;
    public String[] processData;
    private ArrayList<ProcessConsole> mainList;

    public ProcessConsole (String processName, ArrayList<ProcessConsole> mainList)
    {
        this.processName = processName;
        processData = new String[0];
        this.mainList = mainList;

        this.mainList.add (this);
    }

    public void updateWith (String... processData)
    {
        this.processData = processData;
    }

    public void destroy ()
    {
        mainList.remove (this);
    }

    public void revive ()
    {
        mainList.add (this);
    }
}