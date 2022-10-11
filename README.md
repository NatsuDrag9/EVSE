# EVSE

The frequency of an electric power system is a non-linear function of the electricity power consumption and production. Whenever a load is connected to or disconnected from the grid, the frequency of the grid is affected. Electric vehicles that are plugged into the socket affect the grid frequency.
This disturbs the amount of current being drawn to charge the vehicle thereby leading to various unwanted consequences.

This project implements a possible solution that monitors the power system’s frequency using Arduino MKR100. The application allows the electric vehicle owner (the user) to either opt for automatic charging or manually select the charing current remotely from a web application. The current charging the vehicle is controlled by the duty cycle of PWM signal generated by the microcontroller based on the IEC61851 standard.

![Dashboard Image](../assets/outputs/Remote_Control_On_IoT_Dashboard.png?raw=true)

![Dashboard Image](../assets/outputs/Remote_Control_Off_IoT_Dashboard.png?raw=true)

![Dashboard Image](../assets/arch_and_flowchart/Schematic_Connections.png?raw=true)

![Dashboard Image](../assets/arch_and_flowchart/System_overview.jpg?raw=true)
