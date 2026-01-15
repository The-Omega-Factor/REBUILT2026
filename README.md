# Omega Factor 2026 Robot's Program

## Scope:

- ### Teleop

    - **Swerve** drivetrain have **Field-Centric** control
    - Drivetrain have automatic heading error correction using **April Tags** and **Limelight**
    - **Intake** has a upward and downward limiter
    - **Intake** is programmed to a button
    - **AUTOMATIC Shooter Speed adjustments**
    - **PathPlanner** localization with **LimeLight Realignment** programmed to a button
    - Nearly automatic shooting  
    This means driver only need to *get into the alliance zone and press a button.*  
    After the button is pressed, the robot will automatically turn (in place) to the Hub and score

    #### Manual Override for all Autonomous Movements

- ### Autonomous
    Use **Limelight** for most of localization and **switch to PathPlanner whenever April Tags is out of view**.  

    ### Paths

    *All Paths will start with shooting the 8 preload Fuels.* 

    ***3 Main Paths***  

    1. Robot Starts near the **DEPOT**  
        **1.1** Collect Fuels from Depot  
        **1.2**  Shoot Fuels  
        **1.3** Repeat  

    2. Robot Starts in the **MIDDLE**  
        **2.1** Ascend Level 2.  
        This lets teammates ascend level 1 easier

    3. Robot Starts near the **HUMAN PLAYER**  
        **3.1** Collect Fuels from the Human Player  
        **3.2** Shoot the Fuels  
        **3.3** Repeat


#### Note for Team

Whenever you start working, use:  
git pull origin main  

At the end of the day, or whenever a milestone is reached. Use these commands 
in order in the terminal:  

**1.** git add .  
**2.** git commit -m "Put some message here"  
**3.** git push origin main  

Check GitHub to make sure the commit have went through