# CM33 Project  

This project aims to leverage the capabilities of the **NXP-LPC5569** board, equipped with the **CM33** microprocessor, through the development of an embedded calculator. It is structured around three main areas:  

## 1. VT100 Sequences Management and FAT File System  
- Designing a communication interface with the SD card.  
- Integrating the **libc** library into **FatFS** for optimized file management.  

## 2. Communication with the LCD Screen  
- Implementing efficient communication with an LCD screen.  
- Initializing and synchronizing **core 1** with **core 0** to execute specific tasks.  
- Running display functions on **core 1**, in both **synchronous** and **asynchronous** modes.  

## 3. Sensor Data Acquisition and PowerQuad Utilization  
- Developing functionalities to retrieve data from the **MMA8652** accelerometer.  
- Managing time to ensure synchronization of operations.  
- Implementing **cosine** computation of a vector using **PowerQuad**.  
- Performing **direct and inverse Fourier transforms** for analog signal analysis.  

