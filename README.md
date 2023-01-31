<h1>VHF Transmitter</h1>

The VHF Transmitter is a FM Transmitter with stereo audio signal input and RDS functionality.<br> 
Main core of the application is an SPARTAN-S6 FPGA, namelly module [CMODS6](https://digilent.com/reference/programmable-logic/cmod-s6/reference-manual)
equipped with<br>customized shield for audio processing (ADC) and RF transmitting (DAC, AMPLIFIER, RF MIXER)

This project consist of the bitstream VHDL and .psm (assembler) core firmware development.

This project has default structure of the [Xilinx ISE 14.7 W10](https://www.xilinx.com/support/download/index.html/content/xilinx/en/downloadNav/vivado-design-tools/archive-ise.html).
Thus for the VHDL synthezis is XILINX ISE used.<br>However for the firmware is used KCPSM6 PicoBlaze compiler.<br><br>
<img src="/docs/systemPhoto.jpg" alt="Demo" title="Demo">

<h2>Features:</h2>
<ol>
  <li>UART controlled VHF-Transmitter system (frequency, modulation...)</li>
  <li>50Ω RF Output (up to 15dBm)</li>
  <li>Audio JACK 3.5mm audio input (including signal pre-processing for ADCs)</li>
  <li>LCD 16x2 display to display operational states</li>
  <li>5V (USB) power supply</li>
</ol>


As this project is complex, for better understanding I suggest to see:

[VHF-Transmitter Demo Video](https://www.youtube.com/watch?v=P_qsDr-8gFA&t=1s&ab_channel=Vladim%C3%ADr%C5%A0ustek)

[Block diagram](/docs/VHF_Transmitter_CompleteSystem.pdf)

[Full hardware reference and development publication](/docs/V_Sustek_VHF-Transmitter_HW_V1.03.pdf)
