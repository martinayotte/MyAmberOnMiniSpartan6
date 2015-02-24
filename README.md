This project is related to my work getting Amber Core SoftCpu working on MiniSpartan6+ board.

Unfortunately, it is only a PlayGround for now ...

Give it a try :

- Load MyAmber.xise into Xilinx ISE and compile it, it should give you the system.bit file.
- Upload this design on your MiniSpartan6+ board using the following commands :
	xc3sprog -c ftdi ~/minispartan6/spiflasherLX9.bit
	xc3sprog -c ftdi -I ~/git-work/MyAmberOnMiniSpartan6/system.bit
- Then, start a Serial Terminal such as picocom, and reset your MiniSpartan6+ board, it should give you the Amber BootLoader menu.

