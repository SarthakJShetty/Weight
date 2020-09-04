#Here, we've created a script to continuosly check the state of the graphics card instead
# of keying the command over CLI over and over again.
#-Sarthak
#(28/01/2020)

#Pinging the nvidia-smi command every second until uninterrupted
nvidia-smi -q -d MEMORY,TEMPERATURE --loop-ms=100