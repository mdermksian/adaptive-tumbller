## Raspberry Pi Software
This is the raspberry pi software for the project.

### Hardware Setup
The harware setup for this is pretty simple, you'll need 3 female-to-female jumper cables.
These are the connections you need to make:
```
Raspberry PI     | Arduino
---------------------------------
Pin 12 (GPIO 18) | A4 (SDA) <-- or Sam's board SDA
Pin 35 (GPIO 19) | A5 (SCL) <-- or Sam's board SCL
Any ground pin   | Any ground pin
```
You can find a pinout diagram for the pi here: https://learn.sparkfun.com/tutorials/raspberry-gpio/gpio-pinout obviously use the diagram that matches your PI (should be the bigger header pins).

### Software Setup
Some things you'll need to do in advance on the Pi before this will work (just run these once):

Clone this repository onto the Pi:
```
sudo apt install git
git clone <URL-ADDRESS>
```
where `<URL ADDRESS>` is this repository (see clone dropdown menu)


Install python3 if not already installed! (Usually it will be though)
```
sudo apt-get update
sudo apt-get install pigpio python-pigpio python3-pigpio
sudo apt-get install libatlas-base-dev
pip3 install pigpio
pip3 install control
```

Before running the program, you need to have started the pigpio daemon:
```
sudo pigpiod
```

Then to run the script, `cd` to the folder containing `main.py` and run the following commmand:
```
python3 main.py <MODE>
```
where MODE is one of `RLS` or `ADP`

### Software Troubleshooting
If you get an error about wheels when installing scipy, run:
```
sudo apt update
sudo apt install -y python3-scipy
```



