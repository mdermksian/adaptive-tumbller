## Raspberry Pi Software
This is the raspberry pi software for the project.

Some things you'll need to do in advance on the Pi before this will work (just run these once):
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