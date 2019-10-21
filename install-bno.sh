sudo apt-get update
sudo apt-get install -y build-essential python-dev python-smbus python-pip git
pushd ~
git clone https://github.com/adafruit/Adafruit_Python_BNO055.git
cd Adafruit_Python_BNO055
sudo python setup.py install
popd
