# er_cobot_project

I use Pymycobot v3.5.3 as below. I have been working with Elephant robotic cobot 280 m5 and pi that is connected to Jetson Orin Development Kit.
# Elephant robotics
RUN git clone --branch v3.5.3  https://github.com/elephantrobotics/pymycobot.git
RUN pip3 install pyserial
RUN pip3 install crc
RUN python3 /pymycobot/setup.py install
RUN cd /pymycobot && pip3 install .
