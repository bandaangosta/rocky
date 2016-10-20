# Copyright 2011, Google Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
#     * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following disclaimer
# in the documentation and/or other materials provided with the
# distribution.
#     * Neither the name of Google Inc. nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
import math
import serial
import subprocess
from espeak import espeak

_GOODBYE_MESSAGE = u'Goodbye'
MAX_SPEED = 150

# Convert from cartesian to polar coordinates
# Returns r, theta(degrees)
def polar(x, y):
    return math.hypot(x,y),math.degrees(math.atan2(y,x))

def polarToRobotCommands(r, theta):
    if r == 0.0:
        return 'stop', 'stop'

    if r > MAX_SPEED / 2:
        speed = 'fast'
    else:
        speed = 'slow'

    if theta >= (90 - 22.5) and theta <= (90 + 22.5):
        direction = 'front'
    elif theta < (90 - 22.5) and theta > (22.5):
        direction = 'front-right'
    elif theta < (22.5) and theta > (-22.5):
        direction = 'spin-right'
    elif theta > (90 + 22.5) and theta < (180 - 22.5):
        direction = 'front-left'
    elif theta > (180 - 22.5) and theta < (180 + 22.5):
        direction = 'spin-left'
    else:
        direction = 'reverse'

    return speed, direction

def web_socket_do_extra_handshake(request):
    # This example handler accepts any request. See origin_check_wsh.py for how
    # to reject access from untrusted scripts based on origin value.

    pass  # Always accept.


def web_socket_transfer_data(request):
    try:
        espeak.set_voice('spanish-latin-am')
        espeak.set_parameter(espeak.Parameter.Rate, 150)

        #arduino = serial.Serial('/dev/ttyACM0', 9600, timeout=3)
        arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

        while True:
            line = request.ws_stream.receive_message()
            if line is None:
                return

            if arduino.inWaiting() > 0:
                sensorWord = arduino.readline()
                request.ws_stream.send_message("$" + sensorWord)

            if isinstance(line, unicode):
                dx, dy, sound = line.split(';')
                r, theta = polar(float(dx), float(dy))
                speed, direction = polarToRobotCommands(r, theta)

                request.ws_stream.send_message(' / '.join(map(lambda x:str(x), [dx, dy, round(r), round(theta, 1), speed, direction, sound])), binary=False)

                if sound == 'd':
                    subprocess.Popen(["aplay", "/home/pi/audio/dog_bark.wav"])
                elif sound == 'h':
                    subprocess.Popen(["aplay", "/home/pi/audio/Ahooga_Car_Horn-SoundBible.com-1499602683.wav"])
                elif sound == 'L':
                    subprocess.Popen(["aplay", "/home/pi/audio/LucasGangnamClip.wav"])
                elif sound == 'm':
            #os.system("amixer set PCM -- 100%")
                    #subprocess.Popen(["amixer", "set", "PCM", "--", "100%", ";", "aplay", "/home/pi/audio/toodaloo.wav", ";", "amixer", "set", "PCM", "--", "80%"])
                    #subprocess.Popen(["aplay", "/home/pi/audio/toodaloo.wav"])
                    subprocess.Popen(["/bin/sh", "/home/pi/audio/play_toodaloo.sh"])
            #os.system("amixer set PCM -- 80%")
                elif sound == 't':
                    arduino.write('e')
                    arduino.write('f')
                elif sound.startswith('$'):
                    message = sound.split('$')
                    if(message[1] == 'shutdown'):
                        os.system('sudo shutdown now')
                    elif(message[1] == 'reboot'):
                        os.system('sudo reboot')
                    elif(message[1] == 'close_grip'):
                        arduino.write('g')
                    elif(message[1] == 'open_grip'):
                        arduino.write('h')
                    else:
                        espeak.synth(message[1])
                else:
                    if direction == 'spin-right':
                        arduino.write('9')
                    elif direction == 'spin-left':
                        arduino.write('8')
                    elif direction == 'front' and speed == 'fast':
                        arduino.write('1')
                    elif direction == 'front' and speed == 'slow':
                        arduino.write('2')
                    elif direction == 'reverse' and speed == 'fast':
                        arduino.write('5')
                    elif direction == 'reverse' and speed == 'slow':
                        arduino.write('4')
                    elif direction == 'front-right':
                        arduino.write('7')
                    elif direction == 'front-left':
                        arduino.write('6')
                    else:
                        arduino.write('3')

                    # request.ws_stream.send_message('RPI received: %s' % line, binary=False)
            else:
                request.ws_stream.send_message(line, binary=True)
    finally:
        if arduino.isOpen():
            arduino.close()

# vi:sts=4 sw=4 et
