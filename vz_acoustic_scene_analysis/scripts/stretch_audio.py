#!/usr/bin/env python3

# Non ROS imports
from __future__ import print_function
import pyaudio
import wave
import numpy as np
from sympy import re
import usb.core
import struct
import time
import sys
import os
import pickle
from contextlib import contextmanager

# ROS specific imports 
import rospy
from rospy.numpy_msg import numpy_msg

# Custom imports
import stretch_body.hello_utils as hu
hu.print_stretch_re_use()
from vz_acoustic_scene_analysis.msg import VZ_AudioData
import NEU_VZ_ASA.MehrshadTesting.codes.A_CoughDetection.src.DSP as dsp


@contextmanager
def ignore_stderr():
    devnull = None
    try:
        devnull = os.open(os.devnull, os.O_WRONLY)
        stderr = os.dup(2)
        sys.stderr.flush()
        os.dup2(devnull, 2)
        try:
            yield
        finally:
            os.dup2(stderr, 2)
            os.close(stderr)
    finally:
        if devnull is not None:
            os.close(devnull)


# parameter list
# name: (id, offset, type, max, min , r/w, info)
PARAMETERS = {
    'AECFREEZEONOFF': (18, 7, 'int', 1, 0, 'rw', 'Adaptive Echo Canceler updates inhibit.', '0 = Adaptation enabled', '1 = Freeze adaptation, filter only'),
    'AECNORM': (18, 19, 'float', 16, 0.25, 'rw', 'Limit on norm of AEC filter coefficients'),
    'AECPATHCHANGE': (18, 25, 'int', 1, 0, 'ro', 'AEC Path Change Detection.', '0 = false (no path change detected)', '1 = true (path change detected)'),
    'RT60': (18, 26, 'float', 0.9, 0.25, 'ro', 'Current RT60 estimate in seconds'),
    'HPFONOFF': (18, 27, 'int', 3, 0, 'rw', 'High-pass Filter on microphone signals.', '0 = OFF', '1 = ON - 70 Hz cut-off', '2 = ON - 125 Hz cut-off', '3 = ON - 180 Hz cut-off'),
    'RT60ONOFF': (18, 28, 'int', 1, 0, 'rw', 'RT60 Estimation for AES. 0 = OFF 1 = ON'),
    'AECSILENCELEVEL': (18, 30, 'float', 1, 1e-09, 'rw', 'Threshold for signal detection in AEC [-inf .. 0] dBov (Default: -80dBov = 10log10(1x10-8))'),
    'AECSILENCEMODE': (18, 31, 'int', 1, 0, 'ro', 'AEC far-end silence detection status. ', '0 = false (signal detected) ', '1 = true (silence detected)'),
    'AGCONOFF': (19, 0, 'int', 1, 0, 'rw', 'Automatic Gain Control. ', '0 = OFF ', '1 = ON'),
    'AGCMAXGAIN': (19, 1, 'float', 1000, 1, 'rw', 'Maximum AGC gain factor. ', '[0 .. 60] dB (default 30dB = 20log10(31.6))'),
    'AGCDESIREDLEVEL': (19, 2, 'float', 0.99, 1e-08, 'rw', 'Target power level of the output signal. ', '[-inf .. 0] dBov (default: -23dBov = 10log10(0.005))'),
    'AGCGAIN': (19, 3, 'float', 1000, 1, 'rw', 'Current AGC gain factor. ', '[0 .. 60] dB (default: 0.0dB = 20log10(1.0))'),
    'AGCTIME': (19, 4, 'float', 1, 0.1, 'rw', 'Ramps-up / down time-constant in seconds.'),
    'CNIONOFF': (19, 5, 'int', 1, 0, 'rw', 'Comfort Noise Insertion.', '0 = OFF', '1 = ON'),
    'FREEZEONOFF': (19, 6, 'int', 1, 0, 'rw', 'Adaptive beamformer updates.', '0 = Adaptation enabled', '1 = Freeze adaptation, filter only'),
    'STATNOISEONOFF': (19, 8, 'int', 1, 0, 'rw', 'Stationary noise suppression.', '0 = OFF', '1 = ON'),
    'GAMMA_NS': (19, 9, 'float', 3, 0, 'rw', 'Over-subtraction factor of stationary noise. min .. max attenuation'),
    'MIN_NS': (19, 10, 'float', 1, 0, 'rw', 'Gain-floor for stationary noise suppression.', '[-inf .. 0] dB (default: -16dB = 20log10(0.15))'),
    'NONSTATNOISEONOFF': (19, 11, 'int', 1, 0, 'rw', 'Non-stationary noise suppression.', '0 = OFF', '1 = ON'),
    'GAMMA_NN': (19, 12, 'float', 3, 0, 'rw', 'Over-subtraction factor of non- stationary noise. min .. max attenuation'),
    'MIN_NN': (19, 13, 'float', 1, 0, 'rw', 'Gain-floor for non-stationary noise suppression.', '[-inf .. 0] dB (default: -10dB = 20log10(0.3))'),
    'ECHOONOFF': (19, 14, 'int', 1, 0, 'rw', 'Echo suppression.', '0 = OFF', '1 = ON'),
    'GAMMA_E': (19, 15, 'float', 3, 0, 'rw', 'Over-subtraction factor of echo (direct and early components). min .. max attenuation'),
    'GAMMA_ETAIL': (19, 16, 'float', 3, 0, 'rw', 'Over-subtraction factor of echo (tail components). min .. max attenuation'),
    'GAMMA_ENL': (19, 17, 'float', 5, 0, 'rw', 'Over-subtraction factor of non-linear echo. min .. max attenuation'),
    'NLATTENONOFF': (19, 18, 'int', 1, 0, 'rw', 'Non-Linear echo attenuation.', '0 = OFF', '1 = ON'),
    'NLAEC_MODE': (19, 20, 'int', 2, 0, 'rw', 'Non-Linear AEC training mode.', '0 = OFF', '1 = ON - phase 1', '2 = ON - phase 2'),
    'SPEECHDETECTED': (19, 22, 'int', 1, 0, 'ro', 'Speech detection status.', '0 = false (no speech detected)', '1 = true (speech detected)'),
    'FSBUPDATED': (19, 23, 'int', 1, 0, 'ro', 'FSB Update Decision.', '0 = false (FSB was not updated)', '1 = true (FSB was updated)'),
    'FSBPATHCHANGE': (19, 24, 'int', 1, 0, 'ro', 'FSB Path Change Detection.', '0 = false (no path change detected)', '1 = true (path change detected)'),
    'TRANSIENTONOFF': (19, 29, 'int', 1, 0, 'rw', 'Transient echo suppression.', '0 = OFF', '1 = ON'),
    'VOICEACTIVITY': (19, 32, 'int', 1, 0, 'ro', 'VAD voice activity status.', '0 = false (no voice activity)', '1 = true (voice activity)'),
    'STATNOISEONOFF_SR': (19, 33, 'int', 1, 0, 'rw', 'Stationary noise suppression for ASR.', '0 = OFF', '1 = ON'),
    'NONSTATNOISEONOFF_SR': (19, 34, 'int', 1, 0, 'rw', 'Non-stationary noise suppression for ASR.', '0 = OFF', '1 = ON'),
    'GAMMA_NS_SR': (19, 35, 'float', 3, 0, 'rw', 'Over-subtraction factor of stationary noise for ASR. ', '[0.0 .. 3.0] (default: 1.0)'),
    'GAMMA_NN_SR': (19, 36, 'float', 3, 0, 'rw', 'Over-subtraction factor of non-stationary noise for ASR. ', '[0.0 .. 3.0] (default: 1.1)'),
    'MIN_NS_SR': (19, 37, 'float', 1, 0, 'rw', 'Gain-floor for stationary noise suppression for ASR.', '[-inf .. 0] dB (default: -16dB = 20log10(0.15))'),
    'MIN_NN_SR': (19, 38, 'float', 1, 0, 'rw', 'Gain-floor for non-stationary noise suppression for ASR.', '[-inf .. 0] dB (default: -10dB = 20log10(0.3))'),
    'GAMMAVAD_SR': (19, 39, 'float', 1000, 0, 'rw', 'Set the threshold for voice activity detection.', '[-inf .. 60] dB (default: 3.5dB 20log10(1.5))'),
    # 'KEYWORDDETECT': (20, 0, 'int', 1, 0, 'ro', 'Keyword detected. Current value so needs polling.'),
    'DOAANGLE': (21, 0, 'int', 359, 0, 'ro', 'DOA angle. Current value. Orientation depends on build configuration.')
}


class Tuning:
    def __init__(self):
        self.TIMEOUT = rospy.get_param("/VZ_ACOUSTIC_SCENE_ANALYSIS/TIMEOUT_MILLISECONDS", 100000)
        self.dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)

        if not self.dev:
            raise RuntimeError("Failed to find Respeaker device")
        rospy.loginfo("Initializing Respeaker device")


    def write(self, name, value):
        try:
            data = PARAMETERS[name]
        except KeyError:
            return

        if data[5] == 'ro':
            raise ValueError('{} is read-only'.format(name))

        id = data[0]

        # 4 bytes offset, 4 bytes value, 4 bytes type
        if data[2] == 'int':
            payload = struct.pack(b'iii', data[1], int(value), 1)
        else:
            payload = struct.pack(b'ifi', data[1], float(value), 0)

        self.dev.ctrl_transfer()
        # (
        #     usb.util.CTRL_OUT | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
        #     0, 0, id, payload, self.TIMEOUT)

    def read(self, name):
        try:
            data = PARAMETERS[name]
        except KeyError:
            return

        id = data[0]

        cmd = 0x80 | data[1]
        if data[2] == 'int':
            cmd |= 0x40

        length = 8

        response = self.dev.ctrl_transfer(
            usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, cmd, id, length, self.TIMEOUT)

        response = struct.unpack(b'ii', response.tobytes())

        if data[2] == 'int':
            result = response[0]
        else:
            result = response[0] * (2.**response[1])

        return result

    def set_vad_threshold(self, db):
        self.write('GAMMAVAD_SR', db)

    def is_voice(self):
        return self.read('VOICEACTIVITY')

    @property
    def direction(self):
        return self.read('DOAANGLE')

    @property
    def version(self):
        return self.dev.ctrl_transfer(
            usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, 0x80, 0, 1, self.TIMEOUT)[0]

    def close(self):
        """
        close the interface
        """
        usb.util.dispose_resources(self.dev)


def get_respeaker_device_id():

    with ignore_stderr():
        p = pyaudio.PyAudio()
    info = p.get_host_api_info_by_index(0)
    num_devices = info.get('deviceCount')

    device_id = -1
    for i in range(num_devices):
        if (p.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
            if "ReSpeaker" in p.get_device_info_by_host_api_device_index(0, i).get('name'):
                device_id = i

    return device_id


RESPEAKER_RATE = rospy.get_param("/VZ_ACOUSTIC_SCENE_ANALYSIS/RESPEAKER_RATE", 16000)
RESPEAKER_CHANNELS = rospy.get_param("/VZ_ACOUSTIC_SCENE_ANALYSIS/RESPEAKER_CHANNELS", 6) # 6 must flash 6_channels_firmware.bin first
RESPEAKER_WIDTH = rospy.get_param("/VZ_ACOUSTIC_SCENE_ANALYSIS/RESPEAKER_WIDTH", 2)
RESPEAKER_INDEX = get_respeaker_device_id()
CHUNK = rospy.get_param("/VZ_ACOUSTIC_SCENE_ANALYSIS/CHUNK", 1024)

class ROSInterface:
    def __init__(self):
        # Initialiaze list to store audio segments
        self.wav_list = []
        self.record_count = 0 # Count how many times we've recorded f seconds of audio
        self.secs = rospy.get_param("/VZ_ACOUSTIC_SCENE_ANALYSIS/SECONDS")
        self.chunk_size = rospy.get_param("/VZ_ACOUSTIC_SCENE_ANALYSIS/CHUNK_SIZE")
        self.respeaker = Tuning()
        # Publisher for Audio Data
        self.audio_data_pub = rospy.Publisher("/wav_data", numpy_msg(VZ_AudioData), queue_size=10)
        # For Utku's code: float of cough probabilty from sample
        self.cough_prob = 90

        self.model_path = '/home/hello-robot/vz_modules/NEU_VZ_ASA/MehrshadTesting/codes/A_CoughDetection/models/'
        self.cough_classifier_file = self.model_path + 'cough_classifier'
        self.cough_classifier_scaler = self.model_path + 'cough_classification_scaler'
        self.loaded_model = pickle.load(open(self.cough_classifier_file, 'rb'))
        self.loaded_scaler = pickle.load(open(self.cough_classifier_scaler, 'rb'))



    def get_audio(self):
        recorded_frames = self.record_audio(self.chunk_size) # set param here chunk size
        self.wav_list.append(recorded_frames)
        self.record_count += 1
        # Every 5 seconds for 
        # if ((self.record_count % (self.secs/self.chunk_size)) == 0): # set param sequence size
        if (self.record_count % 3 == 0):
            return_list =  self.wav_list
            # Remove 0.2 seconds of audio data
            self.wav_list.pop(0)
            self.wav_list.pop(0)
            # send the frames at the beginning of the list (save as wav for now)
            return return_list
            


    def record_audio(self, seconds=5):
        p = pyaudio.PyAudio()

        stream = p.open(rate=RESPEAKER_RATE,
                        format=p.get_format_from_width(RESPEAKER_WIDTH),
                        channels=RESPEAKER_CHANNELS,
                        input=True,
                        input_device_index=RESPEAKER_INDEX,
                        output= False)

        frames = []
        arr_length = int(RESPEAKER_RATE / CHUNK * seconds) # get length of array to produce n seconds of audio
        for i in range(0, arr_length):
            data = stream.read(CHUNK)
            a = np.frombuffer(data,dtype=np.int16)[0::6] # extracts fused channel 0
            print("a type: ", type(a[0]))
            print("a length: ", len(a))
            self.audio_data_pub.publish(a)
            frames.append(a.tobytes())
        
        # Check to prevent empty frames list
        if (len(frames) == 0):
            # add garbage data
            print("Having issues")
            data[0] == 1
        
        stream.stop_stream()
        stream.close()
        p.terminate()

        return frames


    def process_audio_loop(self):
        dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
        try:
            if dev:
                while not rospy.is_shutdown():
                    if self.respeaker.is_voice() == 1:
                        # wav_data = list of lists of bytes
                        wav_data = self.get_audio()
                        # if wav_data is a list
                        if(isinstance(wav_data, list)):
                            # flatten list
                            flat_list = [item for sublist in wav_data for item in sublist]
                            print(type(flat_list[0]))
                            # Call of Utku's function
                            self.cough_prob = dsp.classify_cough(flat_list,RESPEAKER_RATE,self.loaded_model,self.loaded_scaler)
                            print(self.cough_prob)
                            # do something with probability (publish?)
        except usb.core.USBError:
            print('Respeaker not on USB bus')

    def cleanup(self):
        self.respeaker.close()
        print("shutting down")


if __name__ == "__main__":
    try:
        rospy.init_node("audio_capture")
        audio = ROSInterface()
        audio.process_audio_loop()
    except rospy.ROSInterruptException:
        print('Audio processing node failed!')
        rospy.on_shutdown(audio.cleanup)