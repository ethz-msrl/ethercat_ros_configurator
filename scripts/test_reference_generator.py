#  Copyright (c) 2024, Multi-Scale Robotics Lab - ETH Zürich
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
# 
#  1. Redistributions of source code must retain the above copyright notice, this
#     list of conditions and the following disclaimer.
# 
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
# 
#  3. Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
# 
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.



# This file exposes a ros node to create test reference trajectories for 
# the EtherCAT ROS Configurator. This node takes as intput a series of 
# ROS parameters specifying the command topic to publish to and the signal
# to generate. Stitching multiple signals in a time series of cascaded
# varying signals is not supported yet. For plotting it is suggested to
# store data in ROSbag and use rqt_multiplot or other plotting tools as
# required.

import rospy
import time
import threading
import datetime
import pickle
import numpy as np
import scipy.signal as signal
import matplotlib.pyplot as plt

from ethercat_motor_msgs.msg import MotorCtrlMessage
from ethercat_motor_msgs.msg import MotorStatusMessage

thread_abort = False
def abort_thread():
    global thread_abort
    thread_abort = True

class TestReferenceGenerator:
    def __init__(self):
        # Get the command topic to publish to
        self.command_topic = rospy.get_param('~command_topic')
        self.status_topic = rospy.get_param('~status_topic')
        self.name = rospy.get_param('~name', 'default_name')
        self.signal = rospy.get_param('~signal', 'sine')
        self.frequency = rospy.get_param('~frequency', 1.0)
        self.amplitude = rospy.get_param('~amplitude', 1.0)
        self.amplitude_limit_min = rospy.get_param('~amplitude_limit_min', -1.0)
        self.amplitude_limit_max = rospy.get_param('~amplitude_limit_max', 1.0)
        self.offset = rospy.get_param('~offset', 0.0)
        self.phase_shift = rospy.get_param('~phase_shift', 0.0)
        self.duration = rospy.get_param('~duration', 10.0)
        self.sample_rate = rospy.get_param('~sample_rate', 100.0)
        self.plot = rospy.get_param('~plot', True)
        self.plot_dpi = rospy.get_param('~plot_dpi', 300)
        self.save_data = rospy.get_param('~save_data', False)
        self.save_path = rospy.get_param('~save_data_path') + "/"
        self.command_pub = rospy.Publisher(self.command_topic, MotorCtrlMessage, queue_size=10)
        self.status_sub = rospy.Subscriber(self.status_topic, MotorStatusMessage, self.status_callback)
        self.command_msg = MotorCtrlMessage()
        self.time_vec = np.zeros(int(self.duration*self.sample_rate))
        self.signal_vec = np.zeros(int(self.duration*self.sample_rate))
        self._sample_number = 0
        self._current_time = self.time_vec[0]
        self._current_sample = 0
        self._current_signal_generator = None
        self._current_reading_time = 0.0
        self._initial_reading_time = 0.0
        self._reading_vec = []
        self._reading_times = []
        self._data_prefix = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.publishing_on = False
        self._current_reading_number = 0
        self._mount_signal_generator()
    
    def _sine_wave(self, t):
        self._current_sample = self.amplitude*np.sin(2.0*np.pi*self.frequency*t + self.phase_shift) + self.offset
        self._current_sample = np.clip(self._current_sample, self.amplitude_limit_min, self.amplitude_limit_max)

    def _square_wave(self, t):
        self._current_sample = self.amplitude*signal.square(2.0*np.pi*self.frequency*t + self.phase_shift, 0.5) + self.offset
        self._current_sample = np.clip(self._current_sample, self.amplitude_limit_min, self.amplitude_limit_max)
    
    def _triangle_wave(self, t):
        self._current_sample = self.amplitude*signal.sawtooth(2.0*np.pi*self.frequency*t + self.phase_shift, 0.5) + self.offset
        self._current_sample = np.clip(self._current_sample, self.amplitude_limit_min, self.amplitude_limit_max)

    def _sawtooth_wave(self, t):
        self._current_sample = self.amplitude*signal.sawtooth(2.0*np.pi*self.frequency*t + self.phase_shift, 1.0) + self.offset
        self._current_sample = np.clip(self._current_sample, self.amplitude_limit_min, self.amplitude_limit_max)
        
    def _smoothstep(self, t):
        # Perlin's 2nd order smoothstep function
        g3 = 10
        g4 = -15
        g5 = 6
        t_nrm = t / self.duration
        self._current_sample = self.amplitude*(g3*t_nrm**3 + g4*t_nrm**4 + g5*t_nrm**5) + self.offset
        self.signal_vec[self._sample_number] = self._current_sample
        self._sample_number += 1
    
    def _mount_signal_generator(self):
        if self.signal == 'sine':
            self._current_signal_generator = self._sine_wave
        elif self.signal == 'square':
            self._current_signal_generator = self._square_wave
        elif self.signal == 'triangle':
            self._current_signal_generator = self._triangle_wave
        elif self.signal == 'sawtooth':
            self._current_signal_generator = self._sawtooth_wave
        elif self.signal == 'smoothstep':
            self._current_signal_generator = self._smoothstep
        else:
            rospy.logerr('Signal type not supported')
            raise ValueError('Signal type not supported')
    
    def _update_command_msg(self):
        # TODO: Add support for velocity and torque
        self.command_msg.header.stamp = rospy.Time.now()
        self.command_msg.targetPosition = int(self._current_sample)
        self.command_msg.operationMode = MotorCtrlMessage.NANOTEC_OPERATION_MODE_CYCLIC_SYNCHRONOUS_POSITION
        # Same as MotorCtrlMessage.MAXON_EPSO4_OPERATION_MODE_CYCLIC_SYNCHRONOUS_POSITION

    def publish_command_thread(self):
        rate = rospy.Rate(self.sample_rate)
        t_start = time.time()
        self.publishing_on = True
        while not rospy.is_shutdown():
            self._current_time = time.time() - t_start
            self._current_signal_generator(self._current_time)
            self.signal_vec[self._sample_number] = self._current_sample
            self.time_vec[self._sample_number] = self._current_time
            print("Current time: ", self._current_time)
            self._sample_number += 1
            if self._current_time > self.duration:
                break
            elif self._sample_number == self.time_vec.shape[0]:
                break
            elif thread_abort:
                break
            self._update_command_msg()
            self.command_pub.publish(self.command_msg)
            rate.sleep()
        
        # Plot if required
        print("Final sampling time: ", self._current_time)
        print("Final sampling number: ", self._sample_number)
        print("Final reading time: ", self._current_reading_time)
        print("Final reading number: ", self._current_reading_number)

        if self.save_data:
            self.save_data_to_file()

        if self.plot:
            self.plot_data()
        
        self.publishing_on = False
        
    def status_callback(self, msg):
        if not self.publishing_on:
            return
        if self._current_reading_number == 0:
            self._initial_reading_time = time.time()
        self._current_reading_time = time.time() - self._initial_reading_time
        self._reading_vec.append(msg.actualPosition)
        self._reading_times.append(self._current_reading_time)
        self._current_reading_number += 1
    
    def save_data_to_file(self):
        # Generate the filename with the current date and time prefix
        filename = self.save_path + f"{self.name}_{self._data_prefix}_data.pkl"

        # Create a dictionary to store the data
        data = {
            "commands":{
                "samples": self.signal_vec,
                "time_vectors": self.time_vec
            },
            "readings": {
                "times": self._reading_times,
                "values": self._reading_vec
            },
            "signal": self.signal,
            "frequency": self.frequency,
            "amplitude": self.amplitude,
            "amplitude_limit_min": self.amplitude_limit_min,
            "amplitude_limit_max": self.amplitude_limit_max,
            "offset": self.offset,
            "phase_shift": self.phase_shift,
            "duration": self.duration,
            "sample_rate": self.sample_rate,
            "command_topic": self.command_topic,
            "status_topic": self.status_topic
        }

        # Save the data to the pkl file
        with open(filename, "wb") as file:
            pickle.dump(data, file)

        print(f"Data saved to {filename}")
    
    def plot_data(self):
        plt.figure()
        reading_times = np.array(self._reading_times)
        reading_vec = np.array(self._reading_vec)
        plt.title(f"{self.signal}, f={self.frequency}, A={self.amplitude}, fs={self.sample_rate}, offset={self.offset}, phase={self.phase_shift}, duration={self.duration} \
                  \n amplitude limit: [{self.amplitude_limit_min}, {self.amplitude_limit_max}] \
                  \n command topic: {self.command_topic}")
        plt.suptitle('Reference Generator Test')
        plt.plot(self.time_vec, self.signal_vec, label='Command')
        plt.plot(reading_times, reading_vec, label='Reading')
        plt.xlabel('Time [s]')
        plt.ylabel('Position [motor controller units]')
        plt.legend()
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        fig = plt.gcf()
        fig.set_size_inches(18.5, 5.5)
        if self.save_data:
            plt.savefig(self.save_path + f"{self.name}_{self._data_prefix}_plot.png", dpi=self.plot_dpi)
            print(f"Plot saved to {self.save_path + f'{self.name}_{self._data_prefix}_plot.png'}")
        plt.show()

if __name__ == '__main__':
    rospy.init_node('test_reference_generator')
    rospy.on_shutdown(abort_thread)
    reference_generator = TestReferenceGenerator()
    command_thread = threading.Thread(target=reference_generator.publish_command_thread)
    command_thread.start()
    rospy.spin()