#!/usr/bin/python3
import rospy
import numpy
import scipy.spatial
from matplotlib import pyplot
from hdl_localization.msg import *


class Plotter(object):
	def __init__(self):
		pyplot.ion()
		pyplot.show(block=False)

		self.status_buffer = []
		self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
		self.status_sub = rospy.Subscriber('/status', ScanMatchingStatus, self.status_callback)

	def status_callback(self, status_msg):
		self.status_buffer.append(status_msg)
		if len(self.status_buffer) > 300:
			self.status_buffer = self.status_buffer[-300:]

	def timer_callback(self, event):
		if len(self.status_buffer) < 2:
			return

		errors = {}
		for status in self.status_buffer:
			for label, error in zip(status.prediction_labels, status.prediction_errors):
				if label.data not in errors:
					errors[label.data] = []
				
				trans = [error.translation.x, error.translation.y, error.translation.z]
				quat = [error.rotation.x, error.rotation.y, error.rotation.z, error.rotation.w]

				t = status.header.stamp.secs + status.header.stamp.nsecs / 1e9
				t_error = numpy.linalg.norm(trans)
				r_error = numpy.linalg.norm(scipy.spatial.transform.Rotation.from_quat(quat).as_rotvec())
				errors[label.data].append((t, t_error, r_error))

		pyplot.clf()
		for label in errors:
			errs = numpy.float64(errors[label])
			pyplot.subplot(211)
			pyplot.ylabel('trans error')
			pyplot.plot(errs[:, 0], errs[:, 1], label=label)

			pyplot.subplot(212)
			pyplot.ylabel('rot error')
			pyplot.plot(errs[:, 0], errs[:, 2], label=label)

		pyplot.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), ncol=len(errors))
		pyplot.gcf().canvas.flush_events()

def main():
	rospy.init_node('status_plotter')
	node = Plotter()
	rospy.spin()

if __name__ == '__main__':
	main()
