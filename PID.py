"""
	Modified by : Amoghavarsha S G
	Credit: SimplePID
"""

from time import perf_counter
import threading


class PID(object):
	"""A PID controller."""

	def __init__(
		self,
		axis,
		Kp=1.0,
		Ki=0.0,
		Kd=0.0,
		setpoint=0,
		sample_time=0.01,
		output_limits=(None, None),
		auto_mode=True,
		proportional_on_measurement=False,
		error_map=None,
	):
		"""
		Initialize a new PID controller.
		:param Kp: The value for the proportional gain Kp
		:param Ki: The value for the integral gain Ki
		:param Kd: The value for the derivative gain Kd
		:param setpoint: The initial setpoint that the PID will try to achieve
		:param sample_time: The time in seconds which the controller should wait before generating
			a new output value. The PID works best when it is constantly called (eg. during a
			loop), but with a sample time set so that the time difference between each update is
			(close to) constant. If set to None, the PID will compute a new output value every time
			it is called.
		:param output_limits: The initial output limits to use, given as an iterable with 2
			elements, for example: (lower, upper). The output will never go below the lower limit
			or above the upper limit. Either of the limits can also be set to None to have no limit
			in that direction. Setting output limits also avoids integral windup, since the
			integral term will never be allowed to grow outside of the limits.
		:param auto_mode: Whether the controller should be enabled (auto mode) or not (manual mode)
		:param proportional_on_measurement: Whether the proportional term should be calculated on
			the input directly rather than on the error (which is the traditional way). Using
			proportional-on-measurement avoids overshoot for some types of systems.
		:param error_map: Function to transform the error value in another constrained value.
		"""
		self.axis = axis
		self._current_time = perf_counter
		self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
		self._setpoint = setpoint
		self.sample_time = sample_time
		self.lock = threading.Lock()

		self._auto_mode = auto_mode
		self.proportional_on_measurement = proportional_on_measurement
		self.error_map = error_map

		self._proportional = 0
		self._integral = 0
		self._derivative = 0

		self._last_time = None
		self._last_output = None
		self._last_input = None

		self.output_limits = output_limits
		self.reset()

	def __call__(self, input_):
		"""
		Update the PID controller.
		Call the PID controller with *input_* and calculate and return a control output if
		sample_time seconds has passed since the last update. If no new output is calculated,
		return the previous output instead (or None if no value has been calculated yet).
		"""

		if not self.auto_mode:
			return self._last_output

		now = self._current_time()
		
		dt = now - self._last_time if (now - self._last_time) else 1e-16
		if dt < 0:
			print(f'dt has negative value {dt}')

		if self.sample_time is not None and dt < self.sample_time and self._last_output is not None:
			# Only update every sample_time seconds
			return self._last_output

		# Compute error terms
		error = self.setpoint - input_
		d_input = input_ - (self._last_input if (self._last_input is not None) else input_)

		# Check if must map the error
		if self.error_map is not None:
			error = self.error_map(error, self.axis)

		# Compute the proportional term
		if not self.proportional_on_measurement:
			# Regular proportional-on-error, simply set the proportional term
			self._proportional = self.Kp * error
		else:
			# Add the proportional error on measurement to error_sum
			self._proportional -= self.Kp * d_input

		# Compute integral and derivative terms
		self._integral += self.Ki * error * dt
		self._integral = self.clamp(self._integral)  # Avoid integral windup

		self._derivative = -self.Kd * d_input / dt

		# Compute final output
		output = self._proportional + self._integral + self._derivative
		output = self.clamp(output)

		# Keep track of state
		self._last_output = output
		self._last_input = input_
		self._last_time = now

		return output

	def __repr__(self):
		return (
			'{self.__class__.__name__}('
			'Kp={self.Kp!r}, Ki={self.Ki!r}, Kd={self.Kd!r}, '
			'setpoint={self.setpoint!r}, sample_time={self.sample_time!r}, '
			'output_limits={self.output_limits!r}, auto_mode={self.auto_mode!r}, '
			'proportional_on_measurement={self.proportional_on_measurement!r},'
			'error_map={self.error_map!r}'
			')'
		).format(self=self)

	@property
	def components(self):
		"""
		The P-, I- and D-terms from the last computation as separate components as a tuple. Useful
		for visualizing what the controller is doing or when tuning hard-to-tune systems.
		"""
		return self._proportional, self._integral, self._derivative

	@property
	def setpoint(self):
		"""The setpoint used by the controller."""
		with self.lock:
			return self._setpoint

	@setpoint.setter
	def setpoint(self, setpoint):
		"""Set the setpoint using lock."""
		with self.lock:
			self._setpoint = setpoint

	@property
	def tunings(self):
		"""The tunings used by the controller as a tuple: (Kp, Ki, Kd)."""
		return self.Kp, self.Ki, self.Kd

	@tunings.setter
	def tunings(self, tunings):
		"""Set the PID tunings."""
		self.Kp, self.Ki, self.Kd = tunings

	@property
	def auto_mode(self):
		"""Whether the controller is currently enabled (in auto mode) or not."""
		return self._auto_mode

	@auto_mode.setter
	def auto_mode(self, enabled):
		"""Enable or disable the PID controller."""
		self.set_auto_mode(enabled)

	def set_auto_mode(self, enabled, last_output=None):
		"""
		Enable or disable the PID controller, optionally setting the last output value.
		This is useful if some system has been manually controlled and if the PID should take over.
		In that case, disable the PID by setting auto mode to False and later when the PID should
		be turned back on, pass the last output variable (the control variable) and it will be set
		as the starting I-term when the PID is set to auto mode.
		:param enabled: Whether auto mode should be enabled, True or False
		:param last_output: The last output, or the control variable, that the PID should start
			from when going from manual mode to auto mode. Has no effect if the PID is already in
			auto mode.
		"""
		if enabled and not self._auto_mode:
			# Switching from manual mode to auto, reset
			self.reset()

			self._integral = last_output if (last_output is not None) else 0
			self._integral = self.clamp(self._integral)

		self._auto_mode = enabled

	@property
	def output_limits(self):
		"""
		The current output limits as a 2-tuple: (lower, upper).
		See also the *output_limits* parameter in :meth:`PID.__init__`.
		"""
		return self._min_output, self._max_output

	@output_limits.setter
	def output_limits(self, limits):
		"""Set the output limits."""
		if limits is None:
			self._min_output, self._max_output = None, None
			return

		min_output, max_output = limits

		if (None not in limits) and (max_output < min_output):
			raise ValueError('lower limit must be less than upper limit')

		self._min_output = min_output
		self._max_output = max_output

		self._integral = self.clamp(self._integral)
		self._last_output = self.clamp(self._last_output)

	def reset(self, withI=True):
		"""
		Reset the PID controller internals.
		This sets each term to 0 as well as clearing the integral, the last output and the last
		input (derivative calculation).
		"""
		self._proportional = 0
		if withI:
			self._integral = 0
		self._derivative = 0

		self._integral = self.clamp(self._integral)

		self._last_time = self._current_time()
		self._last_output = None
		self._last_input = None

	def clamp(self, value):
		"""Clamp `value` between output limits"""
		if value is None:
			return None
		lower, upper = self.output_limits
		if (upper is not None) and (value > upper):
			return upper
		if (lower is not None) and (value < lower):
			return lower
		return value
